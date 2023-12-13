# Information: https://clover.coex.tech/programming

import rospy
import cv2
import typing
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
from clover import srv
import numpy as np
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# coder BGR to HSV: https://colordesigner.io/convert/rgbtohsv
# color for bright red
# lower_red = np.array([0, 0, 80])  # hsv(0,100,31)
# upper_red = np.array([40, 40, 255])  # hsv(0,84,100)
# color for dark red
lower_red = np.array([0, 0, 140])  # red
upper_red = np.array([115, 115, 255])
lower_blue = np.array([170, 0, 0])  # blue
upper_blue = np.array([225, 170, 225])
lower_green = np.array([0, 190, 0])  # green
upper_green = np.array([175, 255, 210])
lower_yellow = np.array([0, 205, 105])  # yellow
upper_yellow = np.array([160, 255, 255])

rospy.init_node('flight')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image)


class CircleData:
    def __init__(self, x_px: int, y_px: int) -> None:
        self.x_px: int = x_px
        self.y_px: int = y_px
        self.x_metrs: typing.Optional[float] = None
        self.y_metrs: typing.Optional[float] = None

    def __str__(self) -> str:
        return str("{},{} -> {},{} ".format(
               self.x_px,
               self.y_px,
               '{:.2f}'.format(self.x_metrs) if self.x_metrs is not None else 'None',
               '{:.2f}'.format(self.y_metrs) if self.y_metrs is not None else 'None'))


g_image_width: typing.Optional[int] = None
g_image_height: typing.Optional[int] = None
g_image_size_known: bool = False
g_markers = None
g_circles: typing.Optional[typing.List[CircleData]] = None


def calc_circle_metr(aruco_metr, aruco_px, drone_px, circle_px) -> typing.Optional[float]:
    if aruco_px == drone_px:
        if aruco_px == circle_px:
            circle_metr = 0.0
        else:
            circle_metr = None
    else:
        drone_to_circle_px = circle_px - drone_px
        drone_to_aruco_px = aruco_px - drone_px
        #if abs(drone_to_circle_px) < 5 or abs(drone_to_aruco_px) < 5 or abs(aruco_metr) < 0.01:
        #    circle_metr = None
        #else:
        circle_metr = (drone_to_circle_px * aruco_metr) / drone_to_aruco_px
    return circle_metr


def calc_circle_offset(  # x_circle_metr, y_circle_metr
        x_aruco_metr, y_aruco_metr, x_aruco_px, y_aruco_px,
        x_drone_px, y_drone_px,
        x_circle_px, y_circle_px) -> typing.Tuple[typing.Optional[float], typing.Optional[float]]:
    x_circle_metr: typing.Optional[float] = calc_circle_metr(x_aruco_metr, x_aruco_px, x_drone_px, x_circle_px)
    y_circle_metr: typing.Optional[float] = calc_circle_metr(y_aruco_metr, y_aruco_px, y_drone_px, y_circle_px)
    return x_circle_metr, y_circle_metr


def calc_marker_center_px(c1, c2, c3, c4) -> typing.Tuple[int, int]:
    max_x: int = max(c1.x, c2.x, c3.x, c4.x)
    max_y: int = max(c1.y, c2.y, c3.y, c4.y)
    min_x: int = min(c1.x, c2.x, c3.x, c4.x)
    min_y: int = min(c1.y, c2.y, c3.y, c4.y)
    x_aruco_px: int = (max_x - min_x) // 2 + min_x
    y_aruco_px: int = (max_y - min_y) // 2 + min_y
    return x_aruco_px, y_aruco_px


def calc_circles_center_px(contours) -> typing.List[CircleData]:
    circles: typing.List[CircleData] = []
    for c in contours:
        min_x, min_y = max_x, max_y = c[0][0]
        for ci in c:
            x,y = ci[0]
            if x < min_x: min_x = x
            elif x > max_x: max_x = x
            if y < min_y: min_y = y
            elif y > max_y: max_y = y
        #  fill
        x_px = (max_x-min_x)//2+min_x
        y_px = (max_y-min_y)//2+min_y
        circle = CircleData(x_px, y_px)
        circles.append(circle)
    return circles


def calc_circles_center_metr(circles: typing.List[CircleData], markers):
    global g_image_width, g_image_height
    x_drone_px, y_drone_px = g_image_width // 2, g_image_height // 2
    for c in circles:
        x_circle_px: int = c.x_px
        y_circle_px: int = c.y_px
        x_circle_metrs: typing.List[float] = []
        y_circle_metrs: typing.List[float] = []
        for m in markers:
            x_aruco_metr: int = m.pose.position.x
            y_aruco_metr: int = m.pose.position.y
            x_aruco_px, y_aruco_px = calc_marker_center_px(m.c1, m.c2, m.c3, m.c4)
            circle_metr = calc_circle_offset(
                x_aruco_metr, y_aruco_metr, x_aruco_px, y_aruco_px,
                x_drone_px, y_drone_px,
                x_circle_px, y_circle_px)
            # calc average of x and y of circle
            # 82 : (-0.7601148629662632, -0.12342062177803896)
            # 92 : (-0.6897661848135627, None)
            # 82 : (-0.51978442835193, -0.13464067830331522)
            # 92 : (-0.471678346968098, None)
            # 82 : (-0.4583045497296587, -0.24123121529343977)
            # 92 : (-0.41588843496111866, None)
            # 82 : (-0.6539223453459764, -0.3309916674956499)
            # 92 : (-0.593401791346962, None)
            if circle_metr[0] is not None:
                # check incorrect offsets (if meters too big)
                if abs(circle_metr[0]) < 10.0:
                    x_circle_metrs.append(circle_metr[0])
            if circle_metr[1] is not None:
                # check incorrect offsets (if meters too big)
                if abs(circle_metr[1]) < 10.0:
                    y_circle_metrs.append(circle_metr[1])
            # print(m.id, ':', circle_metr)
        if x_circle_metrs and y_circle_metrs:
            x_avg: float = sum(x_circle_metrs) / len(x_circle_metrs)
            y_avg: float = sum(y_circle_metrs) / len(y_circle_metrs)
            # print('X, Y:', x_avg, y_avg)
            c.x_metrs = x_avg
            c.y_metrs = y_avg


@long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    global g_image_size_known, g_image_width, g_image_height, g_circles
    if not g_image_size_known:
        g_image_size_known = True
        g_image_width = data.width
        g_image_height = data.height
        print("Camera size: ", g_image_width, g_image_height)
    frame = cv2.inRange(img, lower_red, upper_red)
    contours, _hierarchy = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        g_circles = None
    else:
        # circle centers (px)
        circles: typing.List[CircleData] = calc_circles_center_px(contours)
        # circle centers (meters)
        global g_markers
        if g_markers is not None and g_image_size_known:
            markers = g_markers.copy()
            # print('----------')
            calc_circles_center_metr(circles, markers)
            g_circles = circles.copy()
            for c in g_circles: print(c, end="\t")
            print()
        # draw
        cv2.drawContours(img, contours, -1, (0,255,0), 3)
        for c in circles:
            cv2.circle(img, (c.x_px, c.y_px), 3, (0,255,0), -1)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)


@long_callback
def markers_callback(msg):
    # print(msg)
    global g_markers
    if msg.markers is None:
        g_markers = None
    else:
        g_markers = msg.markers.copy()

rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)

print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

# Wait for 5 seconds
rospy.sleep(5)

print('Fly forward 1 m')
navigate(x=0, y=0, z=1, frame_id='aruco_92')

# Wait for 8 seconds
rospy.sleep(8)
if g_circles:
    circles = g_circles.copy()
    # first random circle
    for c in circles:
        if c.x_metrs is None or c.y_metrs is None:
            continue
        navigate(y=-c.x_metrs, x=-c.y_metrs, z=0, frame_id='body')
        rospy.sleep(10)
        navigate(y=c.x_metrs, x=c.y_metrs, z=0, frame_id='body')
        rospy.sleep(10)
        break

print('Perform landing')
land()


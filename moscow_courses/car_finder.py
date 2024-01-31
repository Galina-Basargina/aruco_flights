# Information: https://clover.coex.tech/programming

import rospy
import cv2
import typing
import math
from enum import Enum
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#from clover import long_callback
from clover import srv
import numpy as np
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray
from clover.srv import SetLEDEffect

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service


def navigate_wait(x=0.0, y=0.0, z=0.0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        return res
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)


rospy.init_node('flight')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image)


class Color(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3
    YELLOW = 4


class ColorCircle:
    def __init__(self, color: Color, max_items: int, lower, upper, contour):
        self.color: Color = color
        self.max_items: int = max_items
        self.lower = lower
        self.upper = upper
        self.contour = contour


# coder BGR to HSV: https://colordesigner.io/convert/rgbtohsv
#g_darkred_circle: ColorCircle = ColorCircle(  # bgr8
#    Color.RED, 2,
#    np.array([0, 0, 115]),
#    np.array([115, 115, 255]),
#    (220, 39, 214))
g_red_circle: ColorCircle = ColorCircle(  # bgr8
    Color.RED, 2,
    np.array([29, 29, 51]),
    np.array([88, 89, 255]),
    (220, 39, 214))
g_blue_circle: ColorCircle = ColorCircle(  # bgr8
    Color.BLUE, 1,
    np.array([30, 28, 0]),
    np.array([124, 124, 52]),
    (255, 225, 0))
g_green_circle: ColorCircle = ColorCircle(  # bgr8
    Color.GREEN, 1,
    np.array([0, 190, 0]),
    np.array([190, 255, 190]),
    (0, 67, 35))
g_yellow_circle: ColorCircle = ColorCircle(  # bgr8
    Color.YELLOW, 1,
    np.array([0, 70, 70]),
    np.array([55, 255, 255]),
    (0, 137, 255))

g_colors: typing.List[ColorCircle] = [
    g_red_circle,
    g_blue_circle,
    g_green_circle,
    g_yellow_circle,
]


class CircleData:
    def __init__(self, color: ColorCircle, x_px: int, y_px: int, aruco_id: typing.Optional[int], aruco_pose, weight_x_px: int, weight_y_px: int) -> None:
        self.color: ColorCircle = color
        self.x_px: int = x_px
        self.y_px: int = y_px
        self.x_metrs: typing.Optional[float] = None
        self.y_metrs: typing.Optional[float] = None
        self.aruco_id: typing.Optional[int] = aruco_id
        self.aruco_pose = aruco_pose  # x,y
        self.weight: int = weight_x_px * weight_y_px
        self.weight_x_px: int = weight_x_px
        self.weight_y_px: int = weight_y_px
        self.x_weight_metr: typing.Optional[float] = None
        self.y_weight_metr: typing.Optional[float] = None

    def __str__(self) -> str:
        return str("{}#{} {}x,{}y -> {}dx,{}dy {} weight ".format(
            self.color.color,
            self.aruco_id,
            self.x_px,
            self.y_px,
            '{:.2f}'.format(self.x_metrs) if self.x_metrs is not None else 'None',
            '{:.2f}'.format(self.y_metrs) if self.y_metrs is not None else 'None',
            self.weight))


g_image_width: typing.Optional[int] = None
g_image_height: typing.Optional[int] = None
g_image_size_known: bool = False
g_markers = None
g_circles: typing.Optional[typing.List[CircleData]] = None
g_drone_searching: bool = False
# found circles
g_yellow_found_circle: CircleData = None
g_red_found_circle: CircleData = None
g_blue_found_circle: CircleData = None
g_green_found_circle: CircleData = None


def calc_circle_metr(aruco_metr, aruco_px, drone_px, circle_px, circle_length_px) -> typing.Tuple[typing.Optional[float], typing.Optional[float]]:
    if aruco_px == drone_px:
        if aruco_px == circle_px:
            circle_metr = 0.0
            circle_length_metr = None
        else:
            circle_metr = None
            circle_length_metr = None
    else:
        drone_to_circle_px = circle_px - drone_px
        drone_to_aruco_px = aruco_px - drone_px
        # if abs(drone_to_circle_px) < 5 or abs(drone_to_aruco_px) < 5 or abs(aruco_metr) < 0.01:
        #    circle_metr = None
        # else:
        circle_metr = (drone_to_circle_px * aruco_metr) / drone_to_aruco_px
        circle_length_metr = (circle_length_px * aruco_metr) / drone_to_aruco_px
    return circle_metr, circle_length_metr


def calc_circle_offset(  # x_circle_metr, y_circle_metr, x_circle_length_metr, y_circle_length_metr
        x_aruco_metr, y_aruco_metr, x_aruco_px, y_aruco_px,
        x_drone_px, y_drone_px,
        x_circle_px, y_circle_px,
        x_circle_length_px, y_circle_length_px) -> typing.Tuple[typing.Optional[float], typing.Optional[float], typing.Optional[float], typing.Optional[float]]:
    x_circle_metr, x_circle_length_metr = calc_circle_metr(x_aruco_metr, x_aruco_px, x_drone_px, x_circle_px, x_circle_length_px)
    y_circle_metr, y_circle_length_metr = calc_circle_metr(y_aruco_metr, y_aruco_px, y_drone_px, y_circle_px, y_circle_length_px)
    return x_circle_metr, y_circle_metr, x_circle_length_metr, y_circle_length_metr


def calc_marker_center_px(c1, c2, c3, c4) -> typing.Tuple[int, int]:
    max_x: int = max(c1.x, c2.x, c3.x, c4.x)
    max_y: int = max(c1.y, c2.y, c3.y, c4.y)
    min_x: int = min(c1.x, c2.x, c3.x, c4.x)
    min_y: int = min(c1.y, c2.y, c3.y, c4.y)
    x_aruco_px: int = (max_x - min_x) // 2 + min_x
    y_aruco_px: int = (max_y - min_y) // 2 + min_y
    return x_aruco_px, y_aruco_px


def calc_circles_center_px(color: ColorCircle, contours, aruco_id: typing.Optional[int], aruco_pose) -> typing.List[CircleData]:
    circles: typing.List[CircleData] = []
    for c in contours:
        min_x, min_y = max_x, max_y = c[0][0]
        for ci in c:
            x, y = ci[0]
            if x < min_x:
                min_x = x
            elif x > max_x:
                max_x = x
            if y < min_y:
                min_y = y
            elif y > max_y:
                max_y = y
        #  fill
        x_px = (max_x - min_x) // 2 + min_x
        y_px = (max_y - min_y) // 2 + min_y
        circle = CircleData(color, x_px, y_px, aruco_id, aruco_pose, max_x - min_x, max_y - min_y)
        circles.append(circle)
    return circles


def calc_circles_center_metr(circles: typing.List[CircleData], markers):
    global g_image_width, g_image_height
    x_drone_px, y_drone_px = g_image_width // 2, g_image_height // 2
    for c in circles:
        x_circle_px: int = c.x_px
        y_circle_px: int = c.y_px
        x_circle_length_px: int = c.weight_x_px
        y_circle_length_px: int = c.weight_y_px
        x_circle_metrs: typing.List[float] = []
        y_circle_metrs: typing.List[float] = []
        x_circle_length_metrs: typing.List[float] = []
        y_circle_length_metrs: typing.List[float] = []
        for m in markers:
            x_aruco_metr: float = m.pose.position.x
            y_aruco_metr: float = m.pose.position.y
            x_aruco_px, y_aruco_px = calc_marker_center_px(m.c1, m.c2, m.c3, m.c4)
            circle_metr = calc_circle_offset(  # x_circle_metr, y_circle_metr, x_circle_length_metr, y_circle_length_metr
                x_aruco_metr, y_aruco_metr, x_aruco_px, y_aruco_px,
                x_drone_px, y_drone_px,
                x_circle_px, y_circle_px,
                x_circle_length_px, y_circle_length_px)
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
            if circle_metr[2] is not None:
                x_circle_length_metrs.append(circle_metr[2])
            if circle_metr[3] is not None:
                y_circle_length_metrs.append(circle_metr[3])
            # print(m.id, ':', circle_metr)
        if x_circle_metrs and y_circle_metrs:
            x_avg: float = sum(x_circle_metrs) / len(x_circle_metrs)
            y_avg: float = sum(y_circle_metrs) / len(y_circle_metrs)
            # print('X, Y:', x_avg, y_avg)
            c.x_metrs = x_avg
            c.y_metrs = y_avg
        if x_circle_length_metrs and y_circle_length_metrs:
            x_avg: float = sum(x_circle_length_metrs) / len(x_circle_length_metrs)
            y_avg: float = sum(y_circle_length_metrs) / len(y_circle_length_metrs)
            # print('X, Y:', x_avg, y_avg)
            c.x_weight_metr = x_avg
            c.y_weight_metr = y_avg


# @long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    # calc camera/drone position
    global g_image_size_known, g_image_width, g_image_height, g_drone_searching
    if not g_image_size_known:
        g_image_size_known = True
        g_image_width = data.width
        g_image_height = data.height
        print("Camera size: ", g_image_width, g_image_height)
    if g_drone_searching:
        # markers
        markers = None
        circles_all = []
        # cycle by colors
        global g_colors
        for color in g_colors:
            frame = cv2.inRange(img, color.lower, color.upper)
            contours, _hierarchy = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # calculations
            if not contours:
                continue
            if len(contours) > color.max_items:
                continue
            # aruco number?
            global g_markers
            aruco_id: typing.Optional[int] = None
            aruco_pose = None
            if g_markers is not None and len(g_markers) > 0:
                aruco_id = g_markers[0].id
                aruco_pose = g_markers[0].pose.position
            # circle centers (px)
            circles: typing.List[CircleData] = calc_circles_center_px(color, contours, aruco_id, aruco_pose)
            # circle centers (meters)
            if g_markers is not None and g_image_size_known:
                if markers is None:  # copy only first time
                    markers = g_markers.copy()
                # modify circles
                calc_circles_center_metr(circles, markers)
                # store new circles to all circles
                circles_all.extend(circles)
        # debug (print)
        global g_circles
        if circles_all:
            g_circles = circles_all.copy()
            # redraw
            # cv2.drawContours(img, contours, -1, color.contour, 2)
            for c in g_circles:
                cv2.circle(img, (c.x_px, c.y_px), 2, c.color.contour, -1)
                cv2.circle(img, (c.x_px, c.y_px), 3, (255, 255, 255), 1)
            # print('----------')
            #for c in g_circles: print(c, end="\t")
            #print()
            # store found circles
            global g_yellow_found_circle, g_red_found_circle
            global g_blue_found_circle, g_green_found_circle
            for c in circles_all:
                if c.color.color == Color.YELLOW:
                    if g_yellow_found_circle is None:
                        g_yellow_found_circle = c
                        print('Yellow found:', c)
                    elif g_yellow_found_circle.weight < c.weight:
                        g_yellow_found_circle = c
                        print('Yellow found again:', c)
                elif c.color.color == Color.RED:
                    if g_red_found_circle is None:
                        g_red_found_circle = c
                        print('Red found:', c)
                    elif g_red_found_circle.weight < c.weight:
                        g_red_found_circle = c
                        print('Red found again:', c)
                elif c.color.color == Color.BLUE:
                    if g_blue_found_circle is None:
                        g_blue_found_circle = c
                        print('Blue found:', c)
                    elif g_blue_found_circle.weight < c.weight:
                        g_blue_found_circle = c
                        print('Blue found again:', c)
                elif c.color.color == Color.GREEN:
                    if g_green_found_circle is None:
                        g_green_found_circle = c
                        print('Green found:', c)
                    elif g_green_found_circle.weight < c.weight:
                        g_green_found_circle = c
                        print('Green found again:', c)
        else:
            g_circles = None
    # publish image (to browser)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)


# @long_callback
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

print("Drone start searching")
g_drone_searching = True

set_effect(r=0, g=0, b=0)
for x in [24, 12, 14, 26, 24]:
    print(f'Go to {x}')
    navigate_wait(x=0, y=0, z=1, frame_id=f'aruco_{x}')
    # Wait for 5 seconds
    rospy.sleep(5)

print("Drone stop searching")
g_drone_searching = False


def find_car(c):
    print(f'Go to {c.aruco_id} ({c.color.color})')
    print('Drone was at ' +
        '{:.2f},{:.2f}'.format(c.aruco_pose.x, c.aruco_pose.y) +
        ', car was at ' +
        '{:.2f},{:.2f}'.format(c.x_metrs, c.y_metrs) +
        ', car size ' +
        '{:.2f},{:.2f}'.format(c.x_weight_metr, c.y_weight_metr))
    x_offset: float = c.aruco_pose.x - c.x_metrs
    y_offset: float = c.aruco_pose.y - c.y_metrs
    x_half_length: float = c.x_weight_metr / 2.0
    y_half_length: float = c.y_weight_metr / 2.0
    GRID_SIZE: float = 1.0
    print(f'Car found at {c.aruco_id}, offset {x_offset}, {y_offset}')
    # navigate_wait(x=y_offset, y=x_offset, z=1.0, frame_id=f'aruco_{c.aruco_id}')

    if ((abs(round(x_offset) - x_offset) + x_half_length) >= GRID_SIZE/2) or \
        ((abs(round(y_offset) - y_offset) + y_half_length) >= GRID_SIZE/2):
        set_effect(r=255, g=0, b=0)
        print('INCORRECT PARKING!!')
    else:
        set_effect(r=0, g=255, b=0)
        print('Correct parking')


print('Searching finished')
if g_blue_found_circle is None:
    print('blue car is not found')
elif not g_blue_found_circle.aruco_id:
    print('blue car is not found')
else:
    find_car(g_blue_found_circle)

print('Perform landing')
land()

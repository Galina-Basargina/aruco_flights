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


rospy.init_node('drone_vision')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image)


class Color(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3
    YELLOW = 4


class ColorCircle:
    def __init__(self, color: Color, lower, upper, contour):
        self.color: Color = color
        self.lower = lower
        self.upper = upper
        self.contour = contour


g_red_circle: ColorCircle = ColorCircle(  # bgr8
    Color.RED,
    np.array([29, 29, 51]),
    np.array([100, 100, 255]),  # np.array([88, 89, 255]),
    (220, 39, 214))
g_blue_circle: ColorCircle = ColorCircle(  # bgr8
    Color.BLUE,
    np.array([30, 28, 0]),
    np.array([204, 204, 52]),  # np.array([124, 124, 52]),
    (255, 225, 0))
# g_green_circle: ColorCircle = ColorCircle(  # bgr8
#    Color.GREEN,
#    np.array([0, 190, 0]),
#    np.array([190, 255, 190]),
#    (0, 67, 35))
g_yellow_circle: ColorCircle = ColorCircle(  # bgr8
    Color.YELLOW,
    np.array([0, 70, 70]),
    np.array([55, 255, 255]),
    (0, 137, 255))

g_colors: typing.List[ColorCircle] = [
    g_red_circle,
    g_blue_circle,
    #g_green_circle,
    g_yellow_circle,
]

g_image_size_known: bool = False
g_image_width: int = 0
g_image_height: int = 0
g_near_color: typing.Optional[Color] = None


def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    global g_image_size_known, g_image_width, g_image_height
    if not g_image_size_known:
        g_image_size_known = True
        g_image_width = data.width
        g_image_height = data.height
        print("Camera size: ", g_image_width, g_image_height)

    global g_near_color
    g_near_color = None
    min_distance = None

    global g_colors
    for color in g_colors:
        frame = cv2.inRange(img, color.lower, color.upper)
        contours, _hierarchy = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # calculations
        if not contours:
            continue
        # print(contours[0])

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
            center_x = (max_x - min_x) // 2 + min_x
            center_y = (max_y - min_y) // 2 + min_y
            distance = ((g_image_width // 2 - center_x)**2 + (g_image_height // 2 - center_y)**2) ** 0.5
            # print(color.color, distance, center_x, center_y)

            if distance < g_image_height // 2:
                if min_distance is None:
                    g_near_color = color.color
                    min_distance = distance
                elif min_distance > distance:
                    g_near_color = color.color
                    min_distance = distance

        cv2.drawContours(img, contours, -1, color.contour, 2)
    # print(g_near_color, min_distance)
    # publish image (to browser)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

if __name__ == '__main__':
    print('Take off and hover 1 m above the ground')
    navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    set_effect(r=0, g=0, b=0)
    rospy.sleep(3)

    for id, dx in [(20, 0), (16, 1), (12, 0), (14, 0), (22, 0)]:
        print(f'Go to {id}, dx={dx}')
        navigate_wait(x=dx, y=0, z=1, frame_id=f'aruco_{id}')

        # Wait for 5 seconds
        rospy.sleep(3)
        if g_near_color is None:
            set_effect(r=0, g=0, b=0)
        elif g_near_color == Color.RED:
            set_effect(r=255, g=0, b=0)
        elif g_near_color == Color.BLUE:
            set_effect(r=0, g=0, b=255)
        elif g_near_color == Color.YELLOW:
            set_effect(r=255, g=255, b=0)
        elif g_near_color == Color.GREEN:
            set_effect(r=0, g=255, b=0)
        # Wait for 5 seconds
        rospy.sleep(5)

    navigate_wait(x=0, y=0, z=1, frame_id=f'aruco_24')
    print('Perform landing')
    land()

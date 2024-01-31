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


rospy.init_node('drone_vision')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image)


class ColorCircle:
    def __init__(self, lower, upper, contour):
        self.lower = lower
        self.upper = upper
        self.contour = contour


g_red_circle: ColorCircle = ColorCircle(  # bgr8
    np.array([29, 29, 51]),
    np.array([100, 100, 255]),  # np.array([88, 89, 255]),
    (220, 39, 214))
g_blue_circle: ColorCircle = ColorCircle(  # bgr8
    np.array([30, 28, 0]),
    np.array([204, 204, 52]),  # np.array([124, 124, 52]),
    (255, 225, 0))
# g_green_circle: ColorCircle = ColorCircle(  # bgr8
#    np.array([0, 190, 0]),
#    np.array([190, 255, 190]),
#    (0, 67, 35))
g_yellow_circle: ColorCircle = ColorCircle(  # bgr8
    np.array([0, 70, 70]),
    np.array([55, 255, 255]),
    (0, 137, 255))

g_colors: typing.List[ColorCircle] = [
    g_red_circle,
    g_blue_circle,
    #g_green_circle,
    g_yellow_circle,
]


def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    global g_colors
    for color in g_colors:
        frame = cv2.inRange(img, color.lower, color.upper)
        contours, _hierarchy = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # calculations
        if not contours:
            continue
        cv2.drawContours(img, contours, -1, color.contour, 2)
    # publish image (to browser)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

if __name__ == '__main__':
    while True:
        rospy.sleep(1)

import rospy
import cv2
import math
import typing
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
import numpy as np
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray
from clover.srv import SetLEDEffect
from pyzbar import pyzbar

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)


def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)


g_max_yaw_gradus = 5
g_max_yaw_radian = math.radians(g_max_yaw_gradus)


def yaw_wait_to_normal():
    global g_max_yaw_radian
    while True:
        yaw = get_telemetry().yaw
        if abs(yaw) < g_max_yaw_radian:
            set_yaw(yaw=-yaw, frame_id='body')
            break
        else:
            if yaw < 0.0:
                set_yaw(yaw=g_max_yaw_radian, frame_id='body')
            else:
                set_yaw(yaw=-g_max_yaw_radian, frame_id='body')


def yaw_wait(yaw: float):
    global g_max_yaw_gradus, g_max_yaw_radian
    for x in range(int(yaw) // g_max_yaw_gradus):
        set_yaw(yaw=g_max_yaw_radian, frame_id='body')
        rospy.sleep(0.5)


rospy.init_node('drone_vision')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image)
font = cv2.FONT_HERSHEY_SIMPLEX


def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    barcodes = pyzbar.decode(img)
    if barcodes:
        # print(barcodes[0].polygon)
        # print(barcodes[0].data.decode('utf-8'))
        cv2.putText(img, barcodes[0].data.decode('utf-8'), (1, 20), font, 1, (0, 0, 255), 2)
        for i in range(4):
            cv2.line(img,
                     (barcodes[0].polygon[i].x, barcodes[0].polygon[i].y),
                     (barcodes[0].polygon[(i+1) % 4].x, barcodes[0].polygon[(i+1) % 4].y),
                     (0, 0, 255), 2)
    # publish image (to browser)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

if __name__ == '__main__':
    navigate_wait(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    set_effect(r=0, b=0, g=0)
    rospy.sleep(5)

    navigate_wait(x=1, y=0, z=1, frame_id='aruco_16')
    rospy.sleep(3)
    yaw_wait_to_normal()

    yaw_wait(360)

    navigate_wait(x=0, y=0, z=1, frame_id='aruco_24')
    rospy.sleep(5)
    land()

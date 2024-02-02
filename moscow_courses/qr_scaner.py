from re import A
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
# v0.24: set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)


def navigate_wait(x=0.0, y=0.0, z=0.0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)


g_max_yaw_gradus: int = 5
g_max_yaw_radian: float = math.radians(g_max_yaw_gradus)


def yaw_wait_to_normal(id: int, dx: float, altitude: float):
    global g_max_yaw_radian
    print(f'Rotate to normal over aruco#{id} dx={dx} altitude={altitude}')
    while True:
        yaw = get_telemetry().yaw
        if abs(yaw) < g_max_yaw_radian:
            navigate(yaw=-yaw, frame_id=f'aruco_{id}', x=dx, y=0, z=altitude)
            rospy.sleep(0.5)
            break
        else:
            if yaw < 0.0:
                navigate(yaw=g_max_yaw_radian, frame_id=f'aruco_{id}', x=dx, y=0, z=altitude)
                rospy.sleep(0.5)
            else:
                navigate(yaw=-g_max_yaw_radian, frame_id=f'aruco_{id}', x=dx, y=0, z=altitude)
                rospy.sleep(0.5)


def yaw_wait(yaw: float, id: int, dx: float, altitude: float):
    print(f'Rotate {yaw} gradus over aruco#{id} dx={dx} altitude={altitude}')

    yaw_from: float = get_telemetry().yaw
    yaw_to: float = yaw_from + math.radians(yaw)
    rotate_to_plus: bool = yaw_to > yaw_from

    global g_max_yaw_gradus, g_max_yaw_radian
    while True:
        if rotate_to_plus:
            yaw_rest = yaw_to - yaw_from
            if yaw_rest < g_max_yaw_radian:
                navigate_wait(yaw=yaw_rest+yaw_from, frame_id='aruco_map', x=1, y=2, z=altitude, tolerance=0.1)
                break
            yaw_from += g_max_yaw_radian
            navigate_wait(yaw=yaw_from, frame_id='aruco_map', x=1, y=2, z=altitude, tolerance=0.1)
        else:
            yaw_rest = yaw_from - yaw_to
            if yaw_rest < g_max_yaw_radian:
                navigate_wait(yaw=-yaw_rest+yaw_from, frame_id='aruco_map', x=1, y=2, z=altitude, tolerance=0.1)
                break
            yaw_from -= g_max_yaw_radian
            navigate_wait(yaw=yaw_from, frame_id='aruco_map', x=1, y=2, z=altitude, tolerance=0.1)
        print(yaw_from)
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
    ALTITUDE: float = 1.2
    DX_AT_16: float = 1.0

    navigate_wait(x=0, y=0, z=ALTITUDE, frame_id='body', auto_arm=True)
    set_effect(r=0, b=0, g=0)
    rospy.sleep(3)

    navigate_wait(x=DX_AT_16, y=0, z=ALTITUDE, frame_id='aruco_16')
    # v0.24: set_yaw(yaw=math.radians(30), frame_id='body')
    rospy.sleep(1)
    yaw_wait_to_normal(16, DX_AT_16, ALTITUDE)

    yaw_wait(360, 16, DX_AT_16, ALTITUDE)

    navigate_wait(x=0, y=0, z=ALTITUDE, frame_id='aruco_24')
    rospy.sleep(3)
    land()

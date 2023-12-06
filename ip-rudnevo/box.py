# Information: https://clover.coex.tech/aruco

import rospy
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
import math

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service


def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)

print('Up')
navigate_wait(z=1, frame_id='body', auto_arm=True)
navigate_wait(z=1, frame_id='aruco_3')

print('43')
navigate_wait(y=-1, z=1, frame_id='aruco_36')
for i in range(6):
    set_effect(effect='fade', r=0, g=0, b=255)
    set_effect(effect='flash', r=0, g=0, b=0)
    rospy.sleep(0.5)
set_effect(effect='fade', r=0, g=0, b=0)
navigate_wait(z=1, frame_id='aruco_3')
land()
rospy.sleep(10)

print('46')
navigate_wait(z=1, frame_id='body', auto_arm=True)
navigate_wait(y=-1, z=1, frame_id='aruco_39')
for i in range(6):
    set_effect(effect='fade', r=0, g=0, b=255)
    set_effect(effect='flash', r=0, g=0, b=0)
    rospy.sleep(0.5)
set_effect(effect='fade', r=0, g=0, b=0)
navigate_wait(z=1, frame_id='aruco_3')
land()
rospy.sleep(10)

print('49')
navigate_wait(z=1, frame_id='body', auto_arm=True)
navigate_wait(y=-1, z=1, frame_id='aruco_42')
for i in range(6):
    set_effect(effect='fade', r=0, g=0, b=255)
    set_effect(effect='flash', r=0, g=0, b=0)
    rospy.sleep(0.5)
set_effect(effect='fade', r=0, g=0, b=0)
navigate_wait(z=1, frame_id='aruco_3')
land()
rospy.sleep(10)

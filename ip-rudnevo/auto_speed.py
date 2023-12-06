# Information: https://clover.coex.tech/aruco

import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from clover.srv import SetLEDEffect


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


def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=1, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)

navigate_wait(z=1, frame_id='body', auto_arm=True)
navigate_wait(z=1, frame_id='aruco_9')
print(9)

navigate_wait(z=1, frame_id='aruco_0')
print(0)
set_effect(effect='fade', r=0, g=0, b=255)  # blue
navigate_wait(z=1, frame_id='aruco_21')
print(21)
navigate_wait(z=1, frame_id='aruco_30')
set_effect(effect='fade', r=255, g=0, b=0)  # red
print(30)
navigate_wait(z=1, frame_id='aruco_50')
print(50)

navigate_wait(z=1, frame_id='aruco_41')
print(41)
set_effect(effect='fade', r=255, g=255, b=255)  #white
navigate_wait(z=1, frame_id='aruco_61')
print(61)
navigate_wait(z=1, frame_id='aruco_70')
print(70)
set_effect(effect='fade', r=0, g=0, b=255)  #blue
navigate_wait(z=1, frame_id='aruco_90')
print(90)
navigate_wait(z=1, frame_id='aruco_81')
print(81)
set_effect(effect='fade', r=255, g=0, b=0)  #red
navigate_wait(z=1, frame_id='aruco_71')
print(71)

navigate_wait(z=1, frame_id='aruco_9')
print(9)
land()

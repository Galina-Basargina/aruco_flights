# Information: https://clover.coex.tech/en/snippets.html#navigate_wait

import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
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

# speed=0.5
def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)

print('Take off 0.7 meter')
set_effect(r=0, g=0, b=0)
navigate_wait(z=0.7, frame_id='body', auto_arm=True)
navigate_wait(x=0, y=0, z=0.7, frame_id='aruco_map')

RADIUS = 1.5/2.0

print('Fly pseudo center')
navigate_wait(x=3.0*0.9, y=2*1.34, z=0.7, frame_id='aruco_map')
start = get_telemetry()

navigate_wait(x=3.0*0.9, y=2*1.34+RADIUS, z=0.7, frame_id='aruco_map')
SPEED = 0.1*4.0
start_stamp = rospy.get_rostime()
#r = rospy.Rate(10)

#while not rospy.is_shutdown():
for i in range(28*5-1):
    angle = (rospy.get_rostime() - start_stamp).to_sec() * SPEED
    #angle = 2.0*3.14 / 36.0 * i
    x = start.x + math.sin(angle) * RADIUS
    y = start.y + math.cos(angle) * RADIUS
    set_position(x=x, y=y, z=start.z)

    #r.sleep(1000)
    print(i, angle, rospy.get_rostime())
    rospy.sleep(0.5/4.0)


print('Land')
set_effect(r=0, g=0, b=0)
navigate_wait(x=0, y=0, z=0.7, frame_id='aruco_map')
land()

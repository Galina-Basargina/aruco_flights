
import math
import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('home')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0.0, y=0.0, z=0.0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.15, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        return res
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)


def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)


if __name__ == '__main__':
    ALTITUDE = 0.7
    YAW = math.radians(0.0)

    print('up')
    navigate_wait(z=ALTITUDE, auto_arm=True)

    print(140)
    navigate_wait(z=ALTITUDE, frame_id='aruco_140', yaw=YAW)
    rospy.sleep(1)
    navigate_wait(z=ALTITUDE, frame_id='aruco_140', yaw=YAW)

    rospy.sleep(2)

    print(141)
    navigate_wait(z=ALTITUDE, frame_id='aruco_141', yaw=YAW)
    rospy.sleep(1)
    navigate_wait(z=ALTITUDE, frame_id='aruco_141', yaw=YAW)
    
    rospy.sleep(8)
    
    print('land')
    land_wait()
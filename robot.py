# Information: https://clover.coex.tech/en/snippets.html#navigate_wait

import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray

rospy.init_node('flight1')


get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

Marker = None
Radius = 0.15


def markers_callback(msg):
    global Marker
    Marker = [(x.pose.position.x, x.pose.position.y, x.pose.position.z) for x in msg.markers if x.id == 14]
    print('ID:', [x.id for x in msg.markers], 'POSE:', Marker)
    # for marker in msg.markers:
    #    print('Marker: %s' % marker)


def navigate_wait(x=0.0, y=0.0, z=0.0, yaw=float('nan'), yaw_rate=0, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)  # yaw_rate=yaw_rate,

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)
        # rospy.spin()


rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)


# navigate_wait(z=3, auto_arm=True)
navigate_wait(frame_id='body', x=0, y=0, z=1.5, auto_arm=True)
flag = 0
while True:
    rospy.sleep(0.1)
    if Marker is None:
        continue
    m = Marker.copy()
    Marker = None
    if len(m) == 0:
        if flag == 0:
            continue
        else:
            break
    
    dis = math.sqrt(m[0][1] ** 2 + m[0][0] ** 2)
    if m[0][2] < 0.30:
        break
    print('!!!!!!!!\n!!!!!!!!\n!!!!!!!!\n', m, dis, '!!!!!!!!\n!!!!!!!!\n!!!!!!!!\n')
    navigate_wait(x=-m[0][1], y=-m[0][0], z=-0.3, tolerance=Radius)
    flag = 1
        
# navigate_wait(frame_id='aruco_14', z=1)
land()

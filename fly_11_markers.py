# Information: https://clover.coex.tech/en/snippets.html#navigate_wait

import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

markerN = None
markerX = None
markersFound = None

def markers_callback(msg):
    global markerX, markerN, markersFound
    if msg.markers:
        l = [m.id for m in msg.markers]
        if not l == markersFound:
            markersFound = l
            print('Detected markers:', markersFound)
        mX = next((marker for marker in msg.markers if marker.id == markerN), None)
        if mX:
            #print(mX)
            #print('pose', mX.pose.position.x, mX.pose.position.y, mX.pose.position.z)
            #print('orie', mX.pose.orientation.x, mX.pose.orientation.y, mX.pose.orientation.z, mX.pose.orientation.w)
            markerX = (mX.pose.position.x, mX.pose.position.y, mX.pose.position.z)
        #for marker in msg.markers:
        #    print('Marker: %s' % marker)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), yaw_rate=0, speed=0.5, \
        frame_id='body', tolerance=0.2, auto_arm=False):

    res = navigate(x=x, y=y, z=z, yaw=yaw, yaw_rate=yaw_rate, speed=speed, \
        frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)
        #rospy.spin()

if __name__ == '__main__':
    rospy.init_node('flight')
    rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)
    #navigate_wait(y=1, frame_id='body')

    auto_arm = True
    markerN = 0
    markerX = None
    for n in range(12):
        # поиск маркера n
        while not markerX:
            print('Взлёт на 0.5 метрa, поиск маркера #', markerN)
            navigate_wait(z=0.5, frame_id='body', auto_arm=auto_arm)
            auto_arm = False
        # полёт к маркеру на высоте 1 метр
        if markerX:
            yxz = (markerX[0], markerX[1], markerX[2])
            markerN = n + 1
            markerX = None
            # во время полета к текущему маркеру ищем следующий маркер
            print('Полёт к маркеру #', n, ' x=', yxz[0], 'y=', yxz[1], 'z=', yxz[2])
            navigate_wait(x=-yxz[1], y=-yxz[0], z=1.0-yxz[2], frame_id='body')

    print('Посадка')
    land()

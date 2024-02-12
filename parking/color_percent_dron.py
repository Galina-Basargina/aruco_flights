# Information: https://clover.coex.tech/en/snippets.html#navigate_wait

import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
from aruco_pose.msg import MarkerArray
import math
from enum import Enum
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('parking')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image)
image_pub_crop = rospy.Publisher('~debug_crop', Image)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0.0, y=0.0, z=0.0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        return res
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)


class State(Enum):
    NONE = 0
    UP = 1
    HOME = 2
    CROP = 3
    ROUTE = 4
    MOVE = 5
    CENTER = 6
    BASE = 7
    COLORS = 8
    MINMAX = 9


g_xmin = None
g_xmax = None
g_ymin = None
g_ymax = None
g_marker = None
g_state: State = State.NONE


@long_callback
def markers_callback(msg):
    global g_marker, g_state
    if g_state == State.CROP:
        print('Detected markers:', [m.id for m in msg.markers])
        g_marker = next((m for m in msg.markers if m.id == 24), None)

rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)


@long_callback
def image_callback(data):
    global g_xmin, g_xmax, g_ymin, g_ymax, g_marker, g_state
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    if g_ymax:
        crop = img[g_ymin:g_ymax+1, g_xmin:g_xmax+1]
        image_pub_crop.publish(bridge.cv2_to_imgmsg(crop, 'bgr8'))
        
        cv2.rectangle(img, (g_xmin, g_ymin), (g_xmax, g_ymax), (0,0,255), 2)
    cv2.putText(img, str(g_state), (2, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)


def change_state(state):
    global g_state
    g_state = state
    print(g_state)


if __name__ == '__main__':
    ALTITUDE = 1.0
    TIME = 5
    
    change_state(State.UP)
    navigate_wait(z=ALTITUDE, auto_arm=True)

    change_state(State.HOME)
    navigate_wait(z=ALTITUDE, yaw=math.radians(0), frame_id='aruco_24')
    
    change_state(State.CENTER)
    rospy.sleep(TIME)

    change_state(State.CROP)
    while True:
        if g_marker == None:
            rospy.sleep(0.1)
            continue
        change_state(State.MINMAX)
        g_xmin = int(min(g_marker.c1.x, g_marker.c2.x, g_marker.c3.x, g_marker.c4.x))-5
        g_xmax = int(max(g_marker.c1.x, g_marker.c2.x, g_marker.c3.x, g_marker.c4.x))+5
        g_ymin = int(min(g_marker.c1.y, g_marker.c2.y, g_marker.c3.y, g_marker.c4.y))-5
        g_ymax = int(max(g_marker.c1.y, g_marker.c2.y, g_marker.c3.y, g_marker.c4.y))+5
        print(g_xmin, g_xmax, g_ymin, g_ymax)
        break
    
    change_state(State.ROUTE)
    for x,y in [(0,1),  # 20
                (0,2),  # 16
                (0,3),  # 12
                (0,4),  # 8
                (1,4),  # 9
                (1,3),  # 13
                (1,2),  # 17
                (1,1),  # 21
                (1,0),  # 25
                ]:
        print(f'go to {x} {y}')
    
        change_state(State.MOVE)
        navigate_wait(x=x, y=y, z=ALTITUDE, frame_id='aruco_map')
        rospy.sleep(1)
        navigate_wait(x=x, y=y, z=ALTITUDE, frame_id='aruco_map')

        change_state(State.CENTER)
        rospy.sleep(TIME)

        change_state(State.COLORS)
        rospy.sleep(1)
        
    change_state(State.BASE)
    navigate_wait(z=ALTITUDE, frame_id='aruco_24')
    land()
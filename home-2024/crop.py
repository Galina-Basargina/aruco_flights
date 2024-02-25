
import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import enum
from aruco_pose.msg import MarkerArray

rospy.init_node('home')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image)
image_crop = rospy.Publisher('~crop', Image)

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


class State(enum.Enum):
    NONE = 0
    UP = 1
    CENTER = 2
    ROUTE = 3
    LAND = 4
    FINISH = 5
    MINMAX = 6
    FINDM = 7

class Global:
    def __init__(self) -> None:
        self.state: State = State.NONE
        self.minx = None
        self.miny = None
        self.maxx = None
        self.maxy = None
        self.marker = None

g_vars: Global = Global()


def markers_callback(msg):
    global g_vars 
    if g_vars.state == State.FINDM:
        print('Detected markers:', [m.id for m in msg.markers])
        g_vars.marker = next((m for m in msg.markers if m.id == 140), None)
        
rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)


def image_callback(data):
    global g_vars
    img = bridge.imgmsg_to_cv2(data, 'bgr8') 
    if g_vars.maxy is not None:
        crop = img[g_vars.miny:g_vars.maxy+1, g_vars.minx:g_vars.maxx+1]
        image_crop.publish(bridge.cv2_to_imgmsg(crop, 'bgr8'))
    cv2.rectangle(img, (g_vars.minx, g_vars.miny), (g_vars.maxx, g_vars.maxy), (0,0,255), 2)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

rospy.Subscriber('main_camera/image_raw', Image, image_callback)


def change_state(state: State):
    print(f'{g_vars.state} -> {state}')
    g_vars.state = state


if __name__ == '__main__':
    ALTITUDE = 0.7
    YAW = math.radians(0.0)

    change_state(State.UP)
    print('up')
    navigate_wait(z=ALTITUDE, auto_arm=True)

    change_state(State.CENTER)
    print(140)
    navigate_wait(z=ALTITUDE, frame_id='aruco_140', yaw=YAW)
    rospy.sleep(1)
    navigate_wait(z=ALTITUDE, frame_id='aruco_140', yaw=YAW)

    change_state(State.FINDM)
    while True:
        if g_vars.marker is None:
            rospy.sleep(0.1)
            continue
        change_state(State.MINMAX)
        m = g_vars.marker
        g_vars.minx = int(min(m.c1.x, m.c2.x, m.c3.x, m.c4.x))
        g_vars.miny = int(min(m.c1.y, m.c2.y, m.c3.y, m.c4.y))
        g_vars.maxx = int(max(m.c1.x, m.c2.x, m.c3.x, m.c4.x))
        g_vars.maxy = int(max(m.c1.y, m.c2.y, m.c3.y, m.c4.y))
        print(g_vars.minx, g_vars.miny, g_vars.maxx, g_vars.maxy)
        break

    change_state(State.ROUTE)
    print(141)
    navigate_wait(z=ALTITUDE, frame_id='aruco_141', yaw=YAW)
    rospy.sleep(1)
    navigate_wait(z=ALTITUDE, frame_id='aruco_141', yaw=YAW)
    
    rospy.sleep(8)
    
    change_state(State.LAND)
    print('land')
    land_wait()

    change_state(State.FINISH)
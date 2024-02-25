
import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import enum
from aruco_pose.msg import MarkerArray
import numpy as np
from clover.srv import SetLEDEffect


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
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service


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
    HOME = 8
    COLOR = 9
    COLOR_STOP = 10


class Global:
    def __init__(self) -> None:
        self.state: State = State.NONE
        self.minx = None
        self.miny = None
        self.maxx = None
        self.maxy = None
        self.marker = None
        self.max_color = None


g_vars: Global = Global()
g_is_simulation = rospy.get_param('/use_sim_time', False)

if g_is_simulation:
    g_red = (  # bgr8
        np.array([29, 29, 135]),
        np.array([100, 100, 255]),
        (0, 0, 255))
    g_blue = (  # bgr8
        np.array([50, 50, 0]),
        np.array([255, 134, 10]),
        (255, 0, 0))
    g_green = (  # bgr8
        np.array([0, 190, 0]),
        np.array([190, 255, 190]),
        (0, 255, 0))
    g_yellow = (  # bgr8
        np.array([0, 70, 70]),
        np.array([55, 255, 255]),
        (0, 255, 255))
else:
    g_red = (  # bgr8
        np.array([0,0,85]),
        np.array([79, 79, 255]),
        (0, 0, 255))
    g_blue = (  # bgr8
        np.array([95, 0, 0]),
        np.array([255, 105, 69]),
        (255, 0, 0))
    g_green = (  # bgr8
       np.array([0, 36, 0]),
       np.array([59, 71, 44]),
       (0, 255, 0))
    g_yellow = (  # bgr8
        np.array([74, 117, 124]),
        np.array([127, 202, 190]),
        (0, 255, 255))

ALTITUDE = 0.7
YAW = math.radians(0.0)
if g_is_simulation:
    MIN_PERCENT = 15.0
else:
    MIN_PERCENT = 18.0


def count_percent(img, low, up) -> float:
    height = len(img)
    width = len(img[0])
    square = width * height
    count = 0.0

    mask = cv2.inRange(img, low, up)
    frame = cv2.bitwise_and(img, img, mask=mask)

    for y in range(height):
        for x in range(width):
            bgr = frame[y][x]
            if not max(bgr) == 0:
                count += 1.0
    return round(count / square * 100.0, 1)


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
        if g_vars.state == State.COLOR:
            r_p = count_percent(crop, g_red[0], g_red[1])
            b_p = count_percent(crop, g_blue[0], g_blue[1])
            g_p = count_percent(crop, g_green[0], g_green[1])
            y_p = count_percent(crop, g_yellow[0], g_yellow[1])

            g_vars.max_color = max(
                (r_p, 'red', g_red[2]),
                (b_p, 'blue', g_blue[2]),
                (g_p, 'green', g_green[2]),
                (y_p, 'yellow', g_yellow[2]),
                (MIN_PERCENT, 'empty', (0, 0, 0))
            )
            print(f'r {r_p} b {b_p} g {g_p} y {y_p} - {g_vars.max_color}')
        image_crop.publish(bridge.cv2_to_imgmsg(crop, 'bgr8'))

        cv2.rectangle(img, (g_vars.minx, g_vars.miny), (g_vars.maxx, g_vars.maxy), (0,0,255), 2)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

rospy.Subscriber('main_camera/image_raw', Image, image_callback)


def change_state(state: State):
    print(f'{g_vars.state} -> {state}')
    g_vars.state = state


if __name__ == '__main__':
    change_state(State.UP)
    navigate_wait(z=ALTITUDE, auto_arm=True)
    print('up done')

    change_state(State.CENTER)
    navigate_wait(z=ALTITUDE, frame_id='aruco_140', yaw=YAW)
    rospy.sleep(1)
    navigate_wait(z=ALTITUDE, frame_id='aruco_140', yaw=YAW)
    print('center done')

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

    for x,y in [(0.51, 0.45),
                (0.93, 0.75),
                (1.23, 0.45),
                (0.93, 0.15)
               ]:
        change_state(State.ROUTE)
        print(f'go to x{x} y{y}')
        navigate_wait(x=x, y=y, z=ALTITUDE, frame_id='aruco_map', yaw=YAW)
        rospy.sleep(1)
        navigate_wait(x=x, y=y, z=ALTITUDE, frame_id='aruco_map', yaw=YAW)
        print(f'x{x} y{y} done')

        g_vars.max_color = None
        change_state(State.COLOR)
        rospy.sleep(2)
        change_state(State.COLOR_STOP)
        rospy.sleep(0.5)
        print(f'color max {g_vars.max_color}')

        if g_vars.max_color is not None:
            set_effect(b=g_vars.max_color[2][0], g=g_vars.max_color[2][1], r=g_vars.max_color[2][2])
            rospy.sleep(1)
        set_effect(b=0, g=0, r=0)

    change_state(State.HOME)
    if g_is_simulation:
        navigate_wait(x=0.0, y=0.0, z=ALTITUDE, frame_id='aruco_map', yaw=YAW)
        rospy.sleep(1)
        navigate_wait(x=0.0, y=0.0, z=ALTITUDE, frame_id='aruco_map', yaw=YAW)
    else:
        navigate_wait(z=ALTITUDE, frame_id='aruco_140', yaw=YAW)
        rospy.sleep(1)
        navigate_wait(z=ALTITUDE, frame_id='aruco_140', yaw=YAW)
    print('home done')

    change_state(State.LAND)
    land_wait()
    print('land done')

    change_state(State.FINISH)
    print('finish done')

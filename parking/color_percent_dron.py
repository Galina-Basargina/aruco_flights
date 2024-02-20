import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
from aruco_pose.msg import MarkerArray
from enum import Enum
import numpy as np

rospy.init_node('parking')
bridge = CvBridge()

image_pub = rospy.Publisher('~debug', Image)
image_pub_crop = rospy.Publisher('~debug_crop', Image)
image_pub_percent = rospy.Publisher('~debug_percent', Image)

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


class State(Enum):
    HOME = 1
    FINDM = 2
    UP = 3
    MINMAX = 4
    MOVE = 5
    STOP = 6
    BASE = 7
    LAND = 8
    COLORS = 9
    FINISHED = 10
    ROUTE = 11

g_xmin = None
g_xmax = None
g_ymin = None
g_ymax = None
g_state: State = State.HOME
g_marker = None
g_tests: int = None
g_parking = None
g_car = None
g_is_simulation = rospy.get_param('/use_sim_time', False)

g_lower_yellow = np.array([0, 70, 70])
g_upper_yellow = np.array([55, 255, 255])

g_lower_red = np.array([20, 20, 135])  # 29,29,135
g_upper_red = np.array([int(131*1.2), int(131*1.2), 255])

g_lower_blue = np.array([50, 0, 0])  # 50, 50, 0
g_upper_blue = np.array([255, int(134*1.4), int(10*1.4)])

g_lower_green = np.array([int(125*1.2), int(104*1.2), int(50*1.2)])
g_upper_green = np.array([int(164*1.3), 255, int(77*1.3)])

g_lower_brown = np.array([120, 150, 140])
g_upper_brown = np.array([255, 197, 180])

def find_percent(img, low, up, pub=None) -> float:
    height: int = len(img)
    width: int = len(img[0])
    square: int = width * height
    count_px: int = 0

    mask = cv2.inRange(img, low, up)
    frame = cv2.bitwise_and(img, img, mask=mask)
    if pub is not None:
        pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))

    for y in range(height):
        for x in range(width):
            bgr = frame[y][x]
            if max(bgr) != 0:
                count_px += 1

    percent = count_px / square
    return round(percent * 100.0, 1)


@long_callback
def markers_callback(msg):
    global g_marker, g_state
    if g_state == State.FINDM:
        print('Detected markers:', [m.id for m in msg.markers])
        g_marker = next((m for m in msg.markers if m.id == 24), None)

rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)


@long_callback
def image_callback(data):
    global g_xmax, g_xmin, g_ymin, g_ymax, g_state
    global g_tests, g_parking, g_car
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    if g_ymax:
        crop = img[g_ymin:g_ymax+1, g_xmin:g_xmax+1]
        image_pub_crop.publish(bridge.cv2_to_imgmsg(crop, 'bgr8'))
        if g_state == State.COLORS:
            blue_per = find_percent(crop, g_lower_blue, g_upper_blue)
            yellow_per = find_percent(crop, g_lower_yellow, g_upper_yellow)
            red_per = find_percent(crop, g_lower_red, g_upper_red, image_pub_percent)
            brown_per = find_percent(crop, g_lower_brown, g_upper_brown)
            green_per = find_percent(crop, g_lower_green, g_upper_green)

            parking_color = max(
                [(blue_per, 'blue', (255,0,0)),
                 (yellow_per, 'yellow', (0,255,255)),
                 (15.0, 'aruco', (0,0,0))]
            )
            car_color = max([
                (red_per, 'red', (0,0,255)),
                (brown_per, 'brown', (255,255,255)),
                (green_per, 'green', (0,255,0)),
                (15.0, 'empty', (0,0,0))
            ])

            print('blue', blue_per,
                  'yellow:', yellow_per,
                  'red', red_per,
                  'brown', brown_per,
                  'green', green_per,
                  '-',
                  parking_color[1], int(parking_color[0]),
                  car_color[1], int(car_color[0]))
            cv2.rectangle(img, (290, 10), (300, 20), parking_color[2], -1)
            cv2.rectangle(img, (270, 10), (280, 20), car_color[2], -1)

            g_tests += 1
            g_parking = (g_parking[0]+blue_per, g_parking[1]+yellow_per)
            g_car = (g_car[0]+red_per, g_car[1]+brown_per, g_car[2]+green_per)

        cv2.rectangle(img, (g_xmin, g_ymin), (g_xmax, g_ymax), (0,0,255), 2)
    cv2.putText(img, str(g_state), (3, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)


def change_state(state):
    global g_state
    g_state = state
    print(g_state)


if __name__ == "__main__":
    ALTITUDE = 1.1
    TIME = 3

    change_state(State.UP)
    navigate_wait(z=ALTITUDE, auto_arm=True)

    change_state(State.HOME)
    navigate_wait(z=ALTITUDE, frame_id='aruco_24')
    rospy.sleep(1)
    navigate_wait(z=ALTITUDE, yaw=math.radians(0), frame_id='aruco_map')

    change_state(State.FINDM)
    # rospy.sleep(TIME)
    while True:
        if g_marker is None:
            rospy.sleep(0.1)
            continue
        change_state(State.MINMAX)
        m = g_marker
        g_xmin = int(min(m.c1.x, m.c2.x, m.c3.x, m.c4.x))
        g_xmax = int(max(m.c1.x, m.c2.x, m.c3.x, m.c4.x))
        g_ymin = int(min(m.c1.y, m.c2.y, m.c3.y, m.c4.y))
        g_ymax = int(max(m.c1.y, m.c2.y, m.c3.y, m.c4.y))
        print(g_xmin, g_xmax, g_ymin, g_ymax)
        break
    
    change_state(State.ROUTE)
    for x,y in [(0,1),  # 20
                (0,2),  # 16
                (0,3),  # 12
                (0,4),  # 8
                (0,5),  # 4
                (0,6),  # 0!
                # (1,4),  # 9
                # (1,3),  # 13
                # (1,2),  # 17
                # (1,1),  # 21
                # (1,0),  # 25
                ]:
        print(f'go to {x} {y}')
    
        change_state(State.MOVE)
        navigate_wait(x=x, y=y, z=ALTITUDE, frame_id='aruco_map', yaw=math.radians(0))
        rospy.sleep(1)
        navigate_wait(x=x, y=y, z=ALTITUDE, frame_id='aruco_map', yaw=math.radians(0))

        change_state(State.STOP)
        rospy.sleep(TIME)

        g_tests = 0
        g_parking = (0.0, 0.0)
        g_car = (0.0, 0.0, 0.0)
        change_state(State.COLORS)
        rospy.sleep(1)
        change_state(State.FINISHED)
        rospy.sleep(0.5)
        print('RESULT:', g_tests, g_parking, g_car)

    change_state(State.BASE)
    navigate_wait(z=ALTITUDE, frame_id='aruco_24')

    change_state(State.LAND)
    land()

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
from clover.srv import SetLEDEffect
import typing


rospy.init_node('parking')
bridge = CvBridge()

image_pub = rospy.Publisher('~debug', Image)
image_pub_crop = rospy.Publisher('~debug_crop', Image)
image_pub_percent = rospy.Publisher('~debug_percent', Image)
image_pub_parking = rospy.Publisher('~debug_parking', Image)
image_pub_parking_color = rospy.Publisher('~debug_color', Image)

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
    STOP_COLORS = 12
    ERROR_LINE_CHECK = 13
    STOP_ERROR_LINE_CHECK = 14


PARKING_LINE_OFFSET = 5

g_xmin = None
g_xmax = None
g_ymin = None
g_ymax = None
g_xmin_line = None
g_xmax_line = None
g_ymin_line = None
g_ymax_line = None
g_state: State = State.HOME
g_marker = None
g_is_simulation = rospy.get_param('/use_sim_time', False)
g_side = ''
g_m_blue = 0.0
g_m_yellow = 0.0
g_m_red = 0.0
g_m_brown = 0.0
g_m_green = 0.0
g_red_line = 0
g_brown_line = 0
g_green_line = 0

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


def error_line_check(img, sides: str, low, up) -> float:
    height: int = len(img)
    width: int = len(img[0])
    count_px: int = 0

    mask = cv2.inRange(img, low, up)
    frame = cv2.bitwise_and(img, img, mask=mask)
    if 'T' in sides:
        for x in range(width):
            bgr = frame[0][x]
            if not max(bgr) == 0:
                count_px += 1
    if 'B' in sides:
        for x in range(width):
            bgr = frame[height-1][x]
            if not max(bgr) == 0:
                count_px += 1
    if 'L' in sides:
        for y in range(height):
            bgr = frame[y][0]
            if not max(bgr) == 0:
                count_px += 1
    if 'R' in sides:
        for y in range(height):
            bgr = frame[y][width-1]
            if not max(bgr) == 0:
                count_px += 1
    return count_px


@long_callback
def markers_callback(msg):
    global g_marker, g_state
    if g_state == State.FINDM:
        print('Detected markers:', [m.id for m in msg.markers])
        g_marker = next((m for m in msg.markers if m.id == 24), None)

rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)


@long_callback
def image_callback(data):
    global g_xmax, g_xmin, g_ymin, g_ymax, g_state, g_side
    global g_xmin_line, g_xmin_line, g_ymin_line, g_ymax_line
    global g_m_blue, g_m_yellow, g_m_red, g_m_brown, g_m_green
    global g_red_line, g_brown_line, g_green_line
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    if g_ymax:
        crop = img[g_ymin:g_ymax+1, g_xmin:g_xmax+1]
        image_pub_crop.publish(bridge.cv2_to_imgmsg(crop, 'bgr8'))
        parking_img =img[g_ymin-PARKING_LINE_OFFSET:g_ymax+1+PARKING_LINE_OFFSET,
                         g_xmin-PARKING_LINE_OFFSET:g_xmax+1+PARKING_LINE_OFFSET]
        image_pub_parking.publish(bridge.cv2_to_imgmsg(crop, 'bgr8'))
        if g_state == State.COLORS:
            # moment values
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
            print('Colors:',
                  'blue', blue_per,
                  'yellow', yellow_per,
                  'red', red_per,
                  'brown', brown_per,
                  'green', green_per,
                  '-',
                  parking_color[1], int(parking_color[0]),
                  car_color[1], int(car_color[0]))
            cv2.rectangle(img, (290, 10), (300, 20), parking_color[2], -1)
            cv2.rectangle(img, (270, 10), (280, 20), car_color[2], -1)

            # max values
            g_m_blue = max(g_m_blue, blue_per)
            g_m_yellow = max(g_m_yellow, yellow_per)
            g_m_red = max(g_m_red, red_per)
            g_m_brown = max(g_m_brown, brown_per)
            g_m_green = max(g_m_green, green_per)

        if g_state == State.ERROR_LINE_CHECK:
            red_line = error_line_check(parking_img, g_side, g_lower_red, g_upper_red)
            brown_line = error_line_check(parking_img, g_side, g_lower_brown, g_upper_brown)
            green_line = error_line_check(parking_img, g_side, g_lower_green, g_upper_green)
            g_red_line = max(g_red_line, red_line)
            g_brown_line = max(g_brown_line, brown_line)
            g_green_line = max(g_green_line, brown_line)

            print('Line check:',
                  'red', red_line,
                  'brown', brown_line,
                  'green', green_line)
            if 'T' in g_side:
                cv2.line(img,  # T
                         (g_xmax_line, g_ymin_line),
                         (g_xmin_line, g_ymin_line),
                         (255, 0, 0), 2)
                cv2.putText(img, 'T', (5, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            if 'B' in g_side:
                cv2.line(img,  # B
                         (g_xmin_line, g_ymax_line),
                         (g_xmax_line, g_ymax_line),
                         (255, 0, 0), 2)
                cv2.putText(img, 'B', (15, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            if 'L' in g_side:
                cv2.line(img,  # L
                         (g_xmin_line, g_ymin_line),
                         (g_xmin_line, g_ymax_line),
                         (255, 0, 0), 2)
                cv2.putText(img, 'L', (25, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            if 'R' in g_side:
                cv2.line(img,  # R
                         (g_xmax_line, g_ymax_line),
                         (g_xmax_line, g_ymin_line),
                         (255, 0, 0), 2)
                cv2.putText(img, 'R', (35, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.rectangle(img, (g_xmin, g_ymin), (g_xmax, g_ymax), (0,0,255), 2)
        # cv2.rectangle(img,
        #               (g_xmin_line, g_ymin_line),
        #               (g_xmax_line, g_ymax_line),
        #               (255, 0, 0), 2)

    cv2.putText(img, str(g_state), (3, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)


def change_state(state):
    global g_state
    g_state = state
    print(g_state)


def parking_check(car: str, par: str) -> typing.Optional[bool]:
    if car == 'empty' and par == 'aruco':
        return None
    elif car == 'empty' or par == 'aruco':
        return True
    elif car == 'red' and par == 'yellow':
        return True
    elif car == 'green' and par == 'blue':
        return True
    else:
        return False


if __name__ == "__main__":
    ALTITUDE = 1.1
    # TIME_FOR_CALIBRATE = 3
    TIME_FOR_COLORS = 3
    TIME_FOR_LINES = 1

    change_state(State.UP)
    navigate_wait(z=ALTITUDE, auto_arm=True)

    change_state(State.HOME)
    navigate_wait(z=ALTITUDE, frame_id='aruco_24')
    rospy.sleep(1)
    navigate_wait(z=ALTITUDE, yaw=math.radians(0), frame_id='aruco_map')

    change_state(State.FINDM)
    # rospy.sleep(TIME_FOR_CALIBRATE)
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
        g_xmax_line = g_xmax + PARKING_LINE_OFFSET
        g_xmin_line = g_xmin - PARKING_LINE_OFFSET
        g_ymax_line = g_ymax + PARKING_LINE_OFFSET
        g_ymin_line = g_ymin - PARKING_LINE_OFFSET
        print(g_xmin, g_xmax, g_ymin, g_ymax)
        break

    count_car = 0
    count_invalid = 0
    count_abonement = 0
    count_normal = 0
    count_error = 0
    f = open('C_report_fly_F_I.txt', 'wt+')

    change_state(State.ROUTE)
    for x,y,side in [(0,1,'TLR'),  # 20
                (0,2,'TR'),  # 16
                (0,3,''),  # 12
                (0,4,'TBLR'),  # 8
                (0,5,'TLR'),  # 4
                (0,6,'R'),  # 0!
                # (1,4,'BLR'),  # 9
                # (1,3,'BLR'),  # 13
                # (1,2,'BLR'),  # 17
                # (1,1,'BLR'),  # 21
                # (1,0,'BLR'),  # 25
                ]:
        print(f'go to {x} {y}')
    
        change_state(State.MOVE)
        navigate_wait(x=x, y=y, z=ALTITUDE, frame_id='aruco_map', yaw=math.radians(0))
        rospy.sleep(1)
        navigate_wait(x=x, y=y, z=ALTITUDE, frame_id='aruco_map', yaw=math.radians(0))

        change_state(State.STOP)
        rospy.sleep(1)

        g_side = side
        g_red_line = 0
        g_brown_line = 0
        g_green_line = 0
        change_state(State.ERROR_LINE_CHECK)
        rospy.sleep(TIME_FOR_LINES)
        change_state(State.STOP_ERROR_LINE_CHECK)
        rospy.sleep(0.3)

        line = max(g_red_line, g_brown_line, g_green_line)

        print('Lines:',
              'red', g_red_line,
              'brown', g_brown_line,
              'green', g_green_line,
              '-', 'line', line)

        g_m_blue = 0.0
        g_m_yellow = 0.0
        g_m_red = 0.0
        g_m_brown = 0.0
        g_m_green = 0.0
        change_state(State.COLORS)
        rospy.sleep(TIME_FOR_COLORS)
        change_state(State.STOP_COLORS)
        rospy.sleep(0.3)

        parking_color_m = max(
            [(g_m_blue, 'blue'),
             (g_m_yellow, 'yellow'),
             (15.0, 'aruco')]
        )
        car_color_m = max([
            (g_m_red, 'red'),
            (g_m_brown, 'brown'),
            (g_m_green, 'green'),
            (15.0, 'empty')
        ])
        car = car_color_m[1]
        parking = parking_color_m[1]
        print('Colors:',
              'blue', g_m_blue,
              'yellow', g_m_yellow,
              'red', g_m_red,
              'brown', g_m_brown,
              'green', g_m_green,
              '-', parking_color_m[1], int(parking_color_m[0]), car_color_m[1], int(car_color_m[0]), 'MAX')

        print('Result:', car, parking, line)
        f.write(f'x{x} y{y} car {car} parking {parking} line {line<10}\n')
        check = parking_check(car, parking)

        # подсчет количества машин
        if not car == 'empty':
            count_car += 1
        else:
            if line >= 10:
                count_car += 1

        # подсчет цветных машин
        if car == 'red':
            count_invalid += 1
        elif car == 'green':
            count_abonement += 1
        elif car == 'brown':
            count_normal += 1

        led_ok = True
        # подсчет ошибок в парковке
        if check is not None and check is False:
            count_error += 1
            led_ok = False
        elif line >= 10:
            count_error += 1
            led_ok = False

        if check is None:
            print("No car and sign.\n")
        elif led_ok:
            print("Parking is OK.\n")
            set_effect(b=0, g=255, r=0)
            rospy.sleep(2)
        else:
            print("Parking is BAD.\n")
            set_effect(b=0, g=0, r=255)
            rospy.sleep(2)
        set_effect(b=0, g=0, r=0)

    s = f'car {count_car} ' \
        f'invalid {count_invalid} ' \
        f'abonement {count_abonement} ' \
        f'normal {count_normal} ' \
        f'error {count_error}\n'
    print(s)
    f.write(s)
    f.close()

    change_state(State.BASE)
    navigate_wait(z=ALTITUDE, frame_id='aruco_24')
    rospy.sleep(1)
    navigate_wait(z=ALTITUDE, frame_id='aruco_24')

    change_state(State.LAND)
    land_wait()
    change_state(State.FINISHED)
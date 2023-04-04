# Information: https://clover.coex.tech/en/snippets.html#navigate_wait
# -*- coding: utf-8 -*-

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

rospy.init_node('flight')

markerN = None
markerX = None
markersFound = None


def markers_callback(msg):
    # алгоритм обнаруживает ближайший маркет, пока для простоты находим первый попавшийся маркер
    global markerX, markerN, markersFound
    if msg.markers and len(msg.markers) == 1:
        l = [m.id for m in msg.markers]
        if not l == markersFound:
            markersFound = l
            print('Detected markers:', markersFound)
        mX = msg.markers[0]
        markerN = mX.id
        markerX = {'x': mX.pose.position.x, 'y': mX.pose.position.y, 'z': mX.pose.position.z}
        #print('Detected markers:', l, mX.pose.position.x, mX.pose.position.y, mX.pose.position.z)
        #print(mX)
        #print('pose', mX.pose.position.x, mX.pose.position.y, mX.pose.position.z)
        #print('orie', mX.pose.orientation.x, mX.pose.orientation.y, mX.pose.orientation.z, mX.pose.orientation.w)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), yaw_rate=0, speed=0.5, \
        frame_id='body', tolerance=0.2, auto_arm=False):

    res = navigate(x=x, y=y, z=z, yaw=yaw, yaw_rate=yaw_rate, speed=speed, \
        frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        # print('Высота подъёма {z}, погрешность={t}'.format(z=telem.z, t=math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)))
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)
        #rospy.spin()


def calc_offsets(offset, pos1, pos3, current_pos):
    # складываю расстояние до уже ранее найденных плиток с расстоянием,
    # на которое хочу пролететь
    if pos1:
        pos1['x'] += -offset['x']
        pos1['y'] += -offset['y']
        print('Изменилось расстояние до плитки 1:', pos1)
    if pos3:
        pos3['x'] += -offset['x']
        pos3['y'] += -offset['y']
        print('Изменилось расстояние до плитки 3:', pos3)

    # Обнуляем координаты как только переместились к искомым плиткам
    if current_pos == 1:
        pos1 = {'x': 0, 'y': 0}
    if current_pos == 3:
        pos3 = {'x': 0, 'y': 0}
    
    print('Расстояние от дрона до плитки 1:', pos1)
    print('Расстояние от дрона до плитки 3:', pos3) 

    return pos1, pos3


if __name__ == '__main__':
    rospy.init_node('flight')
    rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)
    #navigate_wait(y=1, frame_id='body')

    markerX = None
    markerN = None
    previousN = None
    pos1 = None
    pos3 = None
    markers = [0]*11

    print('Подъём на 0.5 метра')
    navigate_wait(z=0.5, frame_id='body', auto_arm=True)

    # поворот в исходное положение
    print('Поворот в исходное положение')
    navigate_wait(yaw=math.radians(-90), frame_id='body')  # , yaw_rate=math.pi/4
    rospy.sleep(2)

    while markerN:
        i = 0
        while True:
           
            if markerN == markers[i]:
                break
            else:
                if not markers[i] == 0:
                    i += 1
                else:
                    markers[i] = markerN
                    print('used markers:', markers)
                    continue

        if previousN == markerN:
            break
        previousN = markerN

        steps = (markerN & 0x1c) >> 2
        direction = markerN & 0x03
        replay = markerN >> 5
        print('Найден маркер #{n} (шаги={s}, направление={d}, код={r}, x={x} y={y} z={z})'.
              format(n=markerN, s=steps, d=direction, r=replay, x=markerX['x'], y=markerX['y'], z=markerX['z']))

        markerN = None

        while True:
            offset = math.sqrt(markerX['x'] ** 2 + markerX['y'] ** 2)
            print('Зависаем над маркером #{n}, смещение={o}'.format(n=previousN, o=offset))
            tolerance = 0.1
            if offset > tolerance:
                navigate_wait(x=-markerX['y'], y=-markerX['x'], frame_id='body', tolerance=tolerance)
                if not markerN:
                    print('Центр маркера #{n} потерян'.format(n=previousN))
                    while not markerN:
                        print('Подъём на 0.4 метрa, поиск маркера')
                        navigate_wait(z=0.4, frame_id='body')
                    print('Спуск до 0.5 метрa')
                    navigate_wait(z=0.5-markerX['z'], frame_id='body')
                    break
            else:
                print('Центр маркера #{n} найден, смещение={o}'.format(n=previousN, o=offset))
                break

        if direction == 0:
            print('Без поворота')
        elif direction == 1:
            print('Поворот по часовой')
            navigate_wait(yaw=math.radians(-90), frame_id='body')  # , yaw_rate=math.pi/4
            rospy.sleep(2)
        elif direction == 2:
            print('Разворот')
            navigate_wait(yaw=math.radians(180), frame_id='body')  # , yaw_rate=math.pi/4
            rospy.sleep(4)
        else:
            print('Поворот против часовой')
            navigate_wait(yaw=math.radians(90), frame_id='body')  # , yaw_rate=math.pi/4
            rospy.sleep(2)

        print('Летим прямо на {s}'.format(s=steps))
        navigate_wait(y=steps, frame_id='body')

        while previousN == markerN:
            if steps == 0:
                print("----------\nПосадка")
                land()
                break
            else:
                print('Подъём на 0.4 метрa, поиск маркера')
                navigate_wait(z=0.4, frame_id='body')
        print('Спуск до 0.5 метрa')
        navigate_wait(z=0.5-markerX['z'], frame_id='body')
        

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
    #m0 = next((marker for marker in msg.markers if marker.id == 3), None)
    #if m0:
    #    print('  #0: ', {'x': m0.pose.position.x, 'y': m0.pose.position.y, 'z': m0.pose.position.z})


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

    auto_arm = True
    markerN = 0
    markerX = None
    pos1 = None
    pos3 = None
    
    for n in range(5):
        # поиск маркера n
        while not markerX:
            # особенность работы функции navigate_wait - она ожидает пока дрон не прилетит
            # в указанную точку, оценка прилета выполняется с учетом погрешности (потому как
            # дрон сдувается ветром, и прилетает он не в указанную точку, а внутрь сферы
            # вокруг этой точки); функция меряет погрешность прилета в сферу, по умолчанию
            # радиус сферы = 20 см
            # ---
            # Когда дрон взлетает его немного сдувает, но взлет на 0.5м выполняется до границы 
            # сферы (которая R=20см) выполняется с недолетом около 16 см; итого: 6 подъемов 
            # по 0.5м это не 3м, а 2м. То есть дрон взлетает на мельшую высоту (ехехехехе)
            print('Подъём на 0.5 метрa, поиск маркера #{n}'.format(n=markerN))
            navigate_wait(z=0.5, frame_id='body', auto_arm=auto_arm)
            auto_arm = False

        # полёт к маркеру на высоте 1 метр
        if markerX:
            # мы не пользуемся измерениями телеметрии, поэтому точная высота подъема нам
            # не известна, однако высотой мы считаем расстояние до маркера по Z.
            current_elevation = markerX[2]
            offset_yxz = {'y': -markerX[0], 'x': -markerX[1], 'z': 1.0 - current_elevation}
            print('Маркер #{n} найден со смещением'.format(n=markerN), offset_yxz)

            # во время полета к текущему маркеру ищем следующий маркер
            markerN = n + 1
            markerX = None
            
            # запоминаем во время полета смещения всех точек 
            print('Полёт к маркеру #{n}'.format(n=n), offset_yxz)
            navigate_wait(x=offset_yxz['x'], y=offset_yxz['y'], z=offset_yxz['z'], frame_id='body')
            print('Прилетели к маркеру #{n}'.format(n=n))
            pos1, pos3 = calc_offsets(offset_yxz, pos1, pos3, n)

    # последнее измерение высоты current_elevation было известно на последнем шаге облета
    # в нашем случае это 4 маркер, к которому мы прилетим в зону сферы, то есть на высоту
    # от 120 до 80 см (см. комментарий выше)
    print("----------\nОблёт маркеров закончен\n")
    
    # полет к маркеру 1 с запоминанием координат маркера 3 и задержкой
    if pos1:
        offset = {'x': pos1['x'], 'y': pos1['y'], 'z': 0}
        # запоминаем во время полета смещения всех точек
        print('Полёт к маркеру {n} по накопленным координатам:'.format(n=1), offset)
        navigate_wait(x=offset['x'], y=offset['y'], z=offset['z'], frame_id='body')
        print('Прилетели к маркеру #{n}'.format(n=1))
        pos1, pos3 = calc_offsets(offset, pos1, pos3, 1)

    # полет к маркеру 3 с запоминанием координат маркера 1 и задержкой
    if pos3:
        offset = {'x': pos3['x'], 'y': pos3['y'], 'z': 0}
        # запоминаем во время полета смещения всех точек
        print('Полёт к маркеру {n} по накопленным координатам:'.format(n=3), offset)
        navigate_wait(x=offset['x'], y=offset['y'], z=offset['z'], frame_id='body')
        print('Прилетели к маркеру #{n}'.format(n=3))
        pos1, pos3 = calc_offsets(offset, pos1, pos3, 3)

    print("----------\nПосадка")
    land()

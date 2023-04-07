# -*- coding: utf-8 -*-

#импорт необходимых модулей и библиотек
import rospy
import cv2 as cv
import math
from clover import srv
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover.srv import SetLEDEffect

#инициализация ноды (программы)
rospy.init_node('flight')

#инициализация используемых сервисов 
bridge = CvBridge()
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

#создание топика для просмотра распознанного изображения
color_debug = rospy.Publisher("/color_debug", Image)

markerN = None
markerX = None
markersFound = None


def markers_callback(msg):
    # алгоритм обнаруживает ближайший маркет, пока для простоты находим первый попавшийся маркер
    global markerX, markerN, markersFound
    if msg.markers:
        l = [m.id for m in msg.markers]
        if not l == markersFound:
            markersFound = l
            print('Detected markers:', markersFound)
        if len(msg.markers) == 1:
            mX = msg.markers[0]
            markerN = mX.id
            markerX = {'x': mX.pose.position.x, 'y': mX.pose.position.y, 'z': mX.pose.position.z}
            #print('Detected markers:', l, mX.pose.position.x, mX.pose.position.y, mX.pose.position.z)
            #print(mX)
            #print('pose', mX.pose.position.x, mX.pose.position.y, mX.pose.position.z)
            #print('orie', mX.pose.orientation.x, mX.pose.orientation.y, mX.pose.orientation.z, mX.pose.orientation.w)
        elif len(msg.markers) >= 2:
            # поиск ближайшего маркера
            l2 = None
            for m in msg.markers:
                x = m.pose.position.x
                y = m.pose.position.y
                z = m.pose.position.z
                l1 = math.sqrt(x**2 + y**2 + z**2)
                if l2 is None or l2 > l1:
                    l2 = l1
                    markerN = m.id
                    markerX = {'x': x, 'y': y, 'z': z}
            print('Ближайший маркер #{n} на дистанции {l2}'.format(n=markerN, l2=l2))
            

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


#объявление процедуры check_temp, при вызове которой будет распознавание цветов
def check_temp():
    #создание переменной frame, где будет храниться изображение с камеры. Изображение это постоянно обрабатывается. В квадратных скобках определена рамка, откуда берется изображение. Кодировка изображения brg8 (8-ми битное изображение с кодировкой BGR)
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')[100:140, 130:170]  
    # print(frame)
   # задание для каждого цвета диапазона в кодировке BGR. В результате в каждую из 
   # переменных запишется бинарное представление цвета (0 - если цвет не попадет в 
   # диапазон и 255 - если попадает). Это бинарное представление представлено в 
   # виде матрицы.
    red = cv.inRange(frame, (0, 0, 220), (0, 0, 255))
    yellow = cv.inRange(frame, (0, 220, 220), (0, 255, 255))
    green = cv.inRange(frame, (94, 220, 31), (134, 255, 71))

    #зададим словарь color, где запишем для каждого ключа 'r', 'y', 'g' значения
    #из бинарной матрицы только не нулевые значения
    color = {'r': cv.countNonZero(red),
             'y': cv.countNonZero(yellow),
             'g': cv.countNonZero(green)}
    
    #публикация топика для просмотра распознанного изображения
    color_debug.publish(bridge.cv2_to_imgmsg(frame, 'bgr8')) 
   
    #зададим условие, если максимальное значение из словаря равно ключу 'y', 
    # тогда выводим на экран сообщение sbrosheno. Простыми словами, если он 
    # увидел желтый цвет, тогда выводить сообщение sbrosheno
    if max(color, key=color.get) == 'y':
        print('yellow')
    elif max(color, key=color.get) == 'r':
        print('red')
    elif max(color, key=color.get) == 'g':
        print('green')
    else:
        print('no color') 

print('подъем на 1 метр')
navigate(x=0, y=0, z=1, speed=1, frame_id='body', auto_arm=True)
rospy.sleep(5)

print('перелет на координаты x=2, y=0')
navigate_wait(x=2, y=0, z=0, speed=1, frame_id='body')
rospy.sleep(3)
check_temp()

print('перелет на координаты x=1, y=2')
navigate_wait(x=-1, y=2, z=0, speed=1, frame_id='body')
rospy.sleep(3)
check_temp()

print('перелет на координаты x=3, y=3')
navigate_wait(x=2, y=1, z=1, speed=0.5, frame_id='body')
rospy.sleep(5)
check_temp()

print('перелет на изначальное местоположение')
navigate_wait(x=-3, y=-3, z=0, speed=1, frame_id='body')
rospy.sleep(5)
print('land')
land()
#raw_input

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
            

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), yaw_rate=0, speed=0.5, \
        frame_id='body', tolerance=0.2, auto_arm=False):

    res = navigate(x=x, y=y, z=z, yaw=yaw, yaw_rate=yaw_rate, speed=speed, \
        frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        print('Высота подъёма {z}, погрешность={t}'.format(z=telem.z, t=math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2)))
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)
        #rospy.spin()

flag = 2
#объявление процедуры check_temp, при вызове которой будет распознавание цветов
def get_azimuth():
    #создание переменной frame, где будет храниться изображение с камеры. Изображение это постоянно обрабатывается. В квадратных скобках определена рамка, откуда берется изображение. Кодировка изображения brg8 (8-ми битное изображение с кодировкой BGR)
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')[100:140, 130:170]  
    #публикация топика для просмотра распознанного изображения
    color_debug.publish(bridge.cv2_to_imgmsg(frame, 'bgr8')) 

    #print(len(frame))
    for x in range(2):
        for y in range(2):
            p = frame[19+x][19+y]
            print('центр', p)
    left = 0
    right = 0
    top = 0
    bottom = 0
    for i in range(20):
        pl = frame[19][19-i] # слева
        pr = frame[19][19+i] # справа
        pt = frame[19-i][19] # сверху
        pb = frame[19+i][19] # снизу
        if pl[0] <= 15 and pl[1] <= 15 and pl[2] >= 220:
            left += 1
        if pr[0] <= 15 and pr[1] <= 15 and pr[2] >= 220:
            right += 1
        if pt[0] <= 15 and pt[1] <= 15 and pt[2] >= 220:
            top += 1
        if pb[0] <= 15 and pb[1] <= 15 and pb[2] >= 220:
            bottom += 1
    print('слева={}, справа={}, сверху={}, снизу={}'.format(left, right, top, bottom))
    if left == 0:
        if right == 0:
            return None
        elif right <= 10:
            return -15.0
        else:
            return -7.5
    if right == 0:
        if left == 0:
            return None
        elif left <= 10:
            return 15.0
        else:
            return 7.5
    if left == right:
        return 0.0
    elif left < right:
        return -7.5 + left/right * 7.5
    else:
        return 7.5 - right/left * 7.5
    """
    #global flag
    #if flag > 0:
    #    for x in enumerate(frame):  # list -> (0, list[0]), (1, list[1]), ...
    #        for y in enumerate(x[1]):  # list(x[1])  ->(0, list[0][0]), (1, list[0][1]), ...
    #            print(x[0], y[0], y[1])
    #    flag -= 1
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
    """ 


if __name__ == '__main__':
    #инициализация ноды (программы)
    rospy.init_node('flight')

    print('подъем на 1 метр')
    navigate_wait(x=0, y=0, z=1.5, speed=0.5, auto_arm=True)
    navigate_wait(x=1, y=0, z=0, speed=0.5)

    azimuth = 0.0
    while True:
        a = get_azimuth()
        if a is None:
            break
        if a == 0:
            azimuth = 0.0
            navigate_wait(x=0.5, speed=0.25, tolerance=0.1)
        else:
            azimuth += a/4.0
            print('новый азимут', a/4.0, 'текущий азимут', azimuth)
            navigate_wait(yaw=math.radians(azimuth))
            rospy.sleep(2)
            navigate_wait(x=0.5, speed=0.25, tolerance=0.1)

    print('land')
    land()
    #raw_input

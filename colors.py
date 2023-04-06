#импорт необходимых модулей и библиотек
import rospy
import cv2 as cv
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


#объявление процедуры check_temp, при вызове которой будет распознавание цветов
def check_temp():
    #создание переменной frame, где будет храниться изображение с камеры. Изображение это постоянно обрабатывается. В квадратных скобках определена рамка, откуда берется изображение. Кодировка изображения brg8 (8-ми битное изображение с кодировкой BGR)
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')[110:130, 140:160]  
    print(frame)
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
        print('y')
    elif max(color, key=color.get) == 'r':
        print('r')
    elif max(color, key=color.get) == 'g':
        print('green')
    else:
        print('no color') 

navigate(x=0, y=0, z=1, speed=1, frame_id='body', auto_arm=True)
rospy.sleep(5)

navigate(x=2, y=0, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(7)
check_temp()

navigate(x=1.2, y=1.5, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(7)
check_temp()

navigate(x=2.5, y=2.5, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(7)
check_temp()

navigate(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(7)

print('land')
land()
#raw_input

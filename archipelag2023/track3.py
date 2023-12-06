import rospy
import math
from pyzbar import pyzbar
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('cv')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

det = True
fin = False

def goto(x=0, y=0, z=0.5, yaw=float('nan'), speed=0.6, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 5) < tolerance:
            break
        rospy.sleep(0.2)

def flyto(list):
    global fin
    for i in list:
        if i == 1:
            goto(x=0.1, z=1, frame_id='aruco_63')
            rospy.sleep(2)
            goto(x=0.1, z=1, frame_id='aruco_11')

        elif i == 2:
            goto(z=1, frame_id='aruco_6')
            rospy.sleep(2)
            goto(z=1, frame_id='aruco_4')
        
        elif i == 3:
            goto(x=-0.5, z=1, frame_id='aruco_9')
            rospy.sleep(2)
            goto(x=-0.5, z=1, frame_id='aruco_23')

        elif i == 4:
            goto(z=1, frame_id='aruco_60')
            rospy.sleep(2)
            goto(z=1, frame_id='aruco_78')

        elif i == 5:
            goto(x=0.1, y=-0.5, z=2.2, frame_id='aruco_63')
            rospy.sleep(3)
            goto(x=0.1, z=2.2, frame_id='aruco_11')
        
        elif i == 6:
            goto(x=-0.5, z=2.2, frame_id='aruco_9')
            rospy.sleep(2)
            goto(z=2.2, frame_id='aruco_23')
    

        

goto(z=1.01, frame_id='body', auto_arm=True, speed=0.3)
rospy.sleep(1)
goto(y=1.01, frame_id='aruco_61')
rospy.sleep(5)

flyto([0,1,2,3,4,5,6])
land()

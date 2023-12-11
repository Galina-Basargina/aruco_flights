# Information: https://clover.coex.tech/programming

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
from clover import srv
import numpy as np
from std_srvs.srv import Trigger

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# coder BGR to HSV: https://colordesigner.io/convert/rgbtohsv
# color for bright red
# lower_red = np.array([0, 0, 80])  # hsv(0,100,31)
# upper_red = np.array([40, 40, 255])  # hsv(0,84,100)
# color for dark red
lower_red = np.array([0, 0, 140])
upper_red = np.array([115, 115, 255])

rospy.init_node('flight')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image)

@long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    frame = cv2.inRange(img, lower_red, upper_red)
    contours, _hierarchy = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0,255,0), 3)
    for c in contours:
        min_x, min_y = max_x, max_y = c[0][0]
        for ci in c:
            x,y = ci[0]
            if x < min_x: min_x = x
            elif x > max_x: max_x = x
            if y < min_y: min_y = y
            elif y > max_y: max_y = y
        cv2.circle(img, ((max_x-min_x)//2+min_x, (max_y-min_y)//2+min_y), 3, (0,255,0), -1)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)



print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

# Wait for 5 seconds
rospy.sleep(5)

print('Fly forward 1 m')
navigate(x=0, y=0, z=1, frame_id='aruco_92')

# Wait for 5 seconds
rospy.sleep(8)

print('Perform landing')
land()


import rospy
import cv2
import typing
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
import numpy as np
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray
from clover.srv import SetLEDEffect
from pyzbar import pyzbar

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service


rospy.init_node('drone_vision')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image)
font = cv2.FONT_HERSHEY_SIMPLEX


def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    barcodes = pyzbar.decode(img)
    if barcodes:
        # print(barcodes[0].polygon)
        print(barcodes[0].data.decode('utf-8'))
        cv2.putText(img, barcodes[0].data.decode('utf-8'), (1, 20), font, 1, (0, 0, 255), 2)
        for i in range(4):
            cv2.line(img,
                     (barcodes[0].polygon[i].x, barcodes[0].polygon[i].y),
                     (barcodes[0].polygon[(i+1) % 4].x, barcodes[0].polygon[(i+1) % 4].y),
                     (0, 0, 255), 2)
        # crash?
        g_qr_data: str = barcodes[0].data.decode('utf-8')
        qr_data = g_qr_data.split(' ')
        if qr_data[0] == 'b':
            qr_data = qr_data[1:]
        print(g_qr_data, qr_data)
        for i in range(len(qr_data) // 2):
            print('----- -----')
            xb = qr_data[i * 2]
            yb = qr_data[i * 2 + 1]
            print(1, xb, yb, type(xb), type(yb))
            x, y = float(xb), float(yb)
            print(2, x, y)
            print(f'Go to {x}, {y}')
    # publish image (to browser)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)


if __name__ == '__main__':
    while True:
        rospy.sleep(1)

#!/bin/python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState

rospy.init_node('robot')

set_link_state = rospy.ServiceProxy('gazebo/set_link_state', SetLinkState)

CENTER = (0.5, 1.2)
RADIUS = 1.0
SPEED = 0.3

sleep_rate = rospy.Rate(10)
start_stamp = rospy.get_rostime()
while True:
    angle = (rospy.get_rostime() - start_stamp).to_sec() * SPEED
    x = CENTER[0] + math.sin(angle) * RADIUS
    y = CENTER[1] + math.cos(angle) * RADIUS

    print(f"x={x}, y={y}")
    set_link_state(LinkState(link_name='aruco_100::marker_100_link',
                             pose=Pose(position=Point(x, y, 0.01),
                             orientation=Quaternion(0, 0, 0, 1))))

    sleep_rate.sleep()

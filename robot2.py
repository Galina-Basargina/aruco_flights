# Information: https://clover.coex.tech/en/snippets.html#navigate_wait

import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray

rospy.init_node('robot1')


get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0.0, y=0.0, z=0.0, yaw=float('nan'), yaw_rate=0, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)  # yaw_rate=yaw_rate,
    if not res.success:
        return res
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)
        # rospy.spin()


def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)


def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)


Radius = 0.15
g_robot_id = 100

class Global:
    def __init__(self):
        self.x_robot = None
        self.y_robot = None
        self.marker_found: int = 0


g_vars: Global = Global()


def markers_callback(msg):
    global g_vars
    m = next((m.pose.position for m in msg.markers if m.id == g_robot_id), None)
    # print(next((m for m in msg.markers if m.id == g_robot_id), None))
    if m is not None:
        # drone
        t = get_telemetry(frame_id='aruco_map')
        g_vars.x_drone = t.x
        g_vars.y_drone = t.y
        g_vars.z_drone = t.z
        # robot
        # y = dy - ry
        # x = dx - rx
        g_vars.x_robot = t.x - m.y
        g_vars.y_robot = t.y - m.x

        if g_vars.marker_found < 0:
            g_vars.marker_found = 0
        g_vars.marker_found += 1
    else:
        if g_vars.marker_found > 0:
            g_vars.marker_found = 0
        g_vars.marker_found -= 1


rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)


ALTITUDE = 1
YAW = math.radians(0)

if __name__ == '__main__':
    # navigate_wait(z=3, auto_arm=True)
    navigate_wait(frame_id='body', x=0, y=0, z=ALTITUDE, auto_arm=True)
    print('up done')

    navigate_wait(x=1, y=1, z=ALTITUDE, frame_id='aruco_map', yaw=YAW)
    rospy.sleep(1)
    navigate_wait(x=1, y=1, z=ALTITUDE, frame_id='aruco_map', yaw=YAW)
    print('center done')
    
    flag = 0
    while True:
        rx, ry = (g_vars.x_robot, g_vars.y_robot)
        if ry is None:
            rospy.sleep(0.1)
            continue

        t = get_telemetry(frame_id='aruco_map')
        dx, dy, dz = (t.x, t.y, t.z)

        s = f'drone: {round(dx, 1)} {round(dy, 1)} {round(dz, 1)}'
        s += f' - robot: {round(rx, 1)} {round(ry, 1)}'

        rx = rx - t.x
        ry = ry - t.y
        s += f' - dist: {round(rx, 1)} {round(ry, 1)}'

        if dz < 0.31:
            print(s)
            break

        if g_vars.marker_found < -30:
            s += ' - robot gone'
            print(s)
            navigate_wait(x=1, y=1, z=ALTITUDE, frame_id='aruco_map', yaw=YAW)
            rospy.sleep(1)
            navigate_wait(x=1, y=1, z=ALTITUDE, frame_id='aruco_map', yaw=YAW)
            print('center done')
            continue

        x = dx + rx
        y = dy + ry
        z = dz #- 0.3
        dist = math.sqrt(rx ** 2 + ry ** 2)

        if dist >= 0.8:
            part = 0.8
        elif dist >= 0.6:
            part = 0.6
        elif dist >= 2*Radius:
            part = 0.3
        else:
            part = 0.1  # 0.05 => 10 Hz, 0.5 m/s

        if dist <= part:
            pass
        else:
            steps = dist / part # >= 1
            x = dx + rx / steps  # 1m ---> 0.05m
            y = dy + ry / steps

        if dist <= Radius:
            z = z - 0.1

        s += f' - move: {round(x, 2)} {round(y, 2)} {round(z, 1)} - dist: {round(dist, 2)} part: {round(part, 2)}'
        if g_vars.marker_found < 0:
            s += ' - search'
        else:
            s += f' - {g_vars.marker_found}'
        #navigate_wait(x=x, y=y, z=z, tolerance=Radius, frame_id='aruco_map', yaw=YAW)
        set_position(x=x, y=y, z=z, frame_id='aruco_map', yaw=YAW)
        flag = 1
        print(s)
            
    # navigate_wait(frame_id='aruco_14', z=1)
    print('land')
    land_wait()
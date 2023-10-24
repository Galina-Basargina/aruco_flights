"""
Install: sudo apt install python3-urllib3
Или:     python3 -m pip install -r requirements.txt
"""
from http.server import BaseHTTPRequestHandler
from http.server import HTTPServer
import json
from urllib.parse import urlparse
from urllib.parse import parse_qs

import rospy
from clover import srv
from std_srvs.srv import Trigger

def run (server_class=HTTPServer, handler_class=BaseHTTPRequestHandler):
    server_address = ('', 8081)
    httpd = server_class(server_address, handler_class)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        httpd.server_close()

class BrowserGetHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(
            '<html><head><meta charset="utf-8"></head>'
            '<body>Hello World!)</body></html>'.encode()
        )
        
class TelegaGetHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        rospy.init_node('flight')

        get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        land = rospy.ServiceProxy('land', Trigger)


        def flight (marker):
            navigate(x=0, y=0, z=0.5, frame_id='body', auto_arm=True)
            rospy.sleep(5)
            navigate(x=0, y=0, z=0.5, frame_id=marker)
            rospy.sleep(5)
            land() 
            rospy.sleep(10)
            navigate(x=0, y=0, z=0.5, frame_id='body', auto_arm=True)
            rospy.sleep(5)
            navigate(x=0, y=0, z=0.5, frame_id='aruco_0')
            rospy.sleep(5)
            land() 

        #print(self.path)
        o = urlparse(self.path)
        #print(o)
        q = parse_qs(o.query)
        print("\n")
        print(q)
        print("\n")
        if q.get('do'):
            do = q.get('do')
            if len(do) == 1 and do[0] == 'Move':
                if q.get('street'):
                    street = q.get('street')
                    if len(street) == 1:
                        self.send_response(200)
                        self.send_header("Content-type", "applicaiton/json")
                        self.end_headers()
                        a = {'do': do[0]}
                        # .. начало выполнения команды
                        if street == ['1']:
                            # летим на ул. Ватутина
                            marker = 'aruco_12'
                            flight(marker)
                        elif street == ['2']:
                            # летим на Витебский пр.
                            marker = 'aruco_2'
                            flight(marker)
                        elif street == ['3']:
                            # летим на ул. Садовая
                            marker = 'aruco_7'
                            flight(marker)
                        elif street == ['4']:
                            # летим на Невский пр.
                            marker = 'aruco_17'
                            flight(marker)
                        a.update({'speed': 0.5, 'altitude': 1})
                        self.wfile.write(json.dumps(a).encode())
                        return
        self.send_response(200)
        self.send_header("Content-type", "applicaiton/json")
        self.end_headers()
        a = {'error': 'Неправильный формат команды'}
        self.wfile.write(json.dumps(a).encode())
                
if __name__ == "__main__":
    run(handler_class=TelegaGetHandler)


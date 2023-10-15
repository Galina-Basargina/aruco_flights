"""
Install: sudo apt install python3-urllib3
Или:     python3 -m pip install -r requirements.txt
"""
from http.server import BaseHTTPRequestHandler
from http.server import HTTPServer
import json
from urllib.parse import urlparse
from urllib.parse import parse_qs

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
        #print(self.path)
        o = urlparse(self.path)
        #print(o)
        q = parse_qs(o.query)
        print(q)
        if q.get('do'):
            do = q.get('do')
            if len(do) == 1:
                self.send_response(200)
                self.send_header("Content-type", "applicaiton/json")
                self.end_headers()
                a = {'name': 'Hello, World!', 'do': do[0], 'speed': 1}
                self.wfile.write(json.dumps(a).encode())
                return
        self.send_response(403)
        self.end_headers()
                
if __name__ == "__main__":
    run(handler_class=TelegaGetHandler)


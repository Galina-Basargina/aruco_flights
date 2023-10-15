import json
import requests
from urllib.parse import urlparse
from urllib.parse import parse_qs

def run(address='127.0.0.1'):
    url = f'http://{address}:8081/'
    #command = {'do': 'Land'}
    #command = {'do': 'Move', 'street': 'Vatutina', 'house': 50}
    command = {'do': 'Start', 'altitude': 4}
    res = requests.get(url, params=command)
    data = res.json()
    print(data)
    

if __name__ == "__main__":
    run()


#!/bin/python3

import cv2
from pyzbar import pyzbar
import sys

filename = '../model_editor_models/aruco_marker_2_17-qr.png'
if len(sys.argv) >= 2:
    filename = sys.argv[1]

source = cv2.imread(filename)
barcodes = pyzbar.decode(source)

print(barcodes)
if barcodes:
    print(barcodes[0].data.decode('utf-8'))

    # crash?
    g_qr_data = barcodes[0].data.decode('utf-8')
    qr_data = g_qr_data.split(' ')
    for i in range(len(qr_data) // 2):
        x, y = float(qr_data[i * 2]), float(qr_data[i * 2 + 1])
        print(f'Go to {x}, {y}')

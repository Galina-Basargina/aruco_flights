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

import sys
import os
import cv2


g_b_min = 0
g_g_min = 0
g_r_min = 0
g_b_max = 0
g_g_max = 0
g_r_max = 0


def b_min(a): 
    global g_b_min 
    g_b_min = a
def g_min(a): 
    global g_g_min 
    g_g_min = a
def r_min(a): 
    global g_r_min 
    g_r_min = a
def b_max(a): 
    global g_b_max 
    g_b_max = a
def g_max(a): 
    global g_g_max 
    g_g_max = a
def r_max(a): 
    global g_r_max 
    g_r_max = a


if __name__ == '__main__':
    filename = './home_colors.bmp'

    frame = cv2.imread(filename)
    cv2.imshow('color', frame)

    cv2.createTrackbar('B_min', 'color', 0, 255, b_min)
    cv2.createTrackbar('G_min', 'color', 0, 255, g_min)
    cv2.createTrackbar('R_min', 'color', 0, 255, r_min)
    cv2.createTrackbar('B_max', 'color', 0, 255, b_max)
    cv2.createTrackbar('G_max', 'color', 0, 255, g_max)
    cv2.createTrackbar('R_max', 'color', 0, 255, r_max)

    while True:
        mask = cv2.inRange(frame, (g_b_min, g_g_min, g_r_min), (g_b_max, g_g_max, g_r_max))
        cv2.imshow('mask', mask)

        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow('result', result)

        cv2.waitKey(10)
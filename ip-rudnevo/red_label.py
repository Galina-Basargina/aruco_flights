# Information: https://clover.coex.tech/programming
import numpy as np
import cv2

# coder BGR to HSV: https://colordesigner.io/convert/rgbtohsv
# color for bright red
# lower_red = np.array([0, 0, 80])  # hsv(0,100,31)
# upper_red = np.array([40, 40, 255])  # hsv(0,84,100)
# color for dark red
lower_red = np.array([0, 0, 140])
upper_red = np.array([115, 115, 255])

cv2.namedWindow( "result", cv2.WINDOW_NORMAL )

img = cv2.imread('red_label.jpeg')
#filtered_image = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
frame = cv2.inRange(img, lower_red, upper_red)
contours, _hierarchy = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(img, contours, -1, (0,255,0), 3)
cv2.imshow('result', img)

cv2.resizeWindow("result", 800, 600)
cv2.waitKey()
cv2.destroyAllWindows()
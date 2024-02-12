# Information: https://clover.coex.tech/en/snippets.html#navigate_wait

import numpy as np
import cv2
from matplotlib import pyplot as plt
import sys


filename = './rainbow.png'
if len(sys.argv) > 1:
    filename = sys.argv[1]
img = cv2.imread(filename)
assert img is not None, 'file not found'


def calc_color_percent(img, lower, upper) -> float:
    percent: float = 0.0
    mask = cv2.inRange(img, lower, upper)
    masked_img = cv2.bitwise_and(img, img, mask=mask)
    cv2.imshow('masked', masked_img)
    cv2.waitKey(5000)
    height = len(masked_img)
    width = len(masked_img[0])
    square = width * height

    count_non_black = 0
    for x in range(height):
        for y in range(width):
            b, g, r = masked_img[x][y]
            if (b,g,r) != (0,0,0):
                # print(b, g, r)
                count_non_black += 1
    percent = count_non_black / square * 100.0
    return percent

g_lower_red = np.array([0, 0, 135]) # 29,29,135
g_upper_red = np.array([131, 131, 255])
red_percent = calc_color_percent(img, g_lower_red, g_upper_red)
print(f'red = {red_percent} %')

g_lower_blue = np.array([50, 0, 0])  # 50, 50, 0
g_upper_blue = np.array([255, 134, 10])
blue_percent = calc_color_percent(img, g_lower_blue, g_upper_blue)
print(f'blue = {blue_percent} %')

g_lower_yellow = np.array([0, 70, 70])
g_upper_yellow = np.array([55, 255, 255])
yellow_percent = calc_color_percent(img, g_lower_yellow, g_upper_yellow)
print(f'yellow = {yellow_percent} %')

g_lower_green = np.array([125, 104, 50])
g_upper_green = np.array([164, 255, 77])
green_percent = calc_color_percent(img, g_lower_green, g_upper_green)
print(f'green = {green_percent} %')

g_lower_brown = np.array([120, 150, 140])
g_upper_brown = np.array([255, 197, 180])
brown_percent = calc_color_percent(img, g_lower_brown, g_upper_brown)
print(f'brown = {brown_percent} %')

color = ('b', 'g', 'r')
for i, col in enumerate(color):
    hist = cv2.calcHist([img], [i], None, [256], [0, 256])
    # hist2, bins = np.histogram(img.ravel(), 256, [0, 256])
    # print(hist2, bins)
    x = [int(x) for x in hist]
    print(col, x, 'sum={s} max={m} at {i}'.format(s=sum(x), m=max(x), i=x.index(max(x))))
    # print(col, hist[1:])
    plt.plot(hist, color=col)
    plt.xlim([0, 256])
plt.show()

# plt.hist(img.ravel(), 256, [0, 256])
# plt.show()




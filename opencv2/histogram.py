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


def calc_weight(img, lower, upper) -> float:
    weight: float = 0.0
    mask = cv2.inRange(img, lower, upper)
    masked_img = cv2.bitwise_and(img, img, mask=mask)
    cv2.imshow('masked', masked_img)
    cv2.waitKey(5000)
    color = ('b', 'g', 'r')
    for i, col in enumerate(color):
        hist = cv2.calcHist([masked_img], [i], None, [256], [0, 256])
        for el, num in enumerate(hist):
            weight += el * int(num)
            # print(weight, el, num)
    return weight

g_lower_red = np.array([0, 0, 135]) # 29,29,135
g_upper_red = np.array([131, 131, 255])
red_weight = calc_weight(img, g_lower_red, g_upper_red)
print(f'red = {red_weight}')

g_lower_blue = np.array([50, 0, 0])  # 50, 50, 0
g_upper_blue = np.array([255, 134, 10])
blue_weight = calc_weight(img, g_lower_blue, g_upper_blue)
print(f'blue = {blue_weight}')

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




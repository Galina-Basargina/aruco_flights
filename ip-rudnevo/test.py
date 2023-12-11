# Information: https://clover.coex.tech/programming
import numpy as np
import cv2

def nothing(args):pass

cv2.namedWindow("setup")
cv2.createTrackbar("b1", "setup", 0, 255, nothing)
cv2.createTrackbar("g1", "setup", 0, 255, nothing)
cv2.createTrackbar("r1", "setup", 0, 255, nothing)
cv2.createTrackbar("b2", "setup", 255, 255, nothing)
cv2.createTrackbar("g2", "setup", 255, 255, nothing)
cv2.createTrackbar("r2", "setup", 255, 255, nothing)

fn = "red_mouse.png" # путь к файлу с картинкой
img = cv2.imread(fn) # загрузка изображения

while True:
    r1 = cv2.getTrackbarPos('r1', 'setup')
    g1 = cv2.getTrackbarPos('g1', 'setup')
    b1 = cv2.getTrackbarPos('b1', 'setup')
    r2 = cv2.getTrackbarPos('r2', 'setup')
    g2 = cv2.getTrackbarPos('g2', 'setup')
    b2 = cv2.getTrackbarPos('b2', 'setup')
    # собираем значения из бегунков в множества
    min_p = (g1, b1, r1)
    max_p = (g2, b2, r2)
    # применяем фильтр, делаем бинаризацию
    img_g = cv2.inRange(img, min_p, max_p)

    cv2.imshow('img', img_g)
    
    if cv2.waitKey(33) & 0xFF == ord('q'):
         break

cv2.destroyAllWindows()

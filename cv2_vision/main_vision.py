# функции с работой камеры дрона в ВM

import cv2
import sys
import vision

def start_video():
    capture = cv2.VideoCapture(0)
    capture.set(4, 640)
    capture.set(3, 480)

    vision.set_threshold_values(capture)
    # vision.set_bright_conrtast_image(capture)

    while True:
        success, source = capture.read()
        # xy - точка отсчета, x1y1 - точка конца
        extracted = vision.extract_image(source, 0, 0, 600, 400)
        # xy = до скольки пикселей уменьшаем
        resized = vision.resize_image(source, 200, 400)
        changed_image = vision.change_color_space(source, cv2.COLOR_BGR2HSV)
        blur = vision.blur_image(source, 20, 20)
        binary = vision.binary_image(source, (127, 127, 127),  (255, 255, 255))

        # cv2.imshow('binary', binary)
        # cv2.imshow('blur', blur)
        # cv2.imshow('extracted', extracted)
        # cv2.imshow('resized', resized)
        # cv2.imshow('changed_image', changed_image)
        cv2.imshow('source', source)



        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    capture.release()
    cv2.destroyAllWindows()

def trackbar_changed(value):
    pass


def start_file(filename):
    source = cv2.imread(filename)
    cv2.imshow('binary', source)
    cv2.createTrackbar('Bmin', 'binary', 0, 255, trackbar_changed)
    cv2.createTrackbar('Gmin', 'binary', 0, 255, trackbar_changed)
    cv2.createTrackbar('Rmin', 'binary', 0, 255, trackbar_changed)
    cv2.createTrackbar('Bmax', 'binary', 0, 255, trackbar_changed)
    cv2.createTrackbar('Gmax', 'binary', 0, 255, trackbar_changed)
    cv2.createTrackbar('Rmax', 'binary', 0, 255, trackbar_changed)
    while True:
        Bmin = cv2.getTrackbarPos('Bmin', 'binary')
        Rmin = cv2.getTrackbarPos('Rmin', 'binary')
        Bmax = cv2.getTrackbarPos('Bmax', 'binary')
        Gmin = cv2.getTrackbarPos('Gmin', 'binary')
        Gmax = cv2.getTrackbarPos('Gmax', 'binary')
        Rmax = cv2.getTrackbarPos('Rmax', 'binary')

        binary = vision.binary_image(source, (Bmin, Gmin, Rmin),  (Bmax, Gmax, Rmax))
        mask = cv2.bitwise_and(source, source, mask=binary)

        cv2.imshow('mask', mask)
        cv2.imshow('source', source)
        cv2.imshow('binary', binary)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    #start_video()  # camera

    filename = '../moscow_courses/blue_moscow.jpg'
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    img = cv2.imread(filename)
    assert img is not None, 'file not found'
    start_file(filename)
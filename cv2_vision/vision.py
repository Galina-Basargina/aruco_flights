import cv2
from cv2 import Mat


def extract_image(source_mat: Mat, x, y, width, height):
    # обрезка изображения
    extracted = source_mat[x:width + x, y:height + y]
    return extracted


def resize_image(source_mat: Mat, width, height):
    # изменение размера изображения
    resize = cv2.resize(source_mat, (width, height))
    return resize


def change_color_space(source_mat: Mat, color):
    # Изменение цветного простанства с BGR на HSV
    changed_image = cv2.cvtColor(source_mat, color)
    return changed_image


def blur_image(source_mat: Mat, x, y):
    # Создание размытия изображения
    blur = cv2.blur(source_mat, (x, y))
    return blur


def bright_image(source_mat: Mat, contrast, bright):
    bright_contrast = cv2.convertScaleAbs(source_mat, beta=bright, alpha=contrast)
    return bright_contrast



def binary_image(source_mat: Mat, min_v: tuple, max_v: tuple):
    # Переворматирование в изображение  с двоичной матрицей
    binary_images = cv2.inRange(source_mat, min_v, max_v)
    return binary_images


def trackbar_changed(value):
    pass


def set_threshold_values(capture):
    success, source = capture.read()

    cv2.imshow('binary', source)
    cv2.createTrackbar('Bmin', 'binary', 0, 255, trackbar_changed)
    cv2.createTrackbar('Gmin', 'binary', 0, 255, trackbar_changed)
    cv2.createTrackbar('Rmin', 'binary', 0, 255, trackbar_changed)
    cv2.createTrackbar('Bmax', 'binary', 0, 255, trackbar_changed)
    cv2.createTrackbar('Gmax', 'binary', 0, 255, trackbar_changed)
    cv2.createTrackbar('Rmax', 'binary', 0, 255, trackbar_changed)
    while True:
        success, source = capture.read()

        Bmin = cv2.getTrackbarPos('Bmin', 'binary')
        Rmin = cv2.getTrackbarPos('Rmin', 'binary')
        Bmax = cv2.getTrackbarPos('Bmax', 'binary')
        Gmin = cv2.getTrackbarPos('Gmin', 'binary')
        Gmax = cv2.getTrackbarPos('Gmax', 'binary')
        Rmax = cv2.getTrackbarPos('Rmax', 'binary')

        binary = binary_image(source, (Bmin, Gmin, Rmin),  (Bmax, Gmax, Rmax))
        mask = cv2.bitwise_and(source, source, mask=binary)

        cv2.imshow('mask', mask)
        cv2.imshow('source', source)
        cv2.imshow('binary', binary)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def set_bright_conrtast_image(capture):
    success, source = capture.read()

    cv2.imshow('bright_contrast_image', source)
    cv2.createTrackbar('bright', 'bright_contrast_image', 0, 255, trackbar_changed)
    cv2.createTrackbar('contrast', 'bright_contrast_image', 0, 255, trackbar_changed)

    while True:
        success, source = capture.read()

        bright = cv2.getTrackbarPos('bright', 'bright_contrast_image')
        contrast = cv2.getTrackbarPos('contrast', 'bright_contrast_image')


        bright_contrast = bright_image(source, bright, contrast)

        cv2.imshow('source', source)
        cv2.imshow('bright_contrast_image', bright_contrast)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

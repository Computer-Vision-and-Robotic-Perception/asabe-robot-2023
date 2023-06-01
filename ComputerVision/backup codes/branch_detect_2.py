import cv2
import numpy as np
import imutils


def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)


 #Branch Color Range
low_brown = np.array([60, 10, 20])
high_brown = np.array([90, 80, 60])

low_b = np.array([1, 30, 40])
high_b = np.array([20, 140, 80])

low_black = np.array([150,20,30])
high_black = np.array([170,90,100])


# resize the frame, blur it, and convert it to the HSV
# color space

frame = cv2.imread('Original_0.png')

frame = imutils.resize(frame, width=600)

orig = frame
gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(orig, cv2.COLOR_BGR2HSV)

def mouse_click(event, x, y, flags, para):
    if event == cv2.EVENT_LBUTTONDOWN:  # 左边鼠标点击
        print('PIX:', x, y)
        print("BGR:", orig[y, x])
        print("GRAY:", gray[y, x])
        print("HSV:", hsv[y, x])
        print(" ")

cv2.imshow("original",orig)
cv2.setMouseCallback("original", mouse_click)

# frame = cv2.GaussianBlur(frame, (11, 11), 0)
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


blackMask = cv2.inRange(hsv, low_black, high_black)
brownMask = cv2.inRange(hsv, low_brown, high_brown)
bMask = cv2.inRange(hsv, low_b, high_b)


finalMask = brownMask | blackMask | bMask
result=finalMask

gray = finalMask

# blur = cv2.GaussianBlur(gray, (0,0), sigmaX=33, sigmaY=33)
divide = cv2.divide(gray, gray, scale=255)
thresh = cv2.threshold(divide, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
result = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)


cv2.imshow("mask", divide)
cv2.waitKey(0)

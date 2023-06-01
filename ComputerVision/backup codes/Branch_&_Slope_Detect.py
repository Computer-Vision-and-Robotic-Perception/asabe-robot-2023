# -*- coding: utf-8 -*-
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
#low_black = np.array([30,30,42])
#high_black = np.array([40,40,54])

# resize the frame, blur it, and convert it to the HSV
# color space

frame = cv2.imread('O5.png')

frame = imutils.resize(frame, width=600)

orig = frame
gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(orig, cv2.COLOR_BGR2HSV)

# def mouse_click(event, x, y, flags, para):
#     if event == cv2.EVENT_LBUTTONDOWN:  # 左边鼠标点击
#         print('PIX:', x, y)
#         print("BGR:", orig[y, x])
#         print("GRAY:", gray[y, x])
#         print("HSV:", hsv[y, x])
#         print(" ")

# cv2.imshow("original",orig)
# cv2.setMouseCallback("original", mouse_click)

# frame = cv2.GaussianBlur(frame, (11, 11), 0)
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


blackMask = cv2.inRange(hsv, low_black, high_black)
brownMask = cv2.inRange(hsv, low_brown, high_brown)
bMask = cv2.inRange(hsv, low_b, high_b)


finalMask = brownMask | blackMask | bMask
result = finalMask

gray = finalMask

# blur = cv2.GaussianBlur(gray, (0,0), sigmaX=33, sigmaY=33)
divide = cv2.divide(gray, gray, scale=255)
thresh = cv2.threshold(divide, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
result = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)


# LINE DETECTION -------------------------------------------------------------

low_threshold = 50
high_threshold = 150
edges = cv2.Canny(gray, low_threshold, high_threshold)

rho = 1  # distance resolution in pixels of the Hough grid
theta = np.pi / 180  # angular resolution in radians of the Hough grid
threshold = 20  # minimum number of votes (intersections in Hough grid cell)
min_line_length = 50  # minimum number of pixels making up a line
max_line_gap = 20  # maximum gap in pixels between connectable line segments
line_image = np.copy(result) * 0  # creating a blank to draw lines on

# Run Hough on edge detected image
# Output "lines" is an array containing endpoints of detected line segments
lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                    min_line_length, max_line_gap)

for line in lines:
    for x1,y1,x2,y2 in line:
        cv2.line(orig,(x1,y1),(x2,y2),(255,0,0),2)
        
        # PRINTING SLOPE OF EACH LINE
        
        slope = round((y2 - y1) / (x2 - x1), 2)
        # Using cv2.putText()
        new_image = cv2.putText(
          img = orig,
          text = str(slope * -1),
          org = (x1, y1),
          fontFace = cv2.FONT_ITALIC,
          fontScale = 0.5,
          color = (125, 246, 55),
          thickness = 1
        )

cv2.imshow("mask", orig)
cv2.waitKey(0)
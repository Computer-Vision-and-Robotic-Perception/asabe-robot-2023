# -*- coding: utf-8 -*-
"""
Created on Mon Jun 27 17:25:08 2022
@author: brila
"""

import cv2
import numpy as np
from numpy.linalg import norm
import imutils
import math
import random
from matplotlib import pyplot as plt

# The end of the script results in showing a labeled image, and an unused array
# called found_balls containing each (x,y) coordinate of a ball belonging to
# the center plant. If this script is going to be turned into a function, you
# can input an image, and simply return the found_balls array at the end of
# this script.

# ------------------------------ Image Input ------------------------------

# Reading image in and creating a copy
frame = cv2.imread('Original_12.png')
frame = imutils.resize(frame, width=600)
orig = frame

(h, w) = frame.shape[:2]

# ---------------------------- CODE PARAMETERS ----------------------------
# -- Edit these parameters to affect how the code will handle detection. --

crop_image = 0  # 1 or 0, do or dont crop the image to focus on the center

exclusion_distance = 200  # If a matched line is farther than this away from a
# ball dont include it, its likely a bad detection.

ball_distance = 100  # The distance from the center pole a ball is allowed to
# be and still be able to be assigned to the pole

circle_mask_size = 45  # How large of a radius around each found circle should
# be included in the line detection range.

# --------------------------- Mask Color Ranges ---------------------------

# Branch Color Range
low_brown = np.array([60, 10, 20])
high_brown = np.array([90, 80, 60])

low_b = np.array([1, 30, 40])
high_b = np.array([20, 140, 80])

low_black = np.array([150, 20, 30])
high_black = np.array([170, 90, 100])

# Pingpong Ball Color Range
greenLower = (50, 86, 40)
greenUpper = (100, 255, 255)

# --------------------------------------------------------------------------


# ----------------------------------- CODE -----------------------------------

# Cropping image to only focus on middle pole
if crop_image == 1:
    frame = frame[0: h, w // 4: w // 2 + w // 4]

orig = frame

# BALL DETECTION AND DRAWING -------------------------------------------------

# resize the frame, blur it, and convert it to the HSV
# color space

blurred = cv2.GaussianBlur(frame, (11, 11), 0)
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
# construct a mask for the color "green", then perform
# a series of dilations and erosions to remove any small
# blobs left in the mask
GreenMask = cv2.inRange(hsv, greenLower, greenUpper)
GreenMask = cv2.erode(GreenMask, None, iterations=2)
GreenMask = cv2.dilate(GreenMask, None, iterations=2)

cv2.imwrite('green mask.png',GreenMask)


img = cv2.imread('green mask.png', 0)
rows, cols = img.shape
img_p = img.copy()
img_C = img.copy()

# ----------------------- Circle Detection Parameters -----------------------
low_threshold = 50
high_threshold = 150
rho = 1  # distance resolution in pixels of the Hough grid
theta = np.pi / 90  # 180  # angular resolution in radians of the Hough grid
threshold = 15  # minimum number of votes (intersections in Hough grid cell)
min_line_length = 30  # minimum number of pixels making up a line
max_line_gap = 15  # maximum gap in pixels between connectable line segments

edges = cv2.Canny(img, 50, 170, apertureSize=3)
#
accuracy = 2
#
threshold = 200


_, thresh = cv2.threshold(img, 150, 255, cv2.THRESH_TOZERO)

dp = 2

minDist = 20

circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, dp, minDist, param1=70, param2=30, minRadius=0, maxRadius=50)

circles = np.uint16(np.around(circles))

for i in circles[0, :]:

    cv2.circle(img_C, (i[0], i[1]), i[2], (0, 255, 0), 2)

    cv2.circle(img_C, (i[0], i[1]), 2, (0, 0, 255), 2)

#cv2.imshow('img', img)
cv2.imshow("img-p", img_p)
cv2.imshow("img_C", img_C)
cv2.waitKey(0)
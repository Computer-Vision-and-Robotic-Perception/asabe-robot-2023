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

# Pingpong Ball Color Range
greenLower = (50, 86, 40)
greenUpper = (100, 255, 255)

# Reading image in and creating a copy
frame = cv2.imread('O5.png')
frame = imutils.resize(frame, width=600)
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

# find contours in the mask and initialize the current
# (x, y) center of the ball
cnts = cv2.findContours(GreenMask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)
center = None
# only proceed if at least one contour was found
if len(cnts) > 0:
    
    circlemask = np.zeros_like(frame)
    
    for c in cnts:
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            # Drawing to Mask
            cv2.circle(circlemask, (int(x), int(y)), int(radius + 45), (100, 255, 255), -1)

# Applying Circle Mask
greencirclemask = cv2.inRange(circlemask, greenLower, greenUpper)
greenresult = cv2.bitwise_and(frame, frame, mask = greencirclemask)

# ----------------------------------------------------------------------------

# BRANCH DETECTION -----------------------------------------------------------

# Branch Color Range
low_brown = np.array([60, 10, 20])
high_brown = np.array([90, 80, 60])

low_b = np.array([1, 30, 40])
high_b = np.array([20, 140, 80])

low_black = np.array([150,20,30])
high_black = np.array([170,90,100])


gray = cv2.cvtColor(greenresult, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(greenresult, cv2.COLOR_BGR2HSV)

hsv = cv2.cvtColor(greenresult, cv2.COLOR_BGR2HSV)


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

# ----------------------------------------------------------------------------

# LINE DETECTION ---------------------------------------------------

low_threshold = 50
high_threshold = 150
edges = cv2.Canny(gray, low_threshold, high_threshold)

rho = 1  # distance resolution in pixels of the Hough grid
theta = np.pi / 90 # 180  # angular resolution in radians of the Hough grid
threshold = 15  # minimum number of votes (intersections in Hough grid cell)
min_line_length = 30  # minimum number of pixels making up a line
max_line_gap = 15  # maximum gap in pixels between connectable line segments
line_image = np.copy(result) * 0  # creating a blank to draw lines on

# Run Hough on edge detected image
# Output "lines" is an array containing endpoints of detected line segments
lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                    min_line_length, max_line_gap)

        
# ----------------------------------------------------------------------------
# POLE ASSIGNMENT AND SLOPE DETECTION ----------------------------------------
if len(cnts) > 0:
    for c in cnts:
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            
            dist = 50000 # an impossibly high starting distance
            # Empty Saved line coordinates
            xs1 = 0
            ys1 = 0
            xs2 = 0
            ys2 = 0 
            
            # Finding the closest line to this iteration's center
            for line in lines:
                for x1,y1,x2,y2 in line:
                    # Calculating distance from current center to current line
                    testpoint = [(x1 + x2) / 2, (y1 + y2) / 2]
                    testdist = math.dist(center, testpoint)
                    
                    # If a closer line was found update our "closest" line for this center
                    if testdist < dist:
                        dist = testdist
                        xs1 = x1
                        ys1 = y1
                        xs2 = x2
                        ys2 = y2
            
            # Drawing things on the orignal image based on found lines/centers etc
            
            # Getting a new random color for this match
            randcolor = (random.randint(100,255), random.randint(100,255), random.randint(100,255))
            
            # Drawing this matching center/line on original image
            cv2.circle(orig, (int(x), int(y)), int(radius), randcolor, 2)
            cv2.circle(orig, center, 3, randcolor, -1)
            cv2.line(orig,(xs1,ys1),(xs2,ys2),randcolor,2)
            
            # PRINTING SLOPE OF EACH LINE
            slope = round((ys2 - ys1) / (xs2 - xs1), 2)
            # Using cv2.putText()
            new_image = cv2.putText(
              img = orig,
              text = str(slope * -1), #str(round(dist,2)),
              org = (xs1, ys1),
              fontFace = cv2.FONT_ITALIC,
              fontScale = 0.5,
              color = randcolor,
              thickness = 2
            )
    

cv2.imshow("mask", orig)
cv2.waitKey(0)
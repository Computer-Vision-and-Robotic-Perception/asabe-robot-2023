# -*- coding: utf-8 -*-
"""
Created on Wed Jun  8 00:13:01 2022

@author: brila
"""

import os
import glob
import math
import cv2 as cv
import numpy as np

def ResizeWithAspectRatio(image, width=None, height=None, inter=cv.INTER_AREA):
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

    return cv.resize(image, dim, interpolation=inter)

# Reading image
image = cv.imread('testim1.jpg')

hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

# Brown threshold
# low_brown = np.array([6,63,0])
# high_brown = np.array([23,255,81])
#low_brown = np.array([6,105,20])
#high_brown = np.array([90,155,55])
low_brown = np.array([3,70,20])
high_brown = np.array([20,180,80])

# Green threshold
low_green = np.array([50,86,40])
high_green = np.array([100,255,255])

# Black threshold (the board pegs)
low_black = np.array([150,5,55])
high_black = np.array([180,40,100])

# Creating masks
brownMask = cv.inRange(hsv, low_brown, high_brown)
greenMask = cv.inRange(hsv, low_green, high_green)
blackMask = cv.inRange(hsv, low_black, high_black)

# Combining masks
finalMask = brownMask | greenMask | blackMask

# Applying masks to image
result = cv.bitwise_and(image, image, mask = finalMask)

# Saving mask portion of the process
cv.imwrite('mask.jpg', result)

# Morphing objects together
kernel = cv.getStructuringElement(cv.MORPH_RECT, (25,25))
result = cv.morphologyEx(result, cv.MORPH_CLOSE, kernel)

# Saving morph portion of the process
cv.imwrite('morph.jpg', result)

# Thesholding and getting rid of noise
gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
blur = cv.GaussianBlur(gray, (0,0), sigmaX=33, sigmaY=33)
divide = cv.divide(gray, blur, scale=255)
thresh = cv.threshold(divide, 0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)[1]
kernel = cv.getStructuringElement(cv.MORPH_RECT, (3,3))
result = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel)

# Finding and separating contours
contours1, hierarchy1 = cv.findContours(image=result, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_NONE)
cv.drawContours(image, contours=contours1, contourIdx=-1, color=(0, 255, 0), thickness=5, lineType=cv.LINE_AA)

# Saving output of the process
cv.imwrite('output.jpg', image)

#image = cv.bitwise_and(image, image, mask = finalMask)

#cv.namedWindow("Image", cv.WINDOW_NORMAL)
resize = ResizeWithAspectRatio(image, width=1280)
cv.imshow('result', resize)
cv.waitKey(0)
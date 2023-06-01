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

# Pingpong Ball Color Range
greenLower = (50, 86, 40)
greenUpper = (100, 255, 255)

# Branch Color Range
brownLower = np.array([0,0,60])
brownUpper = np.array([90,30,100])



# resize the frame, blur it, and convert it to the HSV
# color space

frame = cv2.imread('O5.png')

frame = imutils.resize(frame, width=600)

orig = frame

blurred = cv2.GaussianBlur(frame, (11, 11), 0)
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
# construct a mask for the color "green", then perform
# a series of dilations and erosions to remove any small
# blobs left in the mask
GreenMask = cv2.inRange(hsv, greenLower, greenUpper)
GreenMask = cv2.erode(GreenMask, None, iterations=2)
GreenMask = cv2.dilate(GreenMask, None, iterations=2)

#BrownMask = cv2.inRange(hsv, brownLower, brownUpper)
#BrownMask = cv2.dilate(BrownMask, None, iterations=4)

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
            cv2.circle(circlemask, (int(x), int(y)), int(radius + 35), (100, 255, 255), -1)
            
            # Drawing on Original
            cv2.circle(orig, (int(x), int(y)), int(radius), (100, 255, 255), 2)
            cv2.circle(orig, center, 3, (0, 0, 255), -1)

# Applying Circle Mask
greencirclemask = cv2.inRange(circlemask, greenLower, greenUpper)
greenresult = cv2.bitwise_and(frame, frame, mask = greencirclemask)
#brown = cv2.bitwise_and(orig, orig, mask = BrownMask)

#resultmask = BrownMask | GreenMask | greencirclemask

cv2.imshow("RESULT", greenresult)
cv2.waitKey(0)
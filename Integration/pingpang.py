# -*- coding: utf-8 -*-
'''
Create by Haodong Li
2022.7

Mapping unopened boll
1. detect the interested unopened boll by a green mask======>>>center points, radius
2.detect the branch around, draw the Houghlines of all pic
3. draw a circle buffer and collect all lines geting from last step, assign a line to a center point
4.assign the line to pole or ignore, assign the center point to the
 '''

import cv2
import numpy as np
from numpy.linalg import norm
import imutils
import math
import random

# from skimage.filters import threshold_otsu
# from skimage.morphology import binary_opening, binary_closing, binary_erosion, binary_dilation
import matplotlib.pyplot as plt


def pole_assign(frame):

    # ------------------------------ Image Input ------------------------------

    frame = imutils.resize(frame, height=480, width=640)
    orig = frame

    (h, w) = frame.shape[:2]

    print(frame.shape[:2])

    # ---------------------------- CODE PARAMETERS ----------------------------
    # -- Edit these parameters to affect how the code will handle detection. --

    method = 0  # method 2 = straight distance for maching, method 1 = center of line for matching, method = 0 ends of lines for matching

    crop_image = 0  # 1 or 0, do or dont crop the image to focus on the center

    show_all_lines = 1  # Also show the unused lines for each ball

    show_mask = 1  # Show the line mask before calculations are made

    pick_sensitivity = 15  # The Ball will not be matched with lines closer than its
    # radius plus this value in pixels

    exclusion_distance = 90  # If a matched line is farther than this away from a
    # ball dont include it, its likely a bad detection.

    ball_distance = 150  # 150 # The distance from the center pole a ball is allowed to
    # be and still be able to be assigned to the pole

    circle_mask_size = 60  # How large of a radius around each found circle should
    # be included in the line detection range.

    slope_sensitivity = 0.01  # 0 +- this slope range will not be used for matching

    lm = 5  # Number of pixels +- the center posiiton of the ball that will be considered
    # When matching a ball to one of the four cases

    # --------------------------- Mask Color Ranges ---------------------------

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

    blurred = cv2.GaussianBlur(frame, (45, 45), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    GreenMask = cv2.inRange(hsv, greenLower, greenUpper)
    GreenMask = cv2.erode(GreenMask, None, iterations=2)
    GreenMask = cv2.dilate(GreenMask, None, iterations=2)
    cv2.imshow('Greenmask',GreenMask)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(GreenMask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    center = None

    # only proceed if at least one contour was found
    low_threshold = 50
    high_threshold = 150
    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 90  # 180  # angular resolution in radians of the Hough grid
    threshold = 15  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 30  # minimum number of pixels making up a line
    max_line_gap = 15  # maximum gap in pixels between connectable line segments
    dp = 2
    minDist = 20

    (h, w) = frame.shape[:2]
    print(frame.shape[:2])

    if len(cnts) > 0:
        circlemask = np.zeros_like(frame)

        for c in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            x=int(x)
            y=int(y)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size

            #if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            # Drawing to Mask

            gap = abs(float(cv2.contourArea(c)) - 3.141 * radius * radius)
            #print('gap=',gap)
            if gap > 1000:
                # ### detect the arc and find the center point
                # continue
                #print('detedt a arc')
                mask = np.ones(frame.shape[:2], dtype="uint8") * 0
                # Draw the contours on the mask
                cv2.drawContours(mask, c, -1, 255, -1)
                # remove the contours from the image and show the resulting images
                img = cv2.bitwise_and(mask, mask, mask=mask)

                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
                closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel, iterations=2)
                # cv2.imshow('close', closing)
                # cv2.waitKey(0)

                circles = cv2.HoughCircles(closing, cv2.HOUGH_GRADIENT, dp, minDist, param1=70, param2=30, minRadius=0, maxRadius=40)
                #print('circles = ', circles)
                if circles is None:
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                else:
                    circles = np.uint16(np.around(circles))
                    if circles.shape[1] > 1:
                        p = circles[0, :, 0]
                        list = p.tolist()
                        for i in range(p.size):
                            list[i] = abs(list[i] - w // 2)
                        max_index = list.index(max(list))
                        center=(x, y)
                        (x, y) = (circles[0, max_index - 1, 0], circles[0, max_index - 1, 1])
                        radius = circles[0,max_index - 1,2]

                    else:
                        ((x, y), radius) = cv2.minEnclosingCircle(c)
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            #print('centerpoint=',x, y,'radius=',radius)

            cv2.circle(circlemask, (int(x), int(y)),
                       int(radius + circle_mask_size), (100, 255, 255), -1)

    # Applying Circle Mask
    greencirclemask = cv2.inRange(circlemask, greenLower, greenUpper)
    greenresult = cv2.bitwise_and(frame, frame, mask=greencirclemask)
    cv2.imshow('greenbuffer',greenresult)
    # BRANCH DETECTION -----------------------------------------------------------


    frame2 = cv2.GaussianBlur(frame, (0, 0), sigmaX=1.2, sigmaY=1.2)
    G = frame2[:, :, 1]
    tG = 60
    binary = (G > tG).astype(float)
    # binary = binary_opening(binary, np.ones((9, 9)))
    # binary = binary_closing(binary, np.ones((2, 2)))
    binary = binary.astype(int)
    empty = np.zeros(orig.shape)

    h, w = binary.shape
    counter = 0
    for i in range(0, h):
        for j in range(0, w):
            if binary[i, j] == 1:
                empty[i, j, 0] = orig[i, j, 0]
                empty[i, j, 1] = orig[i, j, 1]
                empty[i, j, 2] = orig[i, j, 2]

            else:
                empty[i, j, 0] = 0
                empty[i, j, 1] = 0
                empty[i, j, 2] = 0

    branchmask = cv2.inRange(empty, 0, 255)
    branchresult = cv2.bitwise_and(frame, frame, mask=branchmask)

    #cv2.imshow("Branch Edge", branchresult)
    # ----------------------------------------------------------------------------
    cv2.waitKey(0)
    # LINE DETECTION -------------------------------------------------------------
    # ----------------------- Line Detection Parameters -----------------------
    low_threshold = 50
    high_threshold = 150
    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # 90 # 180  # angular resolution in radians of the Hough grid
    threshold = 25  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 15  # minimum number of pixels making up a line
    max_line_gap = 3  # maximum gap in pixels between connectable line segments

    edges = cv2.Canny(branchresult.astype(np.uint8), low_threshold, high_threshold)
    # line_image = np.copy(result) * 0  # creating a blank to draw lines on

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                            min_line_length, max_line_gap)

    try:
        if show_all_lines == 1:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(orig, (x1, y1), (x2, y2), (255, 0, 0), 2)

        if show_mask == 1:
            cv2.imshow("mask", edges)  # binary.astype(np.float))
            cv2.waitKey(0)

            cv2.imshow("mask", empty)  # binary.astype(np.float))
            cv2.waitKey(0)
    except:
        print("Could not find any lines!")
        return
    # ----------------------------------------------------------------------------

    # POLE ASSIGNMENT AND SLOPE DETECTION ----------------------------------------
    # print(frame.shape[:2])
    (h, w) = frame.shape[:2]

    found_balls = []

    if len(cnts) > 0:
        for c in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            #if radius > 10:
            gap = abs(float(cv2.contourArea(c)) - 3.141 * radius * radius)
            print(gap)
            if gap > 1000:
                # ### detect the arc and find the center point
                # continue
                print('detedt a arc')
                mask = np.ones(frame.shape[:2], dtype="uint8") * 0
                # Draw the contours on the mask
                cv2.drawContours(mask, c, -1, 255, -1)
                # remove the contours from the image and show the resulting images
                img = cv2.bitwise_and(mask, mask, mask=mask)

                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
                closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel, iterations=2)
                circles = cv2.HoughCircles(closing, cv2.HOUGH_GRADIENT, dp, minDist, param1=70, param2=30, minRadius=0,  maxRadius=40)
                #circles = np.uint16(np.around(circles))
                print('circles = ', circles)
                if circles is None:
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                else:
                    circles = np.uint16(np.around(circles))
                    if circles.shape[1] > 1:
                        p = circles[0, :, 0]
                        list = p.tolist()
                        for i in range(p.size):
                            list[i] = abs(list[i] - w // 2)
                        max_index = list.index(max(list))
                        center = (x, y)
                        (x, y) = (circles[0, max_index - 1, 0], circles[0, max_index - 1, 1])
                        radius = circles[0, max_index - 1, 2]

                    else:
                        ((x, y), radius) = cv2.minEnclosingCircle(c)
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            print('centerpoint=',x, y,'radius=',radius)
            #return the close center point
            dist = 50000  # an impossibly high starting distance
            # Empty Saved line coordinates
            xs1 = 0
            ys1 = 0
            xs2 = 0
            ys2 = 0

            # Finding the closest line to this iteration's center
            for line in lines:
                for x1, y1, x2, y2 in line:
                    testdist = 0

                    if method == 1:
                        # Calculating distance from current center to current line
                        testpoint = [(x1 + x2) / 2, (y1 + y2) / 2]
                        testdist = math.dist(center, testpoint)

                    elif method ==2:
                        # straight distance from current line to current line
                        line_point1 = np.array([x1, y1])
                        line_point2 = np.array([x2, y2])
                        vec1 = line_point1 - center
                        vec2 = line_point2 - center
                        testdist = np.abs(np.cross(vec1, vec2)) / np.linalg.norm(line_point1 - line_point2)


                    else:
                        # Using the closest point of the line to the center of the ball instead
                        dist1 = math.dist(center, [x1, y1])
                        dist2 = math.dist(center, [x2, y2])

                        testdist = min(dist1, dist2)

                    test_slope = (y2 - y1) / (x2 - x1)

                    # Checking for slope sensitivty to NOT use this line if its outside of these parameters
                    if (test_slope > -1 * slope_sensitivity) and (test_slope < slope_sensitivity):
                        continue

                    # If a closer line was found update our "closest" line for
                    # this center
                    if testdist < dist and testdist > radius + pick_sensitivity:
                        dist = testdist
                        xs1 = x1
                        ys1 = y1
                        xs2 = x2
                        ys2 = y2

            # Checking to see if found line is reasonably close enough
            if dist > exclusion_distance:
                continue

            # Cheking to see if current ball is reasonably close enough
            if center[0] > w // 2 + ball_distance or center[0] < w // 2 - ball_distance:
                continue

            # Drawing things on the orignal image based on found lines/centers

            # Slope Calculation
            slope = round((ys2 - ys1) / (xs2 - xs1), 2)

            # Getting a new random color for this match
            # if center[1] < (ys1 + ys2) / 2 and (slope > 0 and center[0] < w//2) or (slope < 0 and center[0] > w//2):
            if (slope > 0 and center[0] > w // 2 and center[0] > xs1 - lm) or (
                    slope < 0 and center[0] > w // 2 and center[0] > xs1 - lm) or (
                    slope < 0 and center[0] < w // 2 and center[0] < xs1 + lm) or (
                    slope > 0 and center[0] < w // 2 and center[0] < xs1 + lm):
                # If these slopes, then this ball belongs to the center pole
                randcolor = (0, 0, 255)
                # Adding to list of centers beloning to the center pole
                found_balls.append(center)
            else:
                randcolor = (random.randint(100, 255),
                             random.randint(100, 255),
                             random.randint(100, 255))

            # Drawing this matching center/line on original image
            cv2.circle(orig, (int(x), int(y)), int(radius), randcolor, 2)
            cv2.circle(orig, center, 3, randcolor, -1)
            cv2.line(orig, (xs1, ys1), (xs2, ys2), randcolor, 2)

            # PRINTING SLOPE OF EACH LINE
            # Using cv2.putText()
            new_image = cv2.putText(
                img=orig,
                text=str(slope * -1),  # str(round(dist,2)),
                org=(xs1, ys1),
                fontFace=cv2.FONT_ITALIC,
                fontScale=0.5,
                color=randcolor,
                thickness=2
            )

    # Drawing Center Circle (Or Line)
    # cv2.circle(orig, (w//2, h//2), 5, (0,0,255), -1)
    cv2.line(orig, (w // 2, h), (w // 2, 0), (0, 0, 255), 1)

    print("Ball coordinates belonging to center pole: ")
    print(found_balls)
    print("Balls displayed in red belong to the center pole")

    cv2.imshow("mask", orig)
    cv2.waitKey(0)

    return found_balls


frame = cv2.imread('img.jpg')
#frame = cv2.imread('Original_11.png')
BALLZ = pole_assign(frame)


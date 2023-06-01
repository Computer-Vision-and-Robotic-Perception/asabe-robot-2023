import imutils
#import math
import random
import os
import cv2
import numpy as np
from numpy.linalg import norm
import imutils
#import math

# The end of the script results in showing a labeled image, and an unused array
# called found_balls containing each (x,y) coordinate of a ball belonging to
# the center plant. If this script is going to be turned into a function, you
# can input an image, and simply return the found_balls array at the end of
# this script.

# ------------------------------ Image Input ------------------------------

# Reading image in and creating a copy
#for img in os.listdir('./original'):
frame = cv2.imread('Original_293.png')
#frame = imutils.resize(frame, width=600)
orig = frame

(h, w, c) = np.shape(frame)

print(h, w, c)

import cv2
import imutils
from skimage.filters import threshold_otsu
from skimage.morphology import binary_opening, binary_closing, binary_erosion, binary_dilation
import numpy as np
import matplotlib.pyplot as plt

def main():
    frame = cv2.imread('Original_15.png')
    frame = imutils.resize(frame, width=600)
    frame = cv2.GaussianBlur(frame, (0, 0), sigmaX=1.2, sigmaY=1.2)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow('h', hsv[:, :, 0])
    # cv2.imshow('s', hsv[:, :, 1])
    # cv2.imshow('v', hsv[:, :, 2])
    # cv2.waitKey()
    B ,G, R = frame[:, :, 0], frame[:, :, 1], frame[:, :, 2]
    tR = threshold_otsu(R) - 20
    tG = threshold_otsu(G) - 20
    tB = threshold_otsu(B) - 20
    tg = threshold_otsu(gray) - 20
    tv = threshold_otsu(hsv[:, :, 2]) - 20
    ts = threshold_otsu(hsv[:, :, 1]) - 20
    binary = (R > tR).astype(float) * (G > tG).astype(float) * (B > tB).astype(float) * (gray > tg).astype(float) *\
                           (hsv[:, :, 2] > tv).astype(float) + (hsv[:, :, 1] > ts).astype(float)

    # plt.hist(R)
    # plt.vlines(tR + 30, 0, 1000)
    # plt.show()

    # binary = binary_dilation(binary, np.ones((2, 2)))
    # binary = binary_closing(binary, np.ones((5, 5)))
    # binary = cv2.medianBlur(binary.astype(np.uint8), 3)
    binary = binary_opening(binary, np.ones((9, 9)))
    binary = binary_closing(binary, np.ones((2, 2)))

    cv2.imshow("Original", frame)
    cv2.imshow('Binary', binary.astype(float))
    cv2.waitKey(0)

if __name__ == '__main__':
    main()

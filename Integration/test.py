import cv2
import time
import numpy as np

def test_rotation_time():
    times_np = []
    times_cv = []

    imp = cv2.imread('test.jpg')

    for i in range(100):
        t1 = time.time()
        cv_imp = cv2.rotate(imp, cv2.ROTATE_180)
        t2 = time.time()
        np_imp = np.flipud(np.fliplr(imp))
        t3 = time.time()
        
        times_cv.append(t2 - t1)
        times_np.append(t3 - t2)
           
        cv2.imshow('Original', imp)
        cv2.imshow('OpenCV', cv_imp)
        cv2.imshow('Numpy', np_imp)
        cv2.waitKey(100)

    print('Times Open CV = %f' % (np.array(times_cv).mean()))
    print('Times Numpy R = %f' % (np.array(times_np).mean()))

if __name__ == '__main__':
    test_rotation_time()
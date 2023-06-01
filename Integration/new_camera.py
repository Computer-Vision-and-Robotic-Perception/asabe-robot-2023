import cv2
import time
import picamera
import numpy as np

class Camera(picamera.PiCamera):
    def __init__(self):
        super(Camera, self).__init__()
        self.resolution = (320, 240)
        self.framerate = 24
#         self.start_preview()
        time.sleep(2)
        self.out = np.empty((240, 320, 3), dtype=np.uint8)
        self.capture(self.out, 'rgb')
        
    def get_frame(self):
        self.capture(self.out, 'rgb')
        return self.out
        
    
if __name__ == '__main__':
    cam = Camera()
    while True:
        frame = cam.get_frame()
        cv2.imshow('Cam', frame)
        key = cv2.waitKey(1)
#         if key = ''



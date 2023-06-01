'''
hole_pole.py
Hole-Pole detection script

Byron Hernandez
University of Florida
July 2022
'''
from camera_utils.arducam.custom.holes_camera import Camera
import matplotlib.pyplot as plt
from comm_utils import Server
from config import *
import numpy as np
import time
import cv2

VERSION = python_version().split('.')[0]


class HolePoleService():
    def __init__(self):
        self.camera = Camera()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        fo, co = 1160, 800
        #fo, co = int(camera.fmt['height']), int(camera.fmt['width'])
        self.fo0, self.fo1, self.co0, self.co1 =  fo*2//9, fo*5//9, co*4//9, co*5//9
        cr = self.co1 - self.co0 - 1
        self.cr0, self.cr1 = cr*3//9, cr*6//9
        self.hole, self.pole, self.empty, self.new = 0, 0, 0, 1
        self.hole_flag, self.pole_flag = False, False

    def __call__(self):
        raw = self.camera.get_frame()
        raw = raw[self.fo0:self.fo1, self.co0:self.co1]
        gray = cv2.cvtColor(raw, cv2.COLOR_BAYER_RG2GRAY)
        bw = (cv2.cvtColor(gray, cv2.COLOR_BAYER_RG2GRAY) < 60).astype(np.uint8)
        bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, self.kernel)*255
        ccom = cv2.connectedComponentsWithStats(bw, 4, cv2.CV_32S)
        if ccom[0] > 1 and (ccom[2][1:, -1] > 100).any():
            arg = np.argmax(ccom[2][1:, -1]) + 1
            if self.new and self.cr0 < ccom[3][arg, 0] < self.cr1:
                self.hole += 1
                self.hole_flag = True
                if ccom[2][arg, -1] > 5000:
                    self.pole += 1
                    self.pole_flag = True
                self.empty, self.new = 0, 0
                # print('Hole', self.hole, 'Pole', self.pole)
        else:
            self.empty += 1
            if self.empty > 3:
                self.new = 1
        for j in range(ccom[0]):
            bw = cv2.putText(bw, str(j), (int(ccom[3][j,0]), int(ccom[3][j,1])), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 127, 2)
        cv2.imshow('Result', bw)
        cv2.waitKey(1)
        return self.hole, self.pole
        
    def end(self):
        self.camera.close()
        cv2.destroyAllWindows()


class HolePoleDummyService():
    def __init__(self):
        self.t = time.time()
        self.t0 = self.t
    
    def __call__(self):
        if time.time() - self.t > 3:
            self.t = time.time()
            message = 'holes dummy answer %f' % (self.t - self.t0)
        else:
            message = False
        return message
    
    def end(self):
        pass


def main():
    task = HolePoleService()
    server_h = Server(HOST_HPo, PORT_HPo[1], 'Hole-Pole', task)
    server_h.pair()
    while True:
        out = server_h.step()
        inp = server_h.read_line()
##        if out:
##            print('Sending: %s' % out[:-1].decode())
##            server_h.write(out)
##        if inp:
##            print('Received %s' % inp.decode())
        if task.hole_flag:
            task.hole_flag = False
            print('Found Hole %d' % (task.hole))
            server_h.write(b'Hole found' + out)
        if task.pole_flag:
            task.pole_flag = False
            print('Found Pole %d' % (task.pole))
            server_h.write(b'Pole found' + out)

# Safe Guard
if __name__ == '__main__':
    main()

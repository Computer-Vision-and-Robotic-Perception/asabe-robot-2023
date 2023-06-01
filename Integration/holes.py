'''
hole_pole.py
Hole-Pole detection script

Byron Hernandez
University of Florida
July 2022
'''
from new_camera import Camera
# import matplotlib.pyplot as plt
from comm_utils import Server
from config import *
import numpy as np
import picamera
import time
import cv2


class HolePoleService():
    def __init__(self):
        self.camera = Camera()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        self.hole, self.pole, self.empty, self.new = 0, 0, 0, 1
        self.hole_flag, self.pole_flag = False, False

    def __call__(self):
        rgb = self.camera.get_frame()
        rgb = rgb[:, 100:220, :]
        bw = (cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY) < 60).astype(np.uint8)
        bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, self.kernel)*255
        ccom = cv2.connectedComponentsWithStats(bw, 4, cv2.CV_32S)
        if ccom[0] > 1 and (ccom[2][1:, -1] > 100).any():
            arg = np.argmax(ccom[2][1:, -1]) + 1
            if self.new and 30 < ccom[3][arg, 0] < 90:
                self.hole += 1
                self.hole_flag = True
                if ccom[2][arg, -1] > 5000:
                    self.pole += 1
                    self.pole_flag = True
                self.empty, self.new = 0, 0
                print('Hole', self.hole, 'Pole', self.pole)
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
        inp = server_h.read_line()
        out = server_h.step()
        if out:
            out.strip(b'(')
            out.strip(b')')
#         if out:
#             print('Sending: %s' % out[:-1].decode())
#             server_h.write(out)
        if inp:
            print('Received %s' % inp.decode())
        if task.hole_flag:
            task.hole_flag = False
            print('Sending Hole: %d ' % (task.hole))
            server_h.write(b'Hole:' + out)
        if task.pole_flag:
            task.pole_flag = False
            print('Sending Pole')
            server_h.write(b'Pole')      
    task.end()
    server_h.end()

# Safe Guard
if __name__ == '__main__':
    main()

'''
cotton.py
Cotton detection script

Byron Hernandez
University of Florida
July 2022
'''
from camera_utils.realsense.camera import Camera
from comm_utils import Server
from config import *
import time
import cv2


class RealSenseService():
    def __init__(self):
        self.camera = Camera()
        
    def __call__(self, op=b'None'):
        output = False
        if op == b'ping_pong':
            output = self.ping_pong()
        elif op == b'cotton':
            output = self.cotton()
        return output
    
    def ping_pong(self):
        
    
    def cotton(self):
        pass
    
    def end(self):
        self.camera.close()
        


class RealSenseDummyService():
    def __init__(self):
        self.t = time.time()
        self.t0 = self.t
    
    def __call__(self):
        if time.time() - self.t > 3:
            self.t = time.time()
            message = 'cotton dummy answer %f' % (self.t - self.t0)
        else:
            message = False
        return message
    
    def end(self):
        pass


def main():
    task = RealSenseService()
    server_c = Server(HOST_RSe, PORT_RSe[1], 'Real-Sens', task)
    server_c.pair()
    while True:
        out = server_c.step()
        inp = server_c.read_line()
        if out:
            print('Sending: %s' % out[:-1].decode())
            server_c.write(out)
        if inp:
            print('Received %s' % inp.decode())

# Safe Guard
if __name__ == '__main__':
    main()

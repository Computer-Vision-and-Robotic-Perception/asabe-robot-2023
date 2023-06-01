'''
main_rpi.py

This is the main script for integration of the software to run the robot designed for the ASABE Robotics Competition 2022.

Script running in the Rapberry Pi

This main process keeps communication between all the following processes:
 - Computer vision (HolePole)[Raspberry Pi 4]
 - Computer vision (PingPong)[Nvidia Jetson Nano]
 - Computer vision (Cotton)[Nvidia Jetson Nano]
 - Communications with arduino

Byron Hernandez
University of Florida
July 2022
'''
from serial import PARITY_EVEN
from comm_utils import Client, Serial
from screen import ASABE_GUI
from nav_utils import *
from config import *
import time

class Robot():
    def __init__(self):
        self.running = False
        self.home = False
        self.start = False
        self.mapping = False
        self.harvesting = False
        self.turning = False
        self.deposing = False
        self.navigation_initialized = False
        self.current_row = 0
        self.last_hole = 0
        self.last_pole = 0
        self.coordinate_x = 25.4 * 6
        self.coordinate_y = 25.4 * 6
        self.coordinate_t = 0.0
        self.raw_ping_pong = []
        self.formated_ping_pong = []

def fromhole(rcv_h, robot):
    print('From Hole-Pole: ', rcv_h)
    com = rcv_h.split(b':')
    if com[0] == b'hole':
        robot.last_hole, robot.last_pole = com[1][1, -1].split(b',')
        print('hole: %d pole: %d' % (robot.last_hole, robot.last_pole))

def fromarduino(rcv_a, robot, client_a):
    print('From Ardu-MEGA: ', rcv_a)
    com = rcv_a.split(b':')
    if com[0] == b'nav init' or com[0] == b'navigate':
        out = com[1].split(b',')
        out = [float(n) for n in out]
        out = LIDAR_Output(np.array(out[:180]), np.array(out[180:]), robot.navigation_initialized)
        robot.navigation_initialized = True
        out = [b'%3.4f' % o for o in out]
        out = b','.join(out)
        client_a.write(b'xyt:' + out + b'\n')
    
# Main loop
def main():
    robot = Robot()
    robot.running = True
    client_h = Client(HOST_RPi, PORT_HPo[0], 'Hole-Pole')
    client_h.connect_to(HOST_HPo, PORT_HPo[1])
    client_s = Client(HOST_RPi, PORT_RSe[0], 'Real-Sens')
    client_s.connect_to(HOST_RSe, PORT_RSe[1])
#     client_a = Serial(port=PORT_Ard, baudrate=25e4, timeout=1e-3)
#     gui = ASABE_GUI()
#     info = [0, 0, 0, 0]
    # This is going to take the robot to init position (12"12"12")
#     client_a.write(b'home\n')            
#     answer_a = client_h.read_line()
    # wait for the home done ack
#     while answer_a is not b'home done':
#         answer_a = client_h.read_line()
#     robot.home = True
    # wait for the gui input
#     while not gui.button.isChecked():
#         pass
#     robot.start = True
    # This is going to take the robot to the starting position (Extended)
#     client_a.write(b'start\n')
    # Wait for start signal
#     while answer_a is not b'start done':
#         answer_a = client_a.read_line()
#     robot.start = True
    # Wait for ready to map signal
#     while answer_a is not b'start done':
#         answer_a = client_h.read_line()
#     robot.current_row = 1
#     t0 = time.time()
    while robot.running:
        rcv_h = client_h.read_line()
        rcv_s = client_s.read_line()
#         rcv_a = client_a.read_line()
        
        if rcv_h:
            fromhole(rcv_h, robot)
        if rcv_s:
            print('From Real-Sens: ', rcv_s)
#         if rcv_a:
#             fromarduino(rcv_a, robot, client_a)
        
#         if time.time() - t0 > 1.0:
#             client_s.write(b'cotton\n')
#             client_s.write(b'ping_pong\n')
#             print('sending Test')
#             client_a.write(b'Test\n')
#             inp = input('Command: ')
#             client_a.write(bytes(inp, 'utf-8') + b'\n')
#             t0 = time.time()
#         gui.app.exec_()
    client_h.end()
    client_s.end()
#     client_a.end()
        

# Safe Guard
if __name__ == '__main__':
    main()
    
# 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

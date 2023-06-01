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
from comm_utils import Client, Serial
from config import *
import time

VERSION = python_version().split('.')[0]

# Main loop
def main():
    client_h = Client(HOST_RPi, PORT_HPo[0], 'Hole-Pole')
    client_h.connect_to(HOST_HPo, PORT_HPo[1])
    client_s = Client(HOST_RPi, PORT_RSe[0], 'Real-Sens')
    client_s.connect_to(HOST_RSe, PORT_RSe[1])
    client_a = Serial(port=PORT_Ard, baudrate=2e6, timeout=1e-3)
    t1 = time.time()
    while True:
        data = client_h.read_line()
        if data:
            print('Answer Hole-Pole: ', data)
            client_h.write(data + b'\n')
        
        data = client_s.read_line()
        if data:
            print('Answer Real-Sens: ', data)
            client_s.write(data + b'\n')
        
        if time.time() - t1 > 3:
            t1 = time.time()
            client_a.write(b'Test')
            
        data = client_a.read_line()
        if data:
            print('Answer Ardu-MEGA: ', data)
        

# Safe Guard
if __name__ == '__main__':
    main()

'''
conf.py
General configurations

Byron Hernandez
University of Florida
July 2022
'''
import socket
from platform import python_version

# Localhost IP address
HOST_RPi = '192.168.0.1'
HOST_JNo = '192.168.0.2'
# Hole-Pole process socket (Raspberry)
HOST_HPo = HOST_RPi
PORT_HPo = 50001, 50002
# Real-Sens process socket (Jetson)
HOST_RSe = HOST_JNo
PORT_RSe = 50003, 50004
# Serial port to Arduino
PORT_Ard = '/dev/ttyS0'

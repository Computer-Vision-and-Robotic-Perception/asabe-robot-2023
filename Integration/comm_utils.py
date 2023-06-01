'''
comm_utils.py
General utilities for TCP/IP and Serial communications

Byron Hernandez
University of Florida
July 2022
'''
import time
import socket
import serial
from config import *
from socket import socket as socketObj
VERSION = python_version().split('.')[0]


class Socket(socketObj):
    def __init__(self, IP='localhost', port=50000,  name='Socket local'):
        super(Socket, self).__init__(socket.AF_INET, socket.SOCK_STREAM)
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.bind((IP, port))
        self.ip = IP
        self.port = port
        self.name = name
        self.buffer = b''
        print('%s at %s:%d has been created' % (name, IP, port))
      
    def write(self, msg):
        done = False
        while not done:
            try:
                self.conn.sendall(msg)
                done = True
            except:
                pass
                
    def read_line(self):
        try:
            data = self.conn.recv(1024)
            if data not in [False, '', 'False', b'', b'False', None]:
                self.buffer += data
        except:
            pass
        line = False
        if self.buffer is not b'':
            lines = self.buffer.split(b'\n')
            if len(lines) == 1:
                line = False
                self.buffer = lines[0]
            elif lines[0] == b'False':
                line = False
                self.buffer = b'\n'.join(lines[1:])     
            else:
                line = lines[0]
                self.buffer = b'\n'.join(lines[1:])             
        return line
    
    def end(self):
        self.shutdown(socket.SHUT_RDWR)
        self.close()
        print('%s at %s:%d has been closed' % (self.name, self.ip, self.port))    


class Server(Socket):
    def __init__(self, IP='localhost', port=50000,  name='Service local', task=type):
        super(Server, self).__init__(IP=IP, port=port, name=name)
        self.task = task
  
    def pair(self, timeout=None):
        self.settimeout(timeout)
        self.listen(5)
        self.conn, self.addr = self.accept()
        self.settimeout(1e-3)
        self.conn.settimeout(1e-3)
        time.sleep(1)
        print('%s at %s:%d received a connection from %s:%d' % (self.name, self.ip, self.port, self.addr[0], self.addr[1]))
    
    def step(self, **kwargs):
        out = self.task(**kwargs)
        if out is not False:
            out = bytes(out) + b'\n'
        return out


class Client(Socket):
    def __init__(self, IP='localhost', port=50000, name='Client local'):
        super(Client, self).__init__(IP=IP, port=port, name=name)
        self.conn = self
        
    def connect_to(self, IP, port, timeout=20.0):
        self.settimeout(timeout)
        self.connect((IP, port))
        self.settimeout(1e-3)
        print('%s has connected to server at %s:%d' % (self.name, IP, port))
        
        
class Serial(serial.Serial):
    def __init__(self, **kwargs):
        super(Serial, self).__init__(**kwargs)
        self.buffer = b''
        print('Connected to Serial at', PORT_Ard)
    
    def read_line(self):
        try:
            self.buffer += self.read(self.in_waiting)
        except:
            pass
        line = False
        if self.buffer is not b'':
            lines = self.buffer.split(b'\n')
            if len(lines) == 1:
                line = False
                self.buffer = lines[0]   
            else:
                line = lines[0]
                self.buffer = b'\n'.join(lines[1:])          
        return line
    

def test_serial_read_line():
    ser = Serial(port='/dev/ttyS0', baudrate=2e6, timeout=1e-3)
    while True:
        message = input('message to send ("exit" to close): ')
        message = bytes(message, 'utf-8')
        t1 = time.time()
        ser.write(message + b'\n')
        t2 = time.time()
        data, _ = ser.read_line()
        t3 = time.time()
        print('Answer A: ', data)
        print('TX Took %3.4f ms' % ((t2 - t1)*1000))
        print('RX Took %3.4f ms' % ((t3 - t2)*1000))
        
        if message == b'exit':
            break


def test_serial_original():
    ser = serial.Serial(port='/dev/ttyS0', baudrate=115200, timeout=0.1, write_timeout=0.1)
    print(ser.name)
    while True:
        message = input('message: ') + '\n'
        
        if VERSION == '2':
            message = bytes(message)
        else:
            message = bytes(message, 'utf-8')
        ser.write(message)

        ack = False
        while not ack:
            data = ser.read_until()
            ack = data == message
            print('Answer', data)
    

# Safe Guard
if __name__ == '__main__':
    pass
    # test_serial_original()
    # test_serial_readline()


#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def draw_line(out, rho, theta):
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
    cv2.line(out, pt1, pt2, (0, 0, 255), 1, cv2.LINE_AA)
    return out

class Publisher(Node):
    def __init__(self):
        super().__init__('locator')
        # self.im = np.ones((300, 300), dtype=int) * np.random.randint(0, 255)
        # self.timer = self.create_timer(timer_period_sec=3, callback=self.timer_callback)
        # Scanner localization
        self.ranges = None
        self.angles = None
        self.robot = {'x': 1.2, 'y': 0.3, 'th': 3.14} # original: 0.15, 0.15, 1.57079632679
        
        # Transformations robot-lidar
        self.Hrl = {'x': -0.109, 'y': 0.0, 'th': 0.0}  # Rx + t  || # This transformations never changes
        # Transformations world-robot (Initial position)
        self.Hrw = {'x': 0.15, 'y': 0.15, 'th': 0.0}
        # Transformation world-lidar : Hwr @ Hwr = Hwr - Hlr 
        # self.Hlw = {'x': }

        # TODO: Given the initial pose, compute the initial parametrization of the board edges 
        #       Go first trhough X and continue counterclockwise
        self.x0 = {'r': 0.0000, 'th': 0.0}       # x axis
        self.y0 = {'r': 0.0000, 'th': np.pi/2}   # y axis
        self.x1 = {'r': 2.4384, 'th': 0.0}       # parallel to x
        self.y1 = {'r': 2.4384, 'th': np.pi/2}   # parallel to y

        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    # def timer_callback(self):
    #     cv2.imwrite('Here.jpg', self.im)

    def get_lines(self, msg):
        a0 = msg.angle_min
        a1 = msg.angle_max
        da = msg.angle_increment
        self.ranges = np.array(msg.ranges)
        self.angles = np.arange(a0, a1, da)
        self.ranges[self.ranges > 12] = 12.0
        c = 120                 # center[pixels]
        res = 2.4384 / float(c) # [m/pixel] 
        im = np.zeros((2*c+1, 2*c+1), dtype=np.uint8)
        x, y = pol2cart(self.ranges, self.angles)
        x = (x / res).astype(int)
        y = (y / res).astype(int)
        for i, j in zip(x, y):
            if -c < i < c and -c < j < c:
                im[c + i - 1: c + i + 1, c + j - 1: c + j + 1] = 1
        return im, res, c, cv2.HoughLines(im, 1, np.pi / 180.0, 10, None, 0, 0) # for visualization change res

    def filter_lines(self, im, lines, save=False):
        out = cv2.cvtColor(im, cv2.COLOR_GRAY2RGB) * 255
        kept_rhos = []
        kept_thes = []
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                if i == 0: 
                    if save: draw_line(out, rho, theta)
                    kept_rhos.append(rho)
                    kept_thes.append(theta)
                    continue
                if np.all(1 < np.abs(np.array(kept_thes) - theta)) and \
                   np.all(np.abs(np.array(kept_thes) - theta) < np.pi - 1):
                    if save: draw_line(out, rho, theta)
                    kept_rhos.append(rho)
                    kept_thes.append(theta)

        if save: cv2.imwrite('Here.jpg', out)
        return kept_rhos, kept_thes

    def listener_callback(self, msg):
        header = msg.header
        if header.stamp.sec < self.get_clock().now().to_msg().sec: return
        im, res, c, lines = self.get_lines(msg)
        rho, theta = self.filter_lines(im, lines, save=False)
        rho, theta = (np.array(rho) - c) * res, np.array(theta)
        print(np.concatenate([rho, theta]).reshape((len(rho), 2)).T)






def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
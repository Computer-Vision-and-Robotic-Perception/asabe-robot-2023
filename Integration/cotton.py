'''
cotton.py
Cotton detection script

Byron Hernandez
University of Florida
July 2022
'''
from camera_utils.realsense.camera import Camera
from camera_utils.realsense.camera import rs
from pingpang import pole_assign
from comm_utils import Server
from config import *
import numpy as np
import imutils
import time
import cv2


class RealSenseService():
    def __init__(self, fps=6):
        self.camera = Camera(fps=fps)
        self.depth, self.color, self.xyz = self.camera.get_frame(mode='rotate')
        # Ping-Pong parameters
        self.exclusion_distance = 200 # If a matched line is farther than this away from a ball dont include it, its likely a bad detection.
        self.ball_distance = 100      # The distance from the center pole a ball is allowed to be and still be able to be assigned to the pole 
        self.circle_mask_size = 45    # How large of a radius around each found circle should be included in the line detection range.
        self.low_threshold = 50
        self.high_threshold = 150
        self.rho = 1                  # distance resolution in pixels of the Hough grid
        self.theta = np.pi / 90       # 180  # angular resolution in radians of the Hough grid
        self.threshold = 15           # minimum number of votes (intersections in Hough grid cell)
        self.min_line_length = 30     # minimum number of pixels making up a line
        self.max_line_gap = 15        # maximum gap in pixels between connectable line segments
        # Color thresholds for ping pong mapping
        self.low_brown = np.array([60, 10, 20])
        self.high_brown = np.array([90, 80, 60])
        self.low_b = np.array([1, 30, 40])
        self.high_b = np.array([20, 140, 80])
        self.low_black = np.array([150,20,30])
        self.high_black = np.array([170,90,100])
        self.greenLower = (50, 86, 40)
        self.greenUpper = (100, 255, 255)
        # Cotton parameters
        self.clipping_distance = 0.3 #0.4572 # meters
        self.clipping_distance = self.clipping_distance/self.camera.depth_scale
        # Color thresholds for cotton mapping
        self.low_green = np.array([50,86,40])
        self.high_green = np.array([100,255,255])
        self.low_brown = np.array([3,70,20])
        self.high_brown = np.array([20,180,80])
        # Kernels
        self.kernel05 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.kernel08 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8))
        self.kernel10 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
        self.kernel20 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20, 20))
        
    def __call__(self, op=b'None'):
        if op == b'ping_pong':
            output = self.ping_pong()
        elif op == b'cotton':
            output = self.cotton()
        else:
            self.depth, self.color, self.xyz = self.camera.get_frame(mode='rotate')
            output = False
        return output
    
    def cotton(self):
        depth_image_3d = np.dstack((self.depth, self.depth, self.depth))
        bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), 0, self.color)
        hsv = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV)
        brownMask = cv2.inRange(hsv, self.low_brown, self.high_brown)
        greenMask = cv2.inRange(hsv, self.low_green, self.high_green)
        Mask =  brownMask | greenMask
        src = cv2.bitwise_and(bg_removed, bg_removed, mask=Mask)
        img = cv2.subtract(bg_removed, src)
        #binary = cv2.erode(img, self.kernel20, iterations=1)
        binary = cv2.erode(img, self.kernel05, iterations=2)
        #binary = cv2.erode(binary, self.kernel08, iterations=1)
        gray = cv2.cvtColor(binary, cv2.COLOR_BGR2GRAY)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(gray, connectivity=8)
        argmax = np.argmax(stats[1:, -1]) + 1
        # Comment from here:
        frame = cv2.circle(self.color, (int(centroids[argmax, 0]), int(centroids[argmax, 1])), 5, (0, 255, 0), -1)
        cv2.imshow('Cotton result', frame)
##        cv2.imshow('mask', gray)
        cv2.waitKey(1)
        # End of comment
        return centroids[argmax]
    
    def ping_pong(self):
        frame = self.color
        orig = frame
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # Apply green masks
        cnts, greenresult = self.get_green_mask(frame, hsv)
##        # Segment branches
##        result, gray = self.segment_branches(greenresult)
##        # Estimate lines
##        lines = self.get_lines(gray)
##        # Associate lines with center pole
##        found_balls = self.associate_balls(orig, cnts, lines)
        
        found_balls = []
        for c in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            found_balls.append((int(x), int(y)))
        
        ps_c = []
        for ps in found_balls:
            (x, y) = ps
            coords = [-self.xyz[y, x][0], self.xyz[y, x][1], self.xyz[y, x][2]]
            ps_c.append(coords)
        
        # Comment from here:
        # print(np.array(ps_c))
        for centroid in found_balls:
            frame = cv2.circle(frame, (centroid[0], centroid[1]), 5, (0, 0, 255), -1)
        cv2.imshow('Ping Pong result', frame)
        cv2.waitKey(1)
        # End of comment
        
        return ps_c
    
    def get_green_mask(self, frame, hsv):
        GreenMask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
        GreenMask = cv2.erode(GreenMask, None, iterations=2)
        GreenMask = cv2.dilate(GreenMask, None, iterations=2)
        cnts = cv2.findContours(GreenMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        circlemask = np.zeros_like(frame)
        for c in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if radius > 10:
                cv2.circle(circlemask, (int(x), int(y)), int(radius + self.circle_mask_size), (100, 255, 255), -1)
        # Applying Circle Mask
        greencirclemask = cv2.inRange(circlemask, self.greenLower, self.greenUpper)
        greenresult = cv2.bitwise_and(frame, frame, mask = greencirclemask)
        return cnts, greenresult
    
    def segment_branches(self, greenresult):
        hsv = cv2.cvtColor(greenresult, cv2.COLOR_BGR2HSV)
        
        blackMask = cv2.inRange(hsv, self.low_black, self.high_black)
        brownMask = cv2.inRange(hsv, self.low_brown, self.high_brown)
        bMask = cv2.inRange(hsv, self.low_b, self.high_b)

        finalMask = brownMask | blackMask | bMask

        divide = cv2.divide(finalMask, finalMask, scale=255)
        thresh = cv2.threshold(divide, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
        result = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, self.kernel10)
        return result, finalMask
    
    def get_lines(self, gray):
        edges = cv2.Canny(gray, self.low_threshold, self.high_threshold)
        lines = cv2.HoughLinesP(edges, self.rho, self.theta, self.threshold, np.array([]), self.min_line_length, self.max_line_gap)
    
    def associate_balls(self, orig, cnts, lines):
        found_balls = []
        for c in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size
            if radius > 10:
                dist = 50000 # an impossibly high starting distance
                # Empty Saved line coordinates
                xs1, ys1, xs2, ys2 = 0, 0, 0, 0
                # Finding the closest line to this iteration's center
                for line in lines:
                    for x1,y1,x2,y2 in line:
                        # Calculating distance from current center to current line
                        testpoint = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
                        testdist = np.linalg.norm(center-testpoint)
                        
                        # If a closer line was found update our "closest" line for this center
                        if testdist < dist:
                            dist = testdist
                            xs1, ys1, xs2, ys2 = xs1 = x1, y1, x2, y2
                
                # Checking to see if found line is reasonably close enough
                if dist > self.exclusion_distance:
                    continue
                
                # Cheking to see if current ball is reasonably close enough
                if center[0] > w//2 + ball_distance or center[0] < w//2 - self.ball_distance:
                    continue
                
                # Slope Calculation
                slope = round((ys2 - ys1) / (xs2 - xs1), 2)
                
                # Getting a new random color for this match
                if (slope > 0 and xs1 < w//2) or (slope < 0 and xs1 > w//2):
                    # If these slopes, then this ball belongs to the center pole
                    randcolor = (0,0,255)
                    # Adding to list of centers beloning to the center pole
                    found_balls.append(center)
                else:
                    randcolor = (random.randint(100,255), random.randint(100,255), random.randint(100,255))
                
                # Drawing this matching center/line on original image
                cv2.circle(orig, (int(x), int(y)), int(radius), randcolor, 2)
                cv2.circle(orig, center, 3, randcolor, -1)
                cv2.line(orig, (xs1, ys1), (xs2, ys2), randcolor, 2)
                
                # Print the slope of each line
                # new_image = cv2.putText(img=orig, text=str(-slope), org=(xs1, ys1), fontFace=cv2.FONT_ITALIC, fontScale=0.5, color=randcolor, thickness=2)
        return found_balls
        
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
    #task = RealSenseService(fps=15)
    task = assign_pole
    server_c = Server(HOST_RSe, PORT_RSe[1], 'Real-Sens', task)
    server_c.pair()
    while True:
        inp = server_c.read_line()
        out = server_c.step(op=inp)  
        if out:
            print('Sending: %s' % out[:-1].decode())
            server_c.write(out)
        if inp:
            print('Received %s' % inp.decode())
#     task.end()
    server_c.end()


def test_task():
    task = RealSenseService(fps=30)
    key = b''
    rand_img = np.random.rand(500, 500)
    while True:
        cv2.imshow('rand', rand_img)
        key = cv2.waitKey(1)
        if key == ord('c'):
            out = task(b'cotton')
        elif key == ord('p'):
            out = task(b'ping_pong')
        elif key == ord('q'):
            break
        else:
            out = task(b'None')
        if out is not False:
            print(out)    
    task.end()


# Safe Guard
if __name__ == '__main__':
    main()
##    test_task()

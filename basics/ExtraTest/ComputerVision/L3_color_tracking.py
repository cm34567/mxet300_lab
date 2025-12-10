# L3_color_tracking.py
# This program was designed to have SCUTTLE following a target using a USB camera input

import cv2              # For image capture and processing
import numpy as np      
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import L2_kinematics as kin
import netifaces as ni
from time import sleep, time
from math import radians, pi
from enum import Enum


class CVTracking:
    @staticmethod
    def getIp():
        for interface in ni.interfaces()[1:]:   #For interfaces eth0 and wlan0
            try:
                ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
                return ip
                
            except KeyError:                    #We get a KeyError if the interface does not have the info
                continue                        #Try the next interface since this one has no IPv4
            
        return 0
    
    @staticmethod
    def getCamera(url, size):
        try:
            camera = cv2.VideoCapture(0)    
        except: 
            print("Error initializing camera")
            pass
        # Try opening camera stream if default method failed
        if not camera.isOpened():
            camera = cv2.VideoCapture(url)    
        camera.set(3, size[0])                       # Set width of images that will be retrived from camera
        camera.set(4, size[1])                       # Set height of images that will be retrived from camera
        return camera


    def __init__(self, size=(240,160), fov=1, HSVMin=(0,0,0), HSVMax=(255,255,255), kernel=None):
        #    Camera
        self.stream_ip = self.getIp()
        if not self.stream_ip: 
            print("Failed to get IP for camera stream")
            exit()
        self.stream_url = 'http://' + self.stream_ip + ':8090/?action=stream'   # Address for stream

        self.size = size
        self.fov = fov
        self.HSVMin = HSVMin
        self.HSVMax = HSVMax
        self.box = (0,0,0,0)
        self.lastFound = 0
        
        self.camera = self.getCamera(self.stream_url, self.size)
        self.kernel = np.ones((5,5),np.uint8) if kernel is None else kernel

    def getBox(self):
        return self.box

    def getStaleness(self):
        return time() - self.lastFound

    def process_image(self):
        ret, image = self.camera.read()  # Get image from camera

        # Make sure image was grabbed
        if not ret:
            print("Failed to retrieve image!")
            return None

        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)              # Convert image to HSV
        height, width, channels = image.shape                       # Get shape of image
        thresh = cv2.inRange(image, self.HSVMin, self.HSVMax)   # Find all pixels in color range

                                    # Set kernel size
        mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, self.kernel)     # Open morph: removes noise w/ erode followed by dilate
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)      # Close morph: fills openings w/ dilate followed by erode
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
        
        if len(cnts) and len(cnts) < 3:                             # If more than 0 and less than 3 closed shapes exist
            self.box = cv2.boundingRect(max(cnts, key=cv2.contourArea)) # return the largest target area
            self.lastFound = time()
        else:
            return None

        return self.box

class CVPathingManager:
    def __init__(self):
        self.tracker = CVTracking(HSVMin = (45, 150, 100), HSVMax = (85,255,255))

        self.target_width = 100         # Target pixel width of tracked object
        self.target_angle = 0.0         # Target pixel width of tracked object
        self.angle_margin_small = 0.1   # Radians object can be from image center to be considered "centered"
        self.angle_margin_big = 0.2     # Radians object can be from image center before turn-only
        self.width_margin = 10          # Minimum width error to drive forward/back
        self.adjust = ()

    def getErrorValues(self):
        x,y,w,h = self.tracker.getBox()                           # Get bounding rectangle (x,y,w,h) of the largest contour
        center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
        angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered
        return w - self.target_width, angle - self.target_angle

    def calc_tracking_kinematics(self):
        e_width, e_angle = self.getErrorValues()
        if (abs(e_angle) > self.angle_margin_big):
            turn_effort = 2*e_angle/self.target_angle
            fwd_effort = 0
        else:
            if (abs(e_width) > self.width_margin):
                fwd_effort = e_width/self.target_width
            else:
                fwd_effort = 0
            
            if (abs(e_angle) > self.angle_margin_small):
                turn_effort = e_angle/self.target_angle
            else:
                turn_effort = 0
        
        return fwd_effort, turn_effort
    
    def homing_kinematics(self):
        pass #TODO: IMPLEMENT
        

def main():

    cvProcessor = CVTracking(HSVMin = (45, 150, 100), HSVMax = (85,255,255))
    box = (0,0,0,0)

    try:
        while True:
            sleep(.05)                                          

            findBox = cvProcessor.process_image()
            box = box if findBox is None else findBox
            x,y,w,h = box                           # Get bounding rectangle (x,y,w,h) of the largest contour
            center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
            angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered


            if len(cnts) and len(cnts) < 3:                             # If more than 0 and less than 3 closed shapes exist
                x,y,w,h = box                           # Get bounding rectangle (x,y,w,h) of the largest contour
                center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
                angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered

                wheel_measured = kin.getPdCurrent()                     # Wheel speed measurements

                # If robot is facing target
                if abs(angle) < angle_margin:                                 
                    e_width = target_width - w                          # Find error in target width and measured width

                    # If error width is within acceptable margin
                    if abs(e_width) < width_margin:
                        sc.driveOpenLoop(np.array([0.,0.]))             # Stop when centered and aligned
                        print("Aligned! ",w)
                        continue

                    fwd_effort = e_width/target_width                   
                    
                    wheel_speed = ik.getPdTargets(np.array([0.8*fwd_effort, -0.5*angle]))   # Find wheel speeds for approach and heading correction
                    sc.driveClosedLoop(wheel_speed, wheel_measured, 0)  # Drive closed loop
                    print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
                    continue

                wheel_speed = ik.getPdTargets(np.array([0, -1.1*angle]))    # Find wheel speeds for only turning

                sc.driveClosedLoop(wheel_speed, wheel_measured, 0)          # Drive robot
                print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)

            else:
                print("No targets")
                sc.driveOpenLoop(np.array([0.,0.]))         # stop if no targets detected

                
    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        pass

    finally:
    	print("Exiting Color Tracking.")

if __name__ == '__main__':
    main()

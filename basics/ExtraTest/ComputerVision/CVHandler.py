import cv2              # For image capture and processing
import numpy as np
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


    def __init__(self, size=(240,160), HSVMin=(0,0,0), HSVMax=(255,255,255), kernel=None):
        #    Camera
        self.stream_ip = self.getIp()
        if not self.stream_ip: 
            print("Failed to get IP for camera stream")
            exit()
        self.stream_url = 'http://' + self.stream_ip + ':8090/?action=stream'   # Address for stream

        self.size = size
        self.HSVMin = HSVMin
        self.HSVMax = HSVMax
        self.box = (0,0,0,0)
        self.lastFound = 0
        
        self.camera = self.getCamera(self.stream_url, self.size)
        self.kernel = np.ones((5,5),np.uint8) if kernel is None else kernel

    """
    Returns last known location, may be stale. Use responsibly.
    """
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
    def __init__(self, fov=1, allowable_staleness = 1):
        self.tracker = CVTracking(HSVMin = (45, 140, 40), HSVMax = (80,255,255))

        self.allowable_staleness = allowable_staleness
        self.fov = fov

        self.target_width = 100         # Target pixel width of tracked object
        self.width_margin_lock = 20         # Target pixel width of tracked object
        self.width_margin = 10          # Minimum width error to drive forward/back

        self.target_angle = 0.0         # Target pixel width of tracked object
        self.angle_margin_lock = 0.08
        self.angle_margin_small = 0.02   # Radians object can be from image center to be considered "centered"
        self.angle_margin_big = 0.2     # Radians object can be from image center before turn-only
        
        self.lock_threshold = 5
        self.lock_confidence = 0
        self.lock_max_confidence = 10
        self.lock_confidence_decay = 2

    def editLockOnSettings(self, lock_threshold=None, lock_max_confidence=None, lock_confidence_decay=None):
        self.lock_confidence = 0
        if(not lock_threshold is None): self.lock_threshold = lock_threshold 
        if(not lock_max_confidence is None): self.lock_max_confidence = lock_max_confidence
        if(not lock_confidence_decay is None): self.lock_confidence_decay = lock_confidence_decay

    """
    Returns last known location, may be stale. Use responsibly.
    """
    def getAngle(self, x):
        return round(((x/self.tracker.size[0])-0.5)*self.fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered

    """
    Returns last known location, may be stale. Use responsibly.
    """
    def getErrorValues(self): 
        x,y,w,h = self.tracker.getBox()                           # Get bounding rectangle (x,y,w,h) of the largest contour
        angle = self.getAngle(int(x+0.5*w))
        return w - self.target_width, angle - self.target_angle
    
    def check_lock(self, dw, da):
        # print(f"|{da}| < {self.angle_margin_lock} & |{dw}| < {self.width_margin_lock} = {abs(da) < self.angle_margin_lock} & {abs(dw) < self.width_margin_lock} = {abs(da) < self.angle_margin_lock and abs(dw) < self.width_margin_lock}")
        if(self.allowable_staleness < self.tracker.getStaleness()):
            self.lock_confidence = 0
            return False

        if(abs(da) < self.angle_margin_lock and abs(dw) < self.width_margin_lock):
            if(self.lock_confidence < self.lock_max_confidence): self.lock_confidence += 1
        else:
            self.lock_confidence = max(0, self.lock_confidence-self.lock_confidence_decay)
        return self.lock_confidence >= self.lock_threshold

    def getLock(self):
        return self.lock_confidence >= self.lock_threshold 

    def calc_tracking_kinematics(self):
        e_width, e_angle = self.getErrorValues()
        if (abs(e_angle) > self.angle_margin_big):      #Big angle: turn only
            turn_effort = 2*e_angle
            fwd_effort = 0
        else:                                           #Small angle: arc
            if (abs(e_width) > self.width_margin):
                fwd_effort = e_width/self.target_width
            else:
                fwd_effort = 0
            
            if (abs(e_angle) > self.angle_margin_small):
                turn_effort = e_angle
            else:
                turn_effort = 0

        return fwd_effort, turn_effort
    
    def homing_kinematics(self):
        pass #TODO: IMPLEMENT


def main():
    print("=== CV Tracking Test ===")
    pm = CVPathingManager()
    tracker = pm.tracker
    lockThreshold = pm.lock_threshold

    try:
        while True:
            box = tracker.process_image()
            staleness = tracker.getStaleness()
            if box:
                x, y, w, h = box
                a = pm.getAngle(x+0.5*w)
                dw, da = pm.getErrorValues()
                pm.check_lock(dw,da)
                print(f"Box: w={w}, a={a}; Error: dw = {dw}, da = {da}; Lock: {pm.lock_confidence}/{lockThreshold}")
            else:
                print(f"No target detected. Staleness: {staleness:.2f}s")
            
            # Quit on pressing 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            sleep(0.4)  # control loop rate

    except KeyboardInterrupt:
        print("\nStopping test...")

    finally:
        tracker.camera.release()
        cv2.destroyAllWindows()
        print("Test finished.")


if __name__ == "__main__":
    main()
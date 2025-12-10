import numpy as np
import queue
import threading
from math import ceil, pi
from enum import Enum
from time import time, sleep

from .RobotModel import TankRobot



def rotate_2d_vector(vector, angle_radians):
    rotation_matrix = np.array([
        [np.cos(angle_radians), -np.sin(angle_radians)],
        [np.sin(angle_radians), np.cos(angle_radians)]
    ])
    rotated_vector = np.dot(rotation_matrix, vector)
    return rotated_vector

def getDeltaAngleDeg(theta1, theta2): 
    """
    Outputs difference in angle, in degrees, from -180 to 180
    """
    return ((theta2 - theta1 + 180) % 360) - 180

def getNPDeltaAngleDeg(theta1, theta2):
    """
    Computes difference between angles (theta2 - theta1) in degrees,
    wrapped to [-180, 180].
    *WARNING: NP arrays/scalars only*
    """
    return np.mod(theta2 - theta1 + 180, 360) - 180

def getDeltaAngleRad(theta1, theta2): 
    """
    Outputs difference in angle, in rad, from -pi to pi
    """
    return ((theta2 - theta1 + pi) % 2*pi) - pi

def getNPDeltaAngleRad(theta1, theta2):
    """
    Computes difference between angles (theta2 - theta1) in rad,
    wrapped to [-pi, pi].
    *WARNING: NP arrays/scalars only*
    """
    return np.mod(theta2 - theta1 + pi, 2*pi) - pi

def getHeadingVector(heading):
    return np.array([np.cos(heading), np.sin(heading)])

class OdometryHandler:

    class LogType(Enum):
        MISSED_TICK = 0

    def __init__(self, robot: TankRobot, tick_wait=0.02, pose_init = (0.0,0.0), heading_init = 0):

        self.robot = robot
            
        # --- Pos/Vel ---
        self.pos_global = np.array(pose_init)   # [m, m]
        self.heading_global = heading_init       # [rad]
        self.v_robot = np.zeros(2)      # [m/s, rad/s]
        self.dmotors_rad = np.zeros(2)
        self.vmotors_rad = np.zeros(2)

        # --- Last condition ---
        self.last_encoder_pos = self.robot.readShaftsRad()  # rad
        self.last_tick = time()

        # --- Tick thread control ---
        self.tick_wait = tick_wait
        self._tick_thread = None
        self._running = False
        self._lock = threading.Lock()
        self._MIN_TICK = self.tick_wait/10 # TREAT AS IMMUTABLE: Minimum tick difference to prevent noise amplification. May occur due to time() inconsistencies

        self._log = queue.Queue()

        # --- Kinematics matrices ---
        # Forward kinematics: [dx, dtheta] = A @ [left_wheel, right_wheel]
        self.A = np.array([
            [self.robot.wheel_radius/2, self.robot.wheel_radius/2],
            [-self.robot.wheel_radius/(2*self.robot.half_wheelbase), self.robot.wheel_radius/(2*self.robot.half_wheelbase)]
        ])

    def getGlobalPose(self):
        with self._lock:
            return self.pos_global.copy(), self.heading_global

    
    def getRobotVelocity(self):
        with self._lock:
            return self.v_robot.copy()

    def getMotorSpeed(self):
        with self._lock:
            return self.dmotors_rad.copy()

    def tick(self):
        """Atomically update global pose and velocities from wheel encoders."""
        with self._lock:
            # --- Read sensors and timestamp ---
            cur_time = time()
            cur_encoder_pos = self.robot.readShaftsRad()  # [rad]

            # --- Compute motion deltas ---
            dt = cur_time - self.last_tick
            self.dmotors_rad = getNPDeltaAngleRad(self.last_encoder_pos, cur_encoder_pos)
            dwheels_rad = self.dmotors_rad * self.robot.pulley_ratio
            dr_local, dtheta = self.A @ dwheels_rad  # local [dx, dtheta]

            # --- Update position, heading, velocity ---
            self.heading_global += dtheta
            self.pos_global += dr_local * getHeadingVector(self.heading_global)  # rotate to global frame
            if dt > self._MIN_TICK:  # avoid divide-by-zero or noise spikes
                self.v_robot = self.A @ (dwheels_rad / dt)  # [linear m/s, angular rad/s]
                self.vmotors_rad = self.dmotors_rad / dt

            # --- Save state for next tick ---
            self.last_encoder_pos = cur_encoder_pos
            self.last_tick = cur_time

    def checkLog(self):
        outString = ""
        while True: 
            try:    #ChatGPT says this is thread safe :)
                code, val = self._log.get()
            except queue.Empty:
                break
            
            if(code == LogType.MISSED_TICK):
                outString += f"WARN.ODOMETER: Missed {val} tick(s)\n"

        return outString


    def _tick_loop(self):
        """Thread target: continuously call tick at fixed rate."""
        next_tick = time()
        wait = self.tick_wait

        while self._running:
            next_tick += self.tick_wait
            self.tick()
            time_to_sleep = next_tick - time()
            if time_to_sleep > 0:
                sleep(time_to_sleep)
            else:
                missed = ceil((time() - next_tick)/self.tick_wait)
                next_tick += missed*self.tick_wait
                self._log.put((0, missed))
                sleep(max(0, next_tick - time()))

    def start(self):
        """Start the odometry thread."""
        if self._tick_thread is None or not self._tick_thread.is_alive():
            self._running = True
            self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
            self._tick_thread.start()
            print("ODOMETRY: Thread Running")

    def stop(self):
        """Stop the odometry thread."""
        self._running = False
        if self._tick_thread is not None:
            self._tick_thread.join()
            self._tick_thread = None



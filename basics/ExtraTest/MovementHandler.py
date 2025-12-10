# speed_control.py takes target speeds and generates duty cycles
# to send to motors, and has a function to execute PID control.

# Import external libraries
import numpy as np                                  # for handling arrays
import threading
from time import time, sleep
from math import pi, ceil

# Import local files
from MovementKinematics.RobotModel import TankRobot
from  MovementKinematics.Odometry import OdometryHandler
from MovementKinematics.Motor import HBridgePWMMotor


class PID:

    def __init__(self, gains = (1,0,0), ki_max=1):
        self.gains = gains
        self.target = None

        self._u_i = 0.0
        self._u_i_max = ki_max
        self._e_prev = None

    def getEfforts(self, current, dt):
        # GENERATE COMPONENTS OF THE CONTROL EFFORT, u
        if(self.target == 0):
            return 0, 0, 0
        # GENERATE COMPONENTS OF THE CONTROL EFFORT, u
        e = self.target - current
        u_p = (e * self.gains[0])            # proportional term
        self._u_i += np.clip(e * self.gains[1] * dt, -self._u_i_max, self._u_i_max)      # integral term

        if(self._e_prev is None):
            u_d = 0
        else:
            u_d = ((e-self._e_prev)/dt * self.gains[2])
        self._e_prev = e

        return u_p, self._u_i, u_d

    def getEffort(self, current, dt):
        
        # GENERATE COMPONENTS OF THE CONTROL EFFORT, u
        if(self.target == 0):
            return 0

        e = self.target - current
        u_p = (e * self.gains[0])            # proportional term
        self._u_i += np.clip(e * self.gains[1] * dt, -self._u_i_max, self._u_i_max)      # integral term

        if(self._e_prev is None):
            u_d = 0
        else:
            u_d = ((e-self._e_prev)/dt * self.gains[2])
        self._e_prev = e

        return u_p + self._u_i + u_d



"""
Feedforward test
Effort:-1.0; Speed:[0. 0.]
Effort:-0.9; Speed:[ 15.11621235 -15.42314052]
Effort:-0.8; Speed:[ 13.26863012 -14.18899753]
Effort:-0.7; Speed:[ 12.27185801 -13.88253937]
Effort:-0.6; Speed:[ 10.77828111 -11.12349296]
Effort:-0.5; Speed:[  9.81772048 -10.77648224]
Effort:-0.4; Speed:[ 8.16942936 -9.78030275]
Effort:-0.3; Speed:[ 6.482466   -6.55918157]
Effort:-0.2; Speed:[ 4.40956818 -4.67797667]
Effort:-0.1; Speed:[ 7.01729957 -7.93760116]
Effort: 0.0; Speed:[ 0.19188503 -0.49890107]
Effort: 0.1; Speed:[-0.03821562  0.07643124]
Effort: 0.2; Speed:[-0.03834407  0.11503221]
Effort: 0.3; Speed:[-7.20988848  9.6643186 ]
Effort: 0.4; Speed:[-0.95880749  1.22727358]
Effort: 0.5; Speed:[-5.25664643  6.63795498]
Effort: 0.6; Speed:[-7.90019695  8.28370165]
Effort: 0.7; Speed:[-6.90308471  7.01813612]
Effort: 0.8; Speed:[-10.51504868  11.2058183 ]
Effort: 0.9; Speed:[-12.04176067  13.65244203]
Effort: 1.0; Speed:[-12.57895436  13.3076133 ]
"""


class TankDriveController:
    def __init__(self, robot, velGains=((1,0,0),(1,0,0)), posGains=((1,0,0),(1,0,0))):
        self.robot = robot
        self.odometry = OdometryHandler(robot,tick_wait=0.01)
        self._filtered_vmotors = np.zeros(2)
        self._filter_n = 3
        self._filter_scale = (self._filter_n-1)/self._filter_n

        self._vel_k_ffwd = np.array([.10, .08])
    
        self._lock = threading.Lock()
        self._tick_wait = 0.02

    def _calcVelFeedforward(self, speeds):
        return map(lambda x, y: x*y, speeds, self._vel_k_ffwd)

    def set_velocity_target(self, motorSpeeds):
        """
        Velocity Thread needs to be started for motors to spin
        """
        self.motorVelocityPID[0].target = motorSpeeds[0]
        self.motorVelocityPID[1].target = motorSpeeds[1]

    def _velCtrlTick(self, dt):
        with self._lock:
            # Compute PID efforts
            alpha = self._filter_scale
            self._filtered_vmotors = alpha*self._filtered_vmotors + (1-alpha)*self.odometry.vmotors_rad
            ffwds = self._calcVelFeedforward((self.motorVelocityPID[0].target,self.motorVelocityPID[1].target))
            for motor, pid, vmotor, ffwd in zip(self.robot.motors, self.motorVelocityPID, self._filtered_vmotors, ffwds):
                # effort = pid.getEffort(vmotor, dt)
                kp, ki, kd = pid.getEfforts(vmotor, dt)
                effort = kp + ki + kd
                print(f"T:{pid.target:6.2f} A:{vmotor:6.2f} E:{pid.target - vmotor:6.2f}; Effort:{effort:7.3f} kp:{kp:7.3f} ki:{ki:7.3f} kd:{kd:3.3f}")
                motor.sendPWM(HBridgePWMMotor.conditionPWM(effort+ffwd))

    def _velCtrlThreadLoop(self):
        """Thread target: continuously call velocity control tick at fixed rate."""
        next_tick = time()
        wait = self._tick_wait

        real_tick_start_prev = next_tick - wait

        while self._velCtrlThreadRunning:
            next_tick += wait
            real_tick_start_cur = time()
            dt = real_tick_start_cur - real_tick_start_prev
            self._velCtrlTick(dt)
            real_tick_start_prev = real_tick_start_cur

            time_to_sleep = next_tick - time()
            if time_to_sleep > 0:
                sleep(time_to_sleep)
            else:
                missed = ceil((time() - next_tick) / self._tick_wait)
                next_tick += missed * wait
                self._log.put((0, missed))
                sleep(max(0, next_tick - time()))


    def start_velocity_thread(self):
        if self._posCtrlThread == True:
            return
        if self._velCtrlThread is None or not self._velCtrlThread.is_alive():
            self._velCtrlThreadRunning = True
            self._velCtrlThread = threading.Thread(target=self._velCtrlThreadLoop, daemon=True)
            self._velCtrlThread.start()


    def start_odometry(self):
        self.odometry.start()

    def stop_odometry(self):
        self.odometry.stop()

    def getRobotVelocity(self):
        return self.odometry.getRobotVelocity()

    def getRobotPose(self):
        return self.odometry.getRobotPose()



def main():
    # --- Create robot instance ---
    robot = TankRobot()  # uses default pins, bus, etc.

    # --- Create controller with PID gains ---
    # Gains: (Kp, Ki, Kd)
    velGains = ((0.10, 0.03, 0.001), (0.10, 0.03, 0.001))
    posGains = ((0.50, 0.03, 0.001), (0.50, 0.03, 0.001))
    controller = TankDriveController(robot, velGains=velGains, posGains=posGains)
    controller.start_odometry()
    sleep(0.05)
    
    """
     # Run for 40 seconds
    for effort in range(-10,11):
        robot.motors[0].sendPWM(HBridgePWMMotor.conditionPWM(-effort/10))
        robot.motors[1].sendPWM(HBridgePWMMotor.conditionPWM(effort/10))
        print(f"Effort:{effort:4.1f}; Speed:{controller.odometry.vmotors_rad}")
        sleep(2)

    """
    # --- Start velocity PID thread ---
    sleep(0.1)
    controller.start_velocity_thread()

    # --- Set target motor velocities ---
    target_velocities = [5.0, -5.0]  # rad/s for left and right wheels
    controller.set_velocity_target(target_velocities)

    print("Controller running. Motors should start moving...")

    try:
        # Run for 5 seconds
        for i in range(5):
            # Optionally, print odometry info
            print(f"Wheel speeds: {controller.odometry.vmotors_rad}")
            sleep(1)

    finally:
        # --- Stop controller thread ---
        controller._velCtrlThreadRunning = False
        controller._velCtrlThread.join()
        print("Controller stopped.")

        # Optionally, stop motors
        for motor in robot.motors:
            motor.sendPWM(0.0)
        print("Motors stopped.")

if __name__ == "__main__":
    main()

# Motors Program for SCUTTLE running RasPi
# This example sends commands to two motors on the appropriate pins for H-bridge
# For pin mapping, see Wiring Guide Pi on the SCUTTLE webpage.
# Last update: 2020.11 with improved PWM method

# Import external libraries
import gpiozero                             # used for PWM outputs
from gpiozero import PWMOutputDevice as pwm # for driving motors, LEDs, etc
import time                                 # for keeping time
import numpy as np                          # for handling arrays


class HBridgePWMMotor:
    @staticmethod
    def computePWM(speed):              # take an argument in range [-1,1]
        if speed == 0:
            x = np.array([0,0])         # set all PWM to zero
        else:
            x = speed + 1.0             # change the range to [0,2]
            chA = 0.5 * x               # channel A sweeps low to high
            chB = 1 - (0.5 * x)         # channel B sweeps high to low
            x = np.array([chB, chA])    # store values to an array
        return(x)

    def __init__(self, channels, freq=150):
        # Broadcom (BCM) pin numbering for RasPi is as follows: PHYSICAL:       NAME:
        self.channels = (pwm(channels[0], frequency=freq,initial_value=0), pwm(channels[1], frequency=freq,initial_value=0))

    def sendPWM(self, speed):          # takes at least 0.3 ms
        PWMVals = HBridgePWMMotor.computePWM(speed)
        self.channels[0].value = PWMVals[0]
        self.channels[1].value = PWMVals[1]

    @staticmethod
    def scaleMotorEffort(x):                             # a fcn to compress the PWM region where motors don't turn
        if -0.222 < x and x < 0.222:
            y = (x * 3)
        elif x > 0.222:
            y = ((x * 0.778) + 0.222)
        else:
            y = ((x * 0.778) - 0.222)
        return y
    
    @staticmethod
    def conditionPWM(effort):
        # CONDITION THE SIGNAL BEFORE SENDING TO MOTORS
        u = HBridgePWMMotor.scaleMotorEffort(effort)    # perform scaling - described above
        u = np.clip(u, -1, 1)
        return u

    @staticmethod
    def openLoop(speed):
        return max(min(-.99, (speed * 1/phi_max * DRS)),.99) # Scale, clip, return

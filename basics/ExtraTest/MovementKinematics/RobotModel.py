import smbus2       # a Python package to communicate over i2c
import numpy as np
from math import pi

from .Encoder import Encoder
from .Motor import HBridgePWMMotor


class TankRobot:
    """
    Encapsulates geometry, encoders, and kinematics for a differential-drive robot.
    Should be treated as immutable
    """
    def __init__(
        self,
        i2c_bus = None,
        wheel_radius=0.041,
        half_wheelbase=0.201,
        encoder_bits=14,
        pulley_ratio=0.5,
        max_wheel_speed=9.7,  # optional: rad/s
        encoder_channels = (0x40 , 0x41)
    ):
        # --- Geometry ---
        self.wheel_radius = wheel_radius
        self.half_wheelbase = half_wheelbase
        self.pulley_ratio = pulley_ratio

        # Other
        
        self.bus = i2c_bus if not i2c_bus is None else smbus2.SMBus(1) # declare the i2c bus object
        self.motors = (
            HBridgePWMMotor((17,18)),
            HBridgePWMMotor((22,23))
        )
        self.motor_encoders = (
            Encoder(encoder_channels[0],self.bus, encoder_bits), 
            Encoder(encoder_channels[1],self.bus, encoder_bits)
        )
        self.max_wheel_speed = max_wheel_speed
        

    def readShaftsRad(self):
        return np.array([2*pi-self.motor_encoders[0].readRad(), self.motor_encoders[1].readRad()])

    def readShaftsDeg(self):
        return np.array([360-self.motor_encoders[0].readDeg(), self.motor_encoders[1].readDeg()])

    def checkEncoders(self):
        return np.array([self.motor_encoders[0].getStaleness(), self.motor_encoders[1].getStaleness()])
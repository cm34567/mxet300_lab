# Program to read left and right encoders on SCUTTLE.
# Left has address 40 and right has 41, and the encoders are placed on
# the motor shafts, so readings indicate movement before pulley ratio
# is considered.
# This code runs on SCUTTLE with rasPi setup. (last updated 2020.11)

# Import external libraries
import smbus2       # a Python package to communicate over i2c
import numpy as np  # use numpy to build the angles array
import time         # for keeping time
from math import pi

class Encoder:
    def __init__(self, channel, i2c_bus, encoder_bits):
        self.bus = i2c_bus      # declare the i2c bus object
        self.channel = channel         # encoder i2c address for RIGHT motor (this encoder has A1 pin pulled high)
        self.max_encoder = 2**encoder_bits
        self.last_pos = 0
        self.staleness = 0

        self._binToDeg = 360/self.max_encoder
        self._binToRad = 2*pi/self.max_encoder

    def _readConvert(self, converter=(lambda x:x)):  # return a reading for an encoder in degrees (motor shaft angle)
        try:
            twoByteReading = self.bus.read_i2c_block_data(self.channel, 0xFE, 2)    # request data from registers 0xFE & 0xFF of the encoder. Approx 700 microseconds.
            self.last_pos = (twoByteReading[0] << 6) | (twoByteReading[1])         # remove unused bits 6 & 7 from byte 0xFF creating 14 bit value
        except:
            print("Encoder reading failed.")                                        # indicate a failed reading
            self.staleness += 1

        self.staleness = 0
        return converter(self.last_pos) # Return last known value

    def getStaleness(self):
        return self.staleness

    def readDeg(self):  # return a reading for an encoder in degrees (motor shaft angle)
        return self._readConvert(lambda binPos: binPos*self._binToDeg)

    def readRad(self):  # return a reading for an encoder in degrees (motor shaft angle)
        return self._readConvert(lambda binPos: binPos*self._binToRad)

# THIS LOOP RUNS IF THE PROGRAM IS CALLED DIRECTLY
if __name__ == "__main__":
    print("Testing Encoders")
    while True:
      encValues = readShaftPositions() # read the values.  Reading will only change if motor pulley moves
      # round the values and print them separated by a tab
      print("Left: ", encValues[0], "\t","Right: ", encValues[1])
      time.sleep(0.5)

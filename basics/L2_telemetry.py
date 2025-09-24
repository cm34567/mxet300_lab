import board
import digitalio
import netifaces as ni
from time import sleep
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
from adafruit_ina219 import INA219

import L1_log as log 
import L1_ina as ina 

# Set up the INA219 sensor

i2c = board.I2C()
ina219 = INA219(i2c, 0x44)

while (1):
    log.tmpFile(ina.readVolts(), "voltage")
    print(ina.readVolts())
    sleep(0.5)



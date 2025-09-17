import board
import digitalio
import netifaces as ni
import L2_compass_heading as cmp
import L1_log as log
import L1_ina as ina
from time import sleep 
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
from adafruit_ina219 import INA219

i2c = board.I2C()
ina219 = INA219(i2c, 0x44)

def log_heading_ang():
   heading_ang = round(cmp.get_heading(),2)

   return heading_ang

while True:
    sleep(1)
    if ((log_heading_ang()) < 22.5) and ((log_heading_ang()) > -22.5):
      print("north")
      log.tmpFile((log_heading_ang()), "degrees")
      log.stringTmpFile("north","direction")
    elif ((log_heading_ang()) < 67.5) and ((log_heading_ang()) > 22.5):
      print ("north east")
      log.tmpFile((log_heading_ang()), "degrees")
      log.stringTmpFile("north east","direction")
    elif ((log_heading_ang()) < 112.5) and ((log_heading_ang()) > 67.5):
      print ("east")
      log.tmpFile((log_heading_ang()), "degrees")
      log.stringTmpFile("east","direction")
    elif ((log_heading_ang()) < 157.5) and ((log_heading_ang()) > 112.5):
      print ("south east")
      log.tmpFile((log_heading_ang()), "degrees")
      log.stringTmpFile("south east","direction")
    elif ((log_heading_ang()) < -157.5) and ((log_heading_ang()) > 157.5):
      print ("south")
      log.tmpFile((log_heading_ang()), "degrees")
      log.stringTmpFile("south","direction")
    elif ((log_heading_ang()) < -112.5) and ((log_heading_ang()) > -157.5):
      print ("south west")
      log.tmpFile((log_heading_ang()), "degrees")
      log.stringTmpFile("south west","direction")
    elif ((log_heading_ang()) < -67.5) and ((log_heading_ang()) > -122.5):
      print ("west")
      log.tmpFile((log_heading_ang()), "degrees")
      log.stringTmpFile("west","direction")
    elif ((log_heading_ang()) < -22.5) and ((log_heading_ang()) > -67.5):
      print ("north west")
      log.tmpFile((log_heading_ang()), "degrees")
      log.stringTmpFile("north west","direction")
    
    log.tmpFile(ina.readVolts(), "voltage")



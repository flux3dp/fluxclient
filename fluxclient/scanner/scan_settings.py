#!/usr/bin/env python3

import math
import struct
import sys
import os
from time import time
from datetime import datetime


from datetime import datetime
name = datetime.fromtimestamp(time()).strftime('%Y-%m-%d-%H-%M-%S/')  # name
scan_step = 600  # steps
camera_port = 0
# serial_port = 'COM9'
serial_port = '/dev/tty.usbmodem1411'
serial_port = '/dev/cu.usbmodem14141'
throw = 3
sleeping_time = 0.07  # wait for some time to make sure hardware reacted
# sleeping_time = 0  # wait for some time to make sure hardware reacted
real_world = 1
theta_a = math.pi / 6 * 2 / 2  # rad between center and laser
img_width = 1024
img_height = 768
store_img = True  # whether or not store the img


sensorWidth = 3.67
sensorHeight = 2.74
focalLength = 3.6

cameraX = 0.0
cameraY = 170.0
cameraZ = 90

laserX_L = -52.0
laserY_L = 77.0
laserZ_L = 133

laserX_R = 52.0
laserY_R = 77.0
laserZ_R = 133

# cameraX = 0.0
# cameraY = 120.0
# cameraZ = 150.0

# laserX_L = -80.0
# laserY_L = 120.0
# laserZ_L = 120

# laserX_R = 80.0
# laserY_R = 120.0
# laserZ_R = 120


if "-n" in sys.argv:
    name = "./" + sys.argv[sys.argv.index("-n") + 1] + "/"
    if not os.path.exists(name):
        os.mkdir(name)
    # print "store into"+name

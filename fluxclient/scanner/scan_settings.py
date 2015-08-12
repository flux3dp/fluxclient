#!/usr/bin/env python3
import math
import struct
import sys
import os
from time import time
from datetime import datetime

name = datetime.fromtimestamp(time()).strftime('%Y-%m-%d-%H-%M-%S/')  # name
scan_step = 400  # steps
camera_port = 0
# serial_port = 'COM9'
serial_port = '/dev/tty.usbmodem1411'
serial_port = '/dev/cu.usbmodem14141'
throw = 3
sleeping_time = 0.07  # wait for some time to make sure hardware reacted
# sleeping_time = 0  # wait for some time to make sure hardware reacted
real_world = 1
theta_a = math.pi / 6 * 2 / 2  # radius between center and laser
img_width = 640
img_height = 480
store_img = True  # whether or not store the img


sensorWidth = 3.67
sensorHeight = 2.74
focalLength = 3.6


cameraX = 0.0
cameraY = 22.28
cameraZ = 174.70

laserX_L = -53.61
laserY_L = 31.62
laserZ_L = 76.47

laserX_R = 53.61
laserY_R = 31.62
laserZ_R = 76.47

# ######### mock up 1, hand measure? #########
# cameraX = 0.0
# cameraY = 90
# cameraZ = 170.0

# laserX_L = -52.0
# laserY_L = 133.
# laserZ_L = 77.0

# laserX_R = 52.0
# laserY_R = 133.
# laserZ_R = 77.0

# ######### prototype #########
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

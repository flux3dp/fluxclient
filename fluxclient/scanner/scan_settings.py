#!/usr/bin/env python3
import math

scan_step = 400  # steps
theta_a = math.pi / 6 * 2 / 2  # radius between center and laser
img_width = 640
img_height = 480

sensorWidth = 3.67
sensorHeight = 2.74
focalLength = 3.6

# ######### mockup 2, measure by solidwork###
cameraX = 0.0
cameraY = 22.28
cameraZ = 174.70

laserX_L = -53.61
laserY_L = 31.62
laserZ_L = 76.47

laserX_R = 53.61
laserY_R = 31.62
laserZ_R = 76.47

# ######### mockup 1, hand measured #########
# cameraX = 0.0
# cameraY = 90
# cameraZ = 170.0

# laserX_L = -52.0
# laserY_L = 133.
# laserZ_L = 77.0

# laserX_R = 52.0
# laserY_R = 133.
# laserZ_R = 77.0
##############################################

# ######### prototype, hand measured #########
# cameraX = 0.0
# cameraY = 120.0
# cameraZ = 150.0

# laserX_L = -80.0
# laserY_L = 120.0
# laserZ_L = 120

# laserX_R = 80.0
# laserY_R = 120.0
# laserZ_R = 120
##############################################

SOR_neighbors = 50
NE_radius = 10

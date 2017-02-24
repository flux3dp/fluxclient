#!/usr/bin/env python3
from math import pi, atan


class ScanSetting(object):
    """docstring for ScanSetting"""
    def __init__(self):
        super(ScanSetting, self).__init__()
        # for scan
        self.scan_step = 400  # steps
        self.theta_a = pi / 6  # radius between center and laser
        
        self.img_width = 640
        self.img_height = 480

        self.sensorWidth = 3.67
        self.sensorHeight = 2.74 + 0.08
        self.focalLength = 3.6

        # ######### mockup 2, measure by solidwork###
        self.cab_m = self.img_width / 2
        self.cab_l = self.img_width / 2
        self.cab_r = self.img_width / 2

        self.cameraX = 0.0
        self.cameraY = 22.28 + 8
        self.cameraZ = -174.70

        self.laserX_L = -53.61
        self.laserY_L = 31.62
        self.laserZ_L = -76.47

        self.laserX_R = 53.61
        self.laserY_R = 31.62
        self.laserZ_R = -76.47

        self.theta_a = atan(self.laserX_L / self.laserZ_L)

        self.MAXLaserRange = 65
        self.LaserRangeMergeDistance = 65
        self.MINLaserRange = 3
        self.MagnitudeThreshold = 3
        self.LLaserAdjustment = 0
        self.RLaserAdjustment = 0

        # for modeling
        self.NoiseNeighbors = 50
        self.NeighborhoodDistance = 10
        self.SegmentationDistance = 2
        self.CloseBottom = -1000
        self.CloseTop = 1000

    def set_camera(self, width=720, height=1280):
        """Set camera parameters"""
        self.img_width = width
        self.img_height = height
        self.cab_m = self.img_width / 2
        self.cab_l = self.img_width / 2
        self.cab_r = self.img_width / 2

        if self.img_width >= 720:
            self.sensorWidth = 3.57 #3.67
            self.sensorHeight = 6.35 # 2.74 + 0.08
        else:
            self.sensorWidth = 3.67
            self.sensorHeight = 2.74 + 0.08

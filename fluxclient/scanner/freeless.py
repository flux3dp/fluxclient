#!/usr/bin/env python3
import sys
import time
import math

import numpy as np
from PIL import Image

try:
    from . import scan_settings
    from .tools import write_stl, dot, normal, normalize, point_dis_sq
except:
    import scan_settings
    from tools import write_stl, dot, normal, normalize, point_dis_sq

NUM_LASER_RANGE_THRESHOLD = 3
RED_HUE_UPPER_THRESHOLD = 5


def pre_cut(img, x=0, y=0, w=None, h=None):
    return img[y: y + h, x: x + w]  # x, y, w, h


class freeless():

    '''
      freeless algorithm base on: http://www.freelss.org/
    '''

    def __init__(self, laserX, laserZ):
        '''
          m_maxLaserWidth, m_minLaserWidth: red dots width within this range will be considered valid laser points
          firstRowLaserCol : red dot location of previous scanning step, reference for next step
          RANGE_DISTANCE_THRESHOLD : two range will be merged if their distance is small than this variable
        '''
        self.m_laserRanges = []
        self.m_maxLaserWidth = 50
        self.m_minLaserWidth = 3
        self.MAX_MAGNITUDE_SQ = (255 * 3.0)  # doesn't really matter(just for scaling)
        self.m_laserMagnitudeThreshold = .8  # main threshold, diff value from 0.0 to 1.0
        self.firstRowLaserCol = 0.5 * scan_settings.img_width
        self.RANGE_DISTANCE_THRESHOLD = 2
        self.numSuspectedBadLaserLocations = 0
        self.results = []
        self.laser_plane = [[0, 0, 0], normalize([laserZ, 0, -1 * laserX])]
        self.place = {}

    def img_to_points(self, img_o, img_red, indices, step, side, cab_offset, clock=False):
        '''
        convert indices on the image into x-y-z-rgb points
        return  [
                                p1[x-coordinate, y-coord, z-coord, r, g, b, step],
                                p2[x-coordinate, y-coord, z-coord, r, g, b, step],
                                p3[x-coordinate, y-coord, z-coord, r, g, b, step],
                                  ...
                ]

        [WARNING] coordinate change here!!!!!!!!!!!
                  switch y, z
        '''
        points = []

        MAX_DIST_XZ_SQ = 70 ** 2
        PLATE_Y = -5.0
        MAX_DIST_Y = 100

        for y, x in indices:
            ray = self.calculateCameraRay(x, y)
            f, point = self.intersectLaserPlane(ray)

            if f:
                if point[0][0] ** 2 + point[0][2] ** 2 < MAX_DIST_XZ_SQ and point[0][1] >= PLATE_Y and point[0][1] < MAX_DIST_Y:
                    # add color
                    point.append([img_o[y][x - cab_offset][0], img_o[y][x - cab_offset][1], img_o[y][x + 10][2]])
                    point.append([x, y])

                    points.append(point)
                else:  # points that out of bounding cylinder
                    pass
                    # print point[0][1] >= 0.0, point[0][0] ** 2 + point[0][2] ** 2 < MAX_DIST_XZ_SQ,
                    # point[0][1] < MAX_DIST_Y, step

        # rotate
        clock = True
        if clock:
            step = - step

        theta = math.pi * 2 * step / scan_settings.scan_step

        c = math.cos(theta)
        s = math.sin(theta)

        new_points = []
        for p in points:
            # [WARNING] change here
            new_points.append([p[0][0] * c + p[0][2] * (-s), p[0][0] * s + p[0][2] * c, p[0][1], p[2][2], p[2][1], p[2][0], step, p[3][0], p[3][1]])
            # new_points.append([p[0][0] * c + p[0][2] * (-s), p[0][0] * s + p[0][2] * c, p[0][1], (step % 2) * 255, 255 - step / 400. * 255, 255 - step / 400. * 255, step, p[3][0], p[3][1]])

        return new_points

    def intersectLaserPlane(self, ray):
        '''

        '''
        # Reference: http://www.scratchapixel.com/lessons/3d-basic-lessons/lesson-7-intersecting-simple-shapes/ray-plane-and-ray-disk-intersection/
        # d = ((p0 - l0) * n) / (l * n)

        # If dn is close to 0 then they don't intersect.  This should never happen
        # print ray[1], self.laser_plane[1]
        denominator = dot(ray[1], self.laser_plane[1])
        # print denominator

        if abs(denominator) < 0.0000001:
            print('warning: < 0.0000001:', denominator, file=sys.stderr)
            return False, None

        v = [self.laser_plane[0][0] - ray[0][0], self.laser_plane[0]
             [1] - ray[0][1], self.laser_plane[0][2] - ray[0][2]]

        # v = [m_laserPlane.point.x - ray.origin.x, m_laserPlane.point.y - ray.origin.y, m_laserPlane.point.z - ray.origin.z]
        numerator = dot(v, self.laser_plane[1])
        d = float(numerator) / denominator
        if d < 0:
            print('warning: d < 0:')
            return False, None

        point = [[ray[0][0] + (ray[1][0] * d), ray[0][1] + (ray[1][1] * d), ray[0][2] + (ray[1][2] * d)]]
        point.append([scan_settings.laserX_L - point[0][0], scan_settings.laserY_L - point[0][1], scan_settings.laserZ_L - point[0][2]])
        # print point

        return True, point

    def calculateCameraRay(self, x, y):
        '''
          calculate a ray at x, y, z, direction from camera to x, y, z
          return ray : [[x,y,z], [direction_x, direction_y, direction_z]]
          warning coord change!!!!!!!!!!!
          in image:y goes down, in realworld :y goes up
        '''
        # if (x, y) in self.place:
        #     return self.place[(x, y)]

        # portion, We subtract by one because the image is 0 indexed
        x = float(x) / (scan_settings.img_width - 1)
        y = float(scan_settings.img_height - 1 - y) / (scan_settings.img_height - 1)

        # x = (x * scan_settings.sensorWidth) + scan_settings.cameraX - (scan_settings.sensorWidth * 0.5) + 0.125  # rule of thumb number
        # y = (y * scan_settings.sensorHeight) + scan_settings.cameraY - (scan_settings.sensorHeight * 0.5) + 0.31  # rule of thumb number
        x = (x * scan_settings.sensorWidth) + scan_settings.cameraX - (scan_settings.sensorWidth * 0.5)  # rule of thumb number
        y = (y * scan_settings.sensorHeight) + scan_settings.cameraY - (scan_settings.sensorHeight * 0.5)  # rule of thumb number
        z = scan_settings.cameraZ + scan_settings.focalLength

        ray = [[x, y, z], normalize([x - scan_settings.cameraX, y - scan_settings.cameraY, z - scan_settings.cameraZ])]
        # self.place[(x, y)] = ray
        return ray

    def writeTrianglesForColumn(self, lastFrame, currentFrame, tri):
        '''
        '''
        iCur = 0
        for iLst in range(len(lastFrame) - 1):
            l1 = lastFrame[iLst]
            l2 = lastFrame[iLst + 1]
            # If the current point is in range
            while iCur + 1 < len(currentFrame):
                c1 = currentFrame[iCur]
                c2 = currentFrame[iCur + 1]
                if check([l1[:3], c1[:3], c2[:3]]):
                    distSq1 = point_dis_sq(l1[:3], c2[:3])
                    distSq2 = point_dis_sq(l2[:3], c2[:3])
                    if distSq1 < distSq2:
                        tri.append([l1[:3], c1[:3], c2[:3]])
                    else:
                        if check([l2[:3], l1[:3], c1[:3]]):
                            tri.append([l2[:3], l1[:3], c1[:3]])
                        else:
                            iCur += 1
                        break
                else:
                    iCur += 1
                    break

                iCur += 1

        return tri

    def stl_writer(self, results, file_name):
        '''
        '''
        step = 0
        tri = []
        lastFrame = []
        # print len(results)
        while step < len(results):
            currentFrame = results[step][:]

            if step > 0:
                tri = self.writeTrianglesForColumn(
                    lastFrame, currentFrame, tri)

            elif True:  # (connectLastFrameToFirst)
                # Store the first frame for usage later on
                firstFrame = currentFrame[:]

            tmp = lastFrame
            lastFrame = currentFrame
            currentFrame = tmp
            step += 1

        if step > 0:
            self.writeTrianglesForColumn(lastFrame, firstFrame, tri)

        print ('tri:', len(tri), 'triangels')
        write_stl(tri, file_name)

    def subProcess(self, img1, img2, maxNumLocations=scan_settings.img_height):
        '''
          find out the location of the laser dots
          return a list of indices [[x,y], [x,y], [x,y]]
        '''
        laserLocations = []
        numMerged = 0
        prevLaserCol = self.firstRowLaserCol

        # diff two img, need set type as 'int' to avoid overflow in uint8
        d = abs((img1.astype(int)) - (img2.astype(int)))
        # squre each element
        d = d.astype(int)
        d = np.multiply(d, d)
        # sum up r, g, b diff number into one number
        d = np.sum(d, axis=2)

        # self.MAX_MAGNITUDE_SQ = (255. * 255.)
        # img1 = img1.astype(int)
        # img2 = img2.astype(int)

        # # 0.21 R + 0.72 G + 0.07 B
        # img1_gray = img1[:, :, 0] * 0.07 + img1[:, :, 1] * 0.72 + img1[:, :, 2] * 0.21
        # img2_gray = img2[:, :, 0] * 0.07 + img2[:, :, 1] * 0.72 + img2[:, :, 2] * 0.21
        # d = abs(img1_gray - img2_gray)
        # d = np.multiply(d, d)

        # some transform
        # self.MAX_MAGNITUDE_SQ = (255 * 3.0 * 255)
        # self.m_laserMagnitudeThreshold = 0.3
        mag = d / self.MAX_MAGNITUDE_SQ  # 0.0 ~ 1.0
        # print(mag, np.amax(mag), file=sys.stderr)
        # self.m_laserMagnitudeThreshold = .3

        for row in range(scan_settings.img_height):
            m_laserRanges = []
            # candidates, [ [starting index, ending index, middle point], ... ]
            m_laserRanges.append([-1, -1, None])

            for col in range(scan_settings.img_width):
                # diff value is bigger than threshold
                if mag[row][col] > self.m_laserMagnitudeThreshold:
                    # new candidate appear: first time > threshold, record as starting index
                    if m_laserRanges[-1][0] == -1:
                        m_laserRanges[-1][0] = col
                    # otherwise keep going
                    else:
                        pass

                # ending point of the candidate appear! (diff value is no longer bigger than threshold)
                elif m_laserRanges[-1][0] != -1:
                    laserWidth = col - m_laserRanges[-1][0]
                    # laser width should within the constrain
                    if laserWidth <= self.m_maxLaserWidth and laserWidth >= self.m_minLaserWidth:
                        wasMerged = False
                        # merge two candidate if they are very near
                        if len(m_laserRanges) > 1:
                            rangeDistance = m_laserRanges[-1][0] - m_laserRanges[-2][1]
                            if rangeDistance < self.RANGE_DISTANCE_THRESHOLD:
                                wasMerged = True
                                m_laserRanges[-2][1] = col
                                m_laserRanges[-2][2] = round((m_laserRanges[-2][0] + m_laserRanges[-2][1]) / 2.)
                                numMerged += 1
                                m_laserRanges[-1][0] = -1

                        if not wasMerged:
                            m_laserRanges[-1][1] = col
                            m_laserRanges[-1][2] = round((m_laserRanges[-1][0] + m_laserRanges[-1][1]) / 2.)

                            m_laserRanges.append([-1, -1, None])
                    # laser width is not within the constrain, reinitialize the point
                    else:
                        m_laserRanges[-1][0] = -1

            # if m_laserRanges[-1][0] != -1 and m_laserRanges[-1][0] != scan_settings.img_width - 1:
            #   print m_laserRanges
            #   input()

            # for each row, find out the best candidate
            if len(m_laserRanges) > 1:
                rangeChoice = self.detectBestLaserRange(m_laserRanges, prevLaserCol)

                prevLaserCol = m_laserRanges[rangeChoice][2]

                centerCol = self.detectLaserRangeCenter(m_laserRanges[rangeChoice], img1, img2, row)
                laserLocations.append([row, centerCol])

                # update self.firstRowLaserCol
                if len(laserLocations) == 1:
                    self.firstRowLaserCol = m_laserRanges[rangeChoice][0]

                # suspect bad laser
                if len(m_laserRanges) > NUM_LASER_RANGE_THRESHOLD:
                    self.numSuspectedBadLaserLocations += 1

        return laserLocations

    def detectBestLaserRange(self, laserRanges, prevLaserCol):
        '''
          determine which is the best candidate from laserRanges
          i.e. the nearest one to the previous row's
          return index of the best one
        '''
        bestRange_index = 0
        dis_best = abs(laserRanges[0][2] - prevLaserCol)
        for i in range(1, len(laserRanges) - 1):  # last one is [-1, -1, None]
            tmpdis = abs(laserRanges[i][2] - prevLaserCol)
            if tmpdis < dis_best:
                dis_best = tmpdis
                bestRange_index = i
        return bestRange_index

    def detectLaserRangeCenter(self, bestRange, img1, img2, row):
        '''
          find the Center of bestrange
          use Weighted arithmetic mean
          return int

          [TODO] in freelss/src/ImageProcessor.cpp ImageProcessor::detectLaserRangeCenter: two more center algorithm to try?
        '''
        d = abs((img1[row][bestRange[0]:bestRange[1]]).astype(int) - (img2[row][bestRange[0]:bestRange[1]]).astype(int))
        d = d.astype(int)
        d = np.multiply(d, d)
        d = np.sum(d, axis=1)

        total = sum(d)
        d = np.multiply(d, range(0, (bestRange[1] - bestRange[0])))
        d = sum(d)
        centerCol = bestRange[0] + round(d / float(total))

        return centerCol

if __name__ == '__main__':
    import subprocess
    from tools import write_pcd

    tmp = freeless(scan_settings.laserX_L, scan_settings.laserZ_L)
    im = np.array(Image.open('../../../rule.jpg'))
    print(im.shape)
    # self, img_o, img_red, indices, step, side, cab_offset, clock=False
    # a = tmp.img_to_points(im, None, [(i, 325) for i in range(200, 352)], 0, 'L', -10)
    a = tmp.img_to_points(im, None, [(i, 325) for i in range(84, 350)], 0, 'L', 10)
    a = tmp.img_to_points(im, None, [(i, 325) for i in range(149, 350)], 0, 'L', 10)
    # a += tmp.img_to_points(im, None, [(i, 325) for i in range(0, 352)], 0, 'L', -10)
    output = '../../../tmp.pcd'
    print(a[0])
    print(a[-1])
    print(a[0][2] - a[-1][2])
    write_pcd(a, '../../../tmp.pcd')
    subprocess.call(['python', '../../../3ds/3ds/PCDViewer/pcd_to_js.py', output], stdout=open('../../../3ds/3ds/PCDViewer/model.js', 'w'))

    # p = tmp.img_to_points(sys.argv[1])
    # scan_settings.write_pcd(p, sys.argv[2])

    # print >>sys.stderr, "\nError: python ./img_to_points.py [img location] [output file name]\n"

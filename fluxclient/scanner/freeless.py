import sys
import time
import math


import numpy

import scan_settings as utl


NUM_LASER_RANGE_THRESHOLD = 3
RED_HUE_UPPER_THRESHOLD = 5


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
        self.m_maxLaserWidth = 120 * 2
        self.m_minLaserWidth = 3
        self.MAX_MAGNITUDE_SQ = (255 * 255 * 3.0)
        self.m_laserMagnitudeThreshold = .8
        self.firstRowLaserCol = 0.5 * utl.img_width
        self.RANGE_DISTANCE_THRESHOLD = 2
        self.numSuspectedBadLaserLocations = 0
        self.results = []
        self.laser_plane = [[0, 0, 0], utl.normalize([laserZ, 0, -1 * laserX])]

    def img_to_points(self, img_o, img_red, indices, step, side, clock=False):
        '''
          convert indices on the image into x-y-z-rgb points
          return  [
                                p1[x-coordinate, y-coord, z-coord, b, g, r],
                                p2[x-coordinate, y-coord, z-coord, b, g, r],
                                p3[x-coordinate, y-coord, z-coord, b, g, r],
                                  ...
                  ]
        '''
        points = []
        MAX_DIST_XZ_SQ = 250 ** 2
        MAX_DIST_Y = 999999

        for y, x in indices:
            ray = self.calculateCameraRay(x, y)
            f, point = self.intersectLaserPlane(ray)

            if f:
                distXZSq = math.sqrt(point[0][0] ** 2 + point[0][2] ** 2)

                if point[0][1] >= 0.0 and distXZSq < MAX_DIST_XZ_SQ and point[0][1] < MAX_DIST_Y:
                    point.append(
                        [img_o[y][x][0], img_o[y][x][1], img_o[y][x][2]])
                    points.append(point)
                else:
                    pass
                    # print point[0][1] >= 0.0, distXZSq < MAX_DIST_XZ_SQ,
                    # point[0][1] < MAX_DIST_Y, step

        # rotate
        clock = True
        if clock:
            step = - step

        if side == 'L':
            theta = math.pi * 2 * step / utl.scan_step
            # + utl.theta_a
        elif side == 'R':
            theta = math.pi * 2 * step / utl.scan_step
            # - utl.theta_a
        else:
            print ('shouldn\'t happen, input=' + side)

        c = math.cos(theta)
        s = math.sin(theta)

        for p in points:
            tmp1 = p[0][0] * c + p[0][2] * (-s)
            tmp2 = p[0][0] * s + p[0][2] * c
            p[0][0] = tmp1
            p[0][2] = tmp2

        # points = [[p[0][0] * 10, p[0][2] * 10, p[0][1] * 10, p[2][0], p[2][1], p[2][2]] for p in points]
        points = [[p[0][0], p[0][2], p[0][1] * 1.2, p[2][0], p[2][1], p[2][2]]
                  for p in points]

        return points

    def intersectLaserPlane(self, ray):
        '''

        '''
        # Reference: http://www.scratchapixel.com/lessons/3d-basic-lessons/lesson-7-intersecting-simple-shapes/ray-plane-and-ray-disk-intersection/
        # d = ((p0 - l0) * n) / (l * n)

        # If dn is close to 0 then they don't intersect.  This should never happen
        # print ray[1], self.laser_plane[1]
        denominator = utl.dot(ray[1], self.laser_plane[1])
        # print denominator

        if abs(denominator) < 0.0000001:
            print ('warning: < 0.0000001:', denominator)
            return False, None

        v = [self.laser_plane[0][0] - ray[0][0], self.laser_plane[0]
             [1] - ray[0][1], self.laser_plane[0][2] - ray[0][2]]

        # v = [m_laserPlane.point.x - ray.origin.x, m_laserPlane.point.y - ray.origin.y, m_laserPlane.point.z - ray.origin.z]
        numerator = utl.dot(v, self.laser_plane[1])
        d = float(numerator) / denominator
        if d < 0:
            # print 'warning: d < 0:', ray
            return False, None

        point = [[ray[0][
            0] + (ray[1][0] * d), ray[0][1] + (ray[1][1] * d), ray[0][2] + (ray[1][2] * d)]]
        point.append([utl.laserX_L - point[0][0],
                      utl.laserY_L - point[0][1], utl.laserZ_L - point[0][2]])
        # print point

        return True, point

    def calculateCameraRay(self, x, y):
        '''
          calculate a ray at x, y, z, direction from camera to x, y, z
          return ray : [[x,y,z], [direction_x, direction_y, direction_z]]
          warning coord change!!!!!!!!!!!
        '''
        # if (x,y) in self.place.keys():
        # return self.place[(x, y)]

        # else:
        # portion, We subtract by one because the image is 0 indexed
        x = float(x) / (utl.img_width - 1)
        y = float(utl.img_height - y) / (utl.img_height - 1)

        x = (x * utl.sensorWidth) + utl.cameraX - (utl.sensorWidth * 0.5)
        y = (y * utl.sensorHeight) + utl.cameraY - (utl.sensorHeight * 0.5)
        z = utl.cameraZ - utl.focalLength

        ray = [[x, y, z], utl.normalize(
            [x - utl.cameraX, y - utl.cameraY, z - utl.cameraZ])]
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
                if utl.check([l1[:3], c1[:3], c2[:3]]):
                    distSq1 = utl.point_dis_sq(l1[:3], c2[:3])
                    distSq2 = utl.point_dis_sq(l2[:3], c2[:3])
                    if distSq1 < distSq2:
                        tri.append([l1[:3], c1[:3], c2[:3]])
                    else:
                        if utl.check([l2[:3], l1[:3], c1[:3]]):
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
        utl.write_stl(tri, file_name)

    def subProcess(self, img1, img2, maxNumLocations=utl.img_height):
        '''
          find out the location of the laser dots
          return a list of indices [[x,y], [x,y], [x,y]]
        '''
        laserLocations = []
        numMerged = 0
        prevLaserCol = self.firstRowLaserCol

        # diff two img

        d = abs(img1 - img2)
        d = d.astype(int)
        # squre each element
        d = numpy.multiply(d, d)
        # sum up r, g, b into one number
        d = numpy.sum(d, axis=2)
        # some transform
        mag = 255.0 * d / self.MAX_MAGNITUDE_SQ

        for row in range(utl.img_height):
            m_laserRanges = []
            # candidates, [ [starting index, ending index, middle point], ... ]
            m_laserRanges.append([-1, -1, None])

            for col in range(utl.img_width):
                # diff value is bigger than threshold
                if mag[row][col] > self.m_laserMagnitudeThreshold:
                    # store the beginning, if it's a new candidate, keep going
                    # otherwise
                    if m_laserRanges[-1][0] == -1:
                        m_laserRanges[-1][0] = col

                # ending point of the candidate appear! (diff value is no
                # longer bigger than threshold)
                elif m_laserRanges[-1][0] != -1:
                    laserWidth = col - m_laserRanges[-1][0]
                    # laser width should within the constrain
                    if laserWidth <= self.m_maxLaserWidth and laserWidth >= self.m_minLaserWidth:
                        wasMerged = False
                        # merge two candidate if they are very near
                        if len(m_laserRanges) > 1:
                            rangeDistance = m_laserRanges[-
                                                          1][0] - m_laserRanges[-2][1]
                            if rangeDistance < self.RANGE_DISTANCE_THRESHOLD:
                                wasMerged = True
                                m_laserRanges[-2][1] = col
                                m_laserRanges[-2][2] = round(
                                    (m_laserRanges[-2][0] + m_laserRanges[-2][1]) / 2.)
                                numMerged += 1
                                m_laserRanges[-1][0] = -1

                        if wasMerged is False:
                            m_laserRanges[-1][1] = col
                            m_laserRanges[-1][2] = round(
                                (m_laserRanges[-1][0] + m_laserRanges[-1][1]) / 2.)

                            m_laserRanges.append([-1, -1, None])
                    # laser width is not within the constrain, reinitialize the
                    # point
                    else:
                        m_laserRanges[-1][0] = -1

            # if m_laserRanges[-1][0] != -1 and m_laserRanges[-1][0] != utl.img_width - 1:
            #   print m_laserRanges
            #   input()

            # find out the best candidate
            if len(m_laserRanges) > 1:
                rangeChoice = self.detectBestLaserRange(
                    m_laserRanges, prevLaserCol)

                prevLaserCol = m_laserRanges[rangeChoice][2]

                centerCol = self.detectLaserRangeCenter(
                    m_laserRanges[rangeChoice], img1, img2, row)
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
          return int
        '''
        bestRange_index = 0
        dis_best = abs(laserRanges[0][2] - prevLaserCol)
        for i in range(1, len(laserRanges) - 1):
            tmpdis = abs(laserRanges[i][2] - prevLaserCol)
            if tmpdis < dis_best:
                dis_best = tmpdis
                bestRange_index = i
        return bestRange_index

    def detectLaserRangeCenter(self, bestRange, img1, img2, row):
        '''
          find the Center of bestrange
          return int
        '''
        d = abs(
            img1[row][bestRange[0]:bestRange[1]] - img2[row][bestRange[0]:bestRange[1]])
        d = d.astype(int)
        d = numpy.multiply(d, d)
        d = numpy.sum(d, axis=1)

        total = sum(d)
        d = numpy.multiply(d, range(0, (bestRange[1] - bestRange[0])))
        d = sum(d)
        centerCol = bestRange[0] + round(d / float(total))

        return centerCol

if __name__ == '__main__':
    tmp = freeless()
    # p = tmp.img_to_points(sys.argv[1])
    # utl.pcd_write(p, sys.argv[2])

    # print >>sys.stderr, "\nError: python ./img_to_points.py [img location] [output file name]\n"

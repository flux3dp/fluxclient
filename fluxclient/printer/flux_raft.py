#!/usr/bin/env python3
import sys
import time
import math
import re
import os
from queue import Queue
from math import ceil, sqrt

import numpy as np


class Raft():
    def __init__(self):
        self.move_re = re.compile(r"G1 ?([XYZEF] ?\-?\d+\.?\d+)?([XYZEF] ?\-?\d+\.?\d+)?([XYZEF] ?\-?\d+\.?\d+)?([XYZEF] ?\-?\d+\.?\d+)?")
        self.axis_re = re.compile(r"([XYZEF]) ?(\-?\d+\.?\d+)")
        self.extrusion = 0.0759617 * 0.8
        self.line_width = 0.4
        self.resolution = 0.3
        self.first_layer = 0.3
        self.layer_height = 0.2
        self.count = 3
        self.z_space = 0.12
        self.width = ceil(172 / self.resolution)
        self.grid = [[]]
        self.gcode = []

    def print_start_gcode(self):
        code = """M107 ; disable fan
M104 S220 ; set temperature
G28 ; home all axes
G1 Z5 F5000 ; lift nozzle

M109 S200 ; wait for temperature to be reached
G21 ; set units to millimeters
G90 ; use absolute coordinates
M82 ; use absolute distances for extrusion
G92 E0 ; reset extrusion distance
G1 Z0.3 F9000.000 ; move to next layer (0)
G1 E-2.00000 F2400.00000 ; retract
G92E0
G1 F1800
"""
        print(code, file=self.output_stream)

    def process(self, gcode, debug=False):
        #Process all gcode on first few layers, and fill the grid, skip skirt...
        self.grid = self.fill_grid(gcode)
        #Select all connected islands, find the edge points at each one
        islands = self.find_islands()
        print(";Islands found %d" % len(islands), file=sys.stderr)
        #Print start gcode
        self.print_start_gcode()
        #Print raft gcode
        raft_gcode = self.generate_gcode(islands)
        #Debug output
        if debug and os.environ.get("flux_debug") == '1':
            self.output_grid()
        #Print other gcode ( uplift Z by elf.count*self.layer_height+self.z_space )
        skip = 15
        for line in gcode:
            #Skip first 15 lines
            if skip > 0:
                skip = skip - 1
                continue
            #Lift Z
            line = re.sub("Z ?(-?[\d.]+)", self.z_rep, line)
            print(line, end="", file=self.output_stream)

    def z_rep(self, matchobj):
        z_old = float(matchobj.group(1))
        return "Z" + str(z_old + self.count * self.layer_height + self.z_space)

    def generate_gcode(self, islands):
        island_id = 0
        for island in islands:
            island_id = island_id + 1
            print(";Island #%d" % island_id, file=self.output_stream)
            edge = island

            x = edge[0][0]
            y = edge[0][1]
            last_point = [0, 0]

            sorted_edge = [[x, y]]

            #Traverse the edge
            while len(edge) > 0:
                if self.is_edge(edge, x - 1, y):
                    x = x - 1
                elif self.is_edge(edge, x + 1, y):
                    x = x + 1
                elif self.is_edge(edge, x, y - 1):
                    y = y - 1
                elif self.is_edge(edge, x, y + 1):
                    y = y + 1
                elif self.is_edge(edge, x - 1, y - 1):
                    (x, y) = (x - 1, y - 1)
                elif self.is_edge(edge, x + 1, y + 1):
                    (x, y) = (x + 1, y + 1)
                elif self.is_edge(edge, x + 1, y - 1):
                    (x, y) = (x + 1, y - 1)
                elif self.is_edge(edge, x - 1, y + 1):
                    (x, y) = (x - 1, y + 1)

                if last_point[0] == x and last_point[1] == y:
                    x = edge[0][0]
                    y = edge[0][1]
                    continue
                else:
                    edge.remove([x, y])

                self.grid[x][y] = 4
                last_point = [x, y]
                sorted_edge.append([x, y])

            #Outline of raft
            print("G92 E0", file=self.output_stream)
            print("G1 Z%lf" % self.first_layer, file=self.output_stream)

            extruded = 0
            (x_min, y_min, x_max, y_max) = (999999, 999999, -1, -1)
            last_point = [0, 0]
            for pt in sorted_edge:
                if x_min > pt[0]:
                    x_min = pt[0]
                if x_max < pt[0]:
                    x_max = pt[0]
                if y_min > pt[1]:
                    y_min = pt[1]
                if y_max < pt[1]:
                    y_max = pt[1]

                x = self.m2g(pt[0])
                y = self.m2g(pt[1])

                if last_point[0] == 0 and last_point[1] == 0:
                    last_point = [x, y]

                e = self.dist(last_point[0], last_point[1], x, y) * self.extrusion
                extruded = extruded + e

                last_point = [x, y]
                print("G1X%lfY%lfE%lf" % (x, y, extruded), file=self.output_stream)

            #Infill of raft
            horizontal_lines = abs(ceil((y_max - y_min) * self.resolution / self.line_width))
            vertical_lines = abs(ceil((x_max - x_min) * self.resolution / self.line_width))

            print("Lines / Horizontal %lf Vertical %lf" % (horizontal_lines, vertical_lines), file=sys.stderr)
            print("Xmin %lf Xmax %lf Ymin %lf Ymax %lf" % (x_min, x_max, y_min, y_max), file=sys.stderr)
            width = len(self.grid)

            for l in range(0, self.count):
                print("G1 Z%lf" % (self.first_layer + l * self.layer_height), file=self.output_stream)
                if l % 2 == 0:
                    for r in range(0, horizontal_lines):
                        y = self.g2m(self.m2g(y_min) + self.line_width * r)
                        range_of_x = range(0, width)
                        if r % 2 == 1:
                            range_of_x = reversed(range_of_x)

                        inside = False
                        fill_start = 0
                        for x in range_of_x:
                            if self.grid[x][y] > 0 and not inside:
                                inside = True
                                fill_start = self.m2g(x)
                            elif self.grid[x][y] == 0 and inside:
                                e = abs(self.m2g(x) - fill_start) * self.extrusion
                                extruded = extruded + e

                                print("G1 X%lf Y%lf ; H line" % (fill_start, self.m2g(y)), file=self.output_stream)
                                print("G1 X%lf Y%lf E%lf" % (self.m2g(x), self.m2g(y), extruded), file=self.output_stream)
                                inside = False
                else:
                    for r in range(0, vertical_lines):
                        x = self.g2m(self.m2g(x_min) + self.line_width * r)
                        range_of_y = range(0, width)
                        if r % 2 == 1:
                            range_of_y = reversed(range_of_y)

                        inside = False
                        fill_start = 0
                        for y in range_of_y:
                            if self.grid[x][y] > 0 and not inside:
                                inside = True
                                fill_start = self.m2g(y)
                            elif self.grid[x][y] == 0 and inside:
                                e = abs(self.m2g(y) - fill_start) * self.extrusion
                                extruded = extruded + e

                                print("G1 X%lf Y%lf ; V line" % (self.m2g(x), fill_start), file=self.output_stream)
                                print("G1 X%lf Y%lf E%lf" % (self.m2g(x), self.m2g(y), extruded), file=self.output_stream)
                                inside = False

    def is_edge(self, edge, x, y):
        if [x, y] in edge and self.check_grid(x, y) == 3:
            return True
        else:
            return False

    def dist(self, x, y, x2, y2):
        return sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2))

    def m2g(self, val):
        return (val - ceil(86 / self.resolution)) * self.resolution

    def g2m(self, val):
        return round(val / self.resolution) + ceil(86 / self.resolution)

    def check_grid(self, x, y):
        if x >= 0 and y >= 0 and x < self.width and y < self.width:
            return self.grid[x][y]
        return 0

    def fill_grid(self, gcode):
        self.grid = np.zeros((ceil(172 / self.resolution), ceil(172 / self.resolution)))
        self.width = ceil(172 / self.resolution)
        print("Grid size %d^2" % len(self.grid), file=sys.stderr)
        x = y = z = 0
        expansion = self.expansion / self.resolution
        last_point = [0, 0]
        min_division = self.resolution * expansion / 10.0
        for ln in range(0, len(gcode)):
            line = gcode[ln]
            if "skirt" in line:
                continue
            if z > 2:
                print("Gcode parsing end", file=sys.stderr)
                break
            if self.move_re.match(line):
                for (axis, number) in self.axis_re.findall(line):
                    if axis == 'X':
                        x = float(number)
                    if axis == 'Y':
                        y = float(number)
                    if axis == 'Z':
                        z = float(number)

                    if (x - last_point[0]) * (x - last_point[0]) + (y - last_point[1]) * (y - last_point[1]) > min_division * min_division:
                        denom = int(ceil(sqrt((x - last_point[0]) * (x - last_point[0]) + (y - last_point[1]) * (y - last_point[1])) / min_division))
                        for numer in range(0, denom + 1):
                            lx = last_point[0] + (x - last_point[0]) * numer / denom
                            ly = last_point[1] + (y - last_point[1]) * numer / denom
                            (rx, ry) = (self.g2m(lx), self.g2m(ly))
                            self.fill_circle(rx, ry, 0, 0, expansion * expansion)
                    else:
                        (rx, ry) = (self.g2m(x), self.g2m(y))
                        self.fill_circle(rx, ry, 0, 0, expansion * expansion)

                    last_point = [x, y]

        return self.grid

    def fill_circle(self, rx, ry, x, y, r):
        Q = Queue()
        Q.put([x, y])
        while not Q.empty():
            n = Q.get()
            (x, y) = n
            if x * x + y * y < r:
                if rx + x >= 0 and ry + y >= 0 and rx + x < self.width and ry + y < self.width and self.grid[rx + x][ry + y] == 0:
                    self.grid[rx + x][ry + y] = 1
                    Q.put([x - 1, y])
                    Q.put([x + 1, y])
                    Q.put([x, y - 1])
                    Q.put([x, y + 1])

    #find all connected islands
    def find_islands(self):
        islands = []
        #iterate all points on grid
        for x in range(0, self.width):
            for y in range(0, self.width):
                if self.grid[x][y] == 1:
                    edge = []
                    self.find_all_connected_points(edge, x, y)
                    islands.append(edge)

        return islands

    #flood grouping
    def find_all_connected_points(self, edge, x, y):
        grid = self.grid
        width = len(self.grid)
        Q = Queue()
        if x >= width or y >= width or x < 0 or y < 0:
            return
        if self.grid[x][y] != 1:
            return

        Q.put([x, y])

        while not Q.empty():
            n = Q.get()
            (x, y) = n

            if self.grid[x][y] == 1:
                #Edge detection
                #If surrounded by filled area, then it's not
                if self.check_grid(x, y - 1) > 0 and self.check_grid(x - 1, y) > 0 and self.check_grid(x + 1, y) > 0 and self.check_grid(x, y + 1) > 0:
                    self.grid[x][y] = 2
                else:  # n is on edge
                    edge.append([x, y])
                    self.grid[x][y] = 3
                if self.check_grid(x - 1, y) == 1:
                    Q.put([x - 1, y])
                if self.check_grid(x + 1, y) == 1:
                    Q.put([x + 1, y])
                if self.check_grid(x, y - 1) == 1:
                    Q.put([x, y - 1])
                if self.check_grid(x, y + 1) == 1:
                    Q.put([x, y + 1])

    #debug tool
    def output_grid(self):
        from PIL import Image
        im = np.zeros((self.width, self.width, 3))
        for x in range(0, self.width):
            for y in range(0, self.width):
                if self.grid[x][y] == 1:
                    im[x][y] = [0, 255, 0]
                elif self.grid[x][y] == 2:
                    im[x][y] = [0, 0, 255]
                elif self.grid[x][y] == 3:
                    im[x][y] = [255, 0, 0]
                elif self.grid[x][y] == 4:
                    im[x][y] = [0, 128, 255]
                else:
                    im[x][y] = [255, 255, 255]

        Image.fromarray(im.astype(np.uint8)).save("grid.png")

    def main(self, gcode, output_stream, debug):
        if type(gcode) == str:
            with open(gcode) as f:
                gcode = f.readlines()
        self.output_stream = output_stream

        self.resolution = 0.5
        self.expansion = 10  # flux only param : equals to raft exapansion
        self.first_layer = 0.3  # equal to first layer height
        self.layer_height = 0.2  # equal to layer height
        self.count = 3  # equal to raft layers
        self.process(gcode)


if __name__ == '__main__':
    #Read gcode
    fname = sys.argv[1]
    with open(fname) as f:
        gcode = f.readlines()
    raft = Raft()
    raft.main(gcode, output_stream=sys.stdout, debug=True)

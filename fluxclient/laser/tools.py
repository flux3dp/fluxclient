# !/usr/bin/env python3
from math import sqrt


from laser_base import LaserBase


class Logo(LaserBase):
    """drawing FLUX Logo"""
    def __init__(self):
        super(Logo, self).__init__()
        self.ratio = .5
        self.ratio = 0.01

    def shift_move(self, x, y, shift_x=-50, shift_y=-50, speed=300):  # default 1
        x *= -1
        x += 100

        x += shift_x
        y += shift_y
        self.laser_speed = speed
        return self.moveTo(x, y)
        # print "G4 P10"

    def close_and_move_and_on(self, x, y, power=0):  # 0 max, 255 min
        gcode = []
        gcode += self.turnOff()
        gcode += self.shift_move(x, y, speed=400)
        gcode += self.turnOn()
        return gcode

    def gcode_generate(self):
        gcode = []
        gcode += self.header('Logo')

        # print "G4 P50"  # pause when starting

        # frame for align
        gcode += self.turnHalf()
        for i in range(1):
            gcode += self.close_and_move_and_on(50, 10, 253)
            gcode += self.shift_move(50, 10, speed=400)
            gcode += self.shift_move(7, 35, speed=400)
            gcode += self.shift_move(7, 85, speed=400)
            gcode += self.shift_move(50, 110, speed=400)
            gcode += self.shift_move(93.3, 85, speed=400)
            gcode += self.shift_move(93.3, 35, speed=400)
            gcode += self.shift_move(50, 10, speed=400)

        gcode += self.close_and_move_and_on(50, 10)
        gcode += self.shift_move(50, 10)
        gcode += self.shift_move(6.7, 35)
        gcode += self.shift_move(6.7, 85)
        gcode += self.shift_move(50, 110)
        gcode += self.shift_move(93.3, 85)
        gcode += self.shift_move(93.3, 35)
        gcode += self.shift_move(50, 10)

        gcode += self.close_and_move_and_on(50, 40)
        gcode += self.shift_move(50, 40)
        gcode += self.shift_move(32.7, 50)
        gcode += self.shift_move(32.7, 70)
        gcode += self.shift_move(50, 80)
        gcode += self.shift_move(67.3, 70)
        gcode += self.shift_move(67.3, 50)
        gcode += self.shift_move(50, 40)

        gcode += self.close_and_move_and_on(50, 40)
        gcode += self.shift_move(50, 40)
        # line that pass (50, 40) and (67.3,50) , line that pass (37.3,1) and (37.3, 2)
        gcode += self.shift_move(37.3, 32.6)
        gcode += self.shift_move(32.7 - (50 - 37.3), 50 - 7.4)
        # line that pass (50, 110) and (6.7,85) , line that pass (20,1) and (20, 2)
        gcode += self.shift_move(20, 92.7)

        gcode += self.close_and_move_and_on(32.7, 70)
        gcode += self.shift_move(32.7, 70)
        gcode += self.shift_move(32.7, 85)
        gcode += self.shift_move(63.0, 102.5)

        gcode += self.close_and_move_and_on(50, 80)
        gcode += self.shift_move(50, 80)
        # line that pass (50, 80) and (32.7,70) , line that pass (63,1) and (63, 2)
        gcode += self.shift_move(63.0, 87.5)
        gcode += self.shift_move(80.3, 77.4)
        gcode += self.shift_move(80.3, 27.5)

        gcode += self.close_and_move_and_on(67.3, 50)
        gcode += self.shift_move(67.3, 50)
        gcode += self.shift_move(67.3, 35)
        gcode += self.shift_move(37.3, 17.3)
        gcode += self.turnOff()
        gcode += ['G1 F5000 Z200']
        return "\n".join(gcode) + "\n"


class Grid(LaserBase):
    """Draw a grid on plate"""
    def __init__(self):
        super(Grid, self).__init__()

    def gcode_generate(self):
        gcode = []
        gcode += self.header('Grid')

        path = []
        path2 = []

        a = 0
        while a <= self.radius:
            b = sqrt(self.radius ** 2 - a ** 2)
            path.append([-b, a, b, a])
            path.append([-b, -a, b, -a])

            path2.append([a, -b, a, b])
            path2.append([-a, -b, -a, b])
            a += 10

        a = 5
        while a <= self.radius:
            b = sqrt(self.radius ** 2 - a ** 2)
            path.append([b, a, -b, a])
            path.append([b, -a, -b, -a])

            path2.append([a, b, a, -b])
            path2.append([-a, b, -a, -b])
            a += 10
        path.sort(key=lambda x: x[-1])
        path2.sort(key=lambda x: x[0])

        path += path2
        for i in path:
            gcode += self.closeTo(i[0], i[1])
            gcode += self.drawTo(i[2], i[3])
        return '\n'.join(gcode)


if __name__ == '__main__':
    # m_logo = Logo()
    # print(m_logo.gcode_generate())
    m_grid = Grid()
    print(m_grid.gcode_generate())
# !/usr/bin/env python3
from math import sqrt, pi, cos, sin
import sys

from fluxclient.laser.laser_base import LaserBase


class Matrix(object):
    """
    A matrix class that support matrix multipying
    """
    def __init__(self, num=3):
        self.M = [[0.0 for __ in range(3)] for _ in range(num)]

    def __getitem__(self, index):
        return self.M[index]

    def __mul__(self, other):
        tmp = Matrix()
        for i in range(len(tmp.M)):
            for j in range(len(tmp.M[0])):
                tmp[i][j] = self[i][j]

        if type(other) == float or type(other) == int:
            for i in range(len(tmp.M)):
                for j in range(len(tmp.M[0])):
                    tmp[i][j] *= other
        elif type(other) == type(tmp):
            for i in range(3):
                for j in range(3):
                    tmp[i][j] = sum(self.M[i][k] * other.M[k][j] for k in range(3))

        return tmp

    def __rmul__(self, other):
        if type(other) == int or type(other) == float:
            return self.__mul__(other)
        return other.__mul__(self)

    def __repr__(self):
        tmp = []
        # tmp.append('[')
        for i in range(3):
            for j in range(3):
                tmp.append(str(self.M[i][j]))
                tmp.append(' ')
            tmp.append('\n')
        # tmp.append(']')
        return ''.join(tmp)

    def set_I(self):
        for i in range(3):
            for j in range(3):
                self.M[i][j] = 0.
            self[i][i] = 1.
        return self


class Circle(LaserBase):
    """draw a circle"""
    def __init__(self):
        super(Circle, self).__init__()
        self.speed = 100

    def gcode_generate(self):
        gcode = []
        gcode += self.header('Circle')
        sample_n = 1000
        gcode += self.moveTo(self.radius, 0)
        for i in range(sample_n + 1):
            theta = (i / sample_n) * 2 * pi
            gcode += self.drawTo(self.radius * cos(theta), self.radius * sin(theta), speed=self.speed)
        gcode += self.turnOff()
        gcode += ['G28']
        return "\n".join(gcode) + "\n"


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
        gcode += self.turnTo()
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
        self.obj_height = 0.0  # change if needed

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
        gcode += self.turnOff()
        return '\n'.join(gcode)


class FindFocal(LaserBase):
    """find the facal length"""
    def __init__(self):
        super(FindFocal, self).__init__()

    def gcode_generate(self):
        gcode = self.header('FindFocal')
        focal_max = 10
        z_candidate = myrange(focal_max, 0.1, -0.02)
        tmp_i = 0
        tmp_y = 0
        step = 50
        length = 30
        while tmp_i + step < len(z_candidate):
            gcode += self.closeTo(-15, tmp_y)
            tmp = 0
            for z in z_candidate[tmp_i:tmp_i + step]:
                gcode += self.drawTo(-15 + 30 / step * tmp, tmp_y, z=z)
                gcode += self.drawTo(-15 + 30 / step * tmp, tmp_y + 1, z=z)
                gcode += self.drawTo(-15 + 30 / step * tmp, tmp_y, z=z)
                tmp += 1

            tmp_i += step
            tmp_y += 5
            print(tmp_y, file=sys.stderr)
        tmp = 0
        for z in z_candidate[tmp_i:]:
            gcode += self.drawTo(-15 + 30 / step * tmp, tmp_y, z=z)
            gcode += self.drawTo(-15 + 30 / step * tmp, tmp_y + 1, z=z)
            gcode += self.drawTo(-15 + 30 / step * tmp, tmp_y, z=z)
            tmp += 1
        gcode += self.turnOff()
        gcode += ['G28']
        return '\n'.join(gcode)


def myrange(start, end, step):
    """
    floating point range~
    """
    R = 1000.  # for floating point error
    start *= R
    end *= R
    step *= R
    output = []
    tmp = start
    if step >= 0:
        while tmp < end:
            output.append(tmp / R)
            tmp += step
    else:
        while tmp > end:
            output.append(tmp / R)
            tmp += step
    return output


def print_help():
    help = [
        '\tthis is a small tool for generating some simple gcode',
        '\tuse -logo   / -l to draw a FLUX logo',
        '\tuse -grid   / -g to draw grid on plate',
        '\tuse -circle / -c to draw grid on plate',
        '\tuse -help   / -h to see this help document'
    ]
    for i in help:
        print(i, file=sys.stderr)

if __name__ == '__main__':
    if '-logo' in sys.argv or '-l' in sys.argv:
        m_obj = Logo()
        print(m_obj.gcode_generate())
    elif '-grid' in sys.argv or '-g' in sys.argv:
        m_obj = Grid()
        print(m_obj.gcode_generate())
    elif '-circle' in sys.argv or '-c' in sys.argv:
        m_obj = Circle()
        print(m_obj.gcode_generate())
    elif '-find' in sys.argv or '-f' in sys.argv:
        pass
        # m_obj = FindFocal()
    else:
        print_help()

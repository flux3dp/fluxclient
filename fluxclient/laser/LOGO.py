# !/usr/bin/env python3
from laser_base import laser_base


class logo(laser_base):
    """drawing FLUX logo"""
    def __init__(self):
        super(logo, self).__init__()
        self.ratio = 4
        # self.ratio = 0.01

    def shift_move(self, x, y, shift_x=-50, shift_y=-50, speed=300):  # default 1
        x += shift_x
        y += shift_y
        return self.moveTo(x, y, speed)
        # print "G4 P10"

    def close_and_move_and_on(self, x, y, power=0):  # 0 max, 255 min
        gcode = []
        gcode += self.turnOff()
        gcode += self.shift_move(x, y, speed=400)
        gcode += self.turnOn()
        return gcode

    def gcode_generate(self):
        gcode = []
        gcode += self.header('LOGO')

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


if __name__ == '__main__':
    m_logo = logo()
    print(m_logo.gcode_generate())

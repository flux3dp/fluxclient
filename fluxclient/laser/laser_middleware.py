
from math import sin, cos

from .laser_base import LaserBase


class LaserMiddleware(LaserBase):
    """Laser Middleware is a middle class to override old behivor with new
       toolpath processor. All laser class shoule be refactor."""

    def turnOn(self, processor):  # noqa
        if self.laser_on is True:
            return []
        self.laser_on = True
        processor.set_toolhead_pwm(1)
        processor.sleep(0.002)

    def turnOff(self, processor):   # noqa
        if self.laser_on is False:
            return
        self.laser_on = False
        processor.set_toolhead_pwm(0)
        processor.sleep(0.002)

    def turnTo(self, processor, power=None):   # noqa
        """
        set laser power
        """
        if power is None:
            self.laser_on = True
            processor.set_toolhead_pwm(self.fram_power / 255)
            processor.sleep(0.002)

        elif power != 0:
            self.laser_on = True
            processor.set_toolhead_pwm(power / 255)
            processor.sleep(0.002)

        elif power == 0:
            return self.turnOff(processor)

    def moveTo(self, processor, x, y, speed=None, z=None, ending=None):  # noqa
        """
            apply global "rotation" and "scale"
            move to position x,y
        """

        x2 = (x * cos(self.rotation) - y * sin(self.rotation)) * self.ratio
        y2 = (x * sin(self.rotation) + y * cos(self.rotation)) * self.ratio

        x = x2
        y = y2

        if speed is None:
            speed = self.laser_speed

        if ending is None:
            if self.laser_on:
                processor.append_comment("Draw")
            else:
                processor.append_comment("Move")
        else:
            processor.append_comment(ending)

        self.current_x = x
        self.current_y = y
        if z is None:
            processor.moveto(feedrate=speed, x=x, y=y)
        else:
            processor.moveto(feedrate=speed, x=x, y=y, z=z)

    def drawTo(self, processor, x, y, speed=None, z=None):  # noqa
        """
            turn on, move to x, y

            draw to position x,y
        """
        self.turnOn(processor)

        if speed is None:
            self.moveTo(processor, x, y, self.laser_speed, z, ending='draw')
        else:
            self.moveTo(processor, x, y, speed, z, ending='draw')

    def closeTo(self, processor, x, y, speed=None, z=None):  # noqa
        """
            turn off, move to x, y
        """
        self.turnOff(processor)

        if speed is None:
            self.moveTo(processor, x, y, self.travel_speed, z)
        else:
            self.moveTo(processor, x, y, speed, z)

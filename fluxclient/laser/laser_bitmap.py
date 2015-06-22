# !/usr/bin/env python3
from math import pi, sin, cos

from fluxclient.laser.laser_base import LaserBase


class LaserBitmap(LaserBase):
    """
    laser_bitmap class:
      generate gcode base on given images
    """
    def __init__(self):
        super(LaserBitmap, self).__init__()
        self.reset()

    def reset(self):
        """
        reset laser_bitmap class
        """

        self.pixel_per_mm = 3  # how many pixel is 1 mm
        self.radius = 75  # laser max radius = 75
        # list holding current image
        self.image_map = [[255 for w in range(self.pixel_per_mm * self.radius * 2)] for h in range(self.pixel_per_mm * self.radius * 2)]
        self.edges = [0, len(self.image_map), 0, len(self.image_map[0])]  # up, down, left, right bound of the image

        self.rotation = 0  # general rotation for final gcode
        self.laser_on = False  # recording if laser is on

        # threshold, pixel on image_map darker than this will trigger laser, actually no use(only 255 or 0 on image_map)
        self.thres = 100
        self.ratio = 1.  # ratio to scale gcode, bot safe, shouldn't be any value except than 1

    #TODO:
    # OLD: def bitmap_moveTo(self, x, y):
    def bitmap_moveTo(self, x, y, speed=600):
        x = float(x) / self.pixel_per_mm - self.radius
        y = float(len(self.image_map) - y) / self.pixel_per_mm - self.radius
        return self.moveTo(x, y)

    def bitmap_drawTo(self, x, y):
        gcode = []
        gcode += self.turnOn()
        #TODO: 
        # OLD: gcode += self.bitmap_moveTo(x, y, speed)
        gcode += self.bitmap_moveTo(x, y, 600)
        gcode += self.turnOff()

        return gcode

    def rotate(self, x, y, rotation, cx=0., cy=0.):
        vx = (x - cx)
        vy = (y - cy)
        print ('f', vx, vy)
        x = cx + vx * cos(rotation) - vy * sin(rotation)
        y = cy + vx * sin(rotation) + vy * cos(rotation)
        return x, y

    def add_image(self, buffer_data, img_width, img_height, x1, y1, x2, y2, rotation, thres=255):
        """
        add image on top of current image i.e self.image_map
          parameters:
            buffer_data: image data in bytes array
            img_width, img_height: trivial
            x1, y1: absolute position of image's top-left corner
            x2, y2: absolute position of image's button_right corner
          return:
            None
        """
        pix = self.to_image(buffer_data, img_width, img_height)
        print("recv", img_width, img_height, x1, y1, x2, y2, rotation)

        # protocol fail
        print("corner:", x1, ',', y1, x2, ',', y2)
        real_width = float(x2 - x1)
        real_height = float(y1 - y2)

        # rotation center
        mx = (x1 + x2) / 2.
        my = (y1 + y2) / 2.
        print('c', mx, my)

        print (x1, y1)
        x1, y1 = self.rotate(x1, y1, -rotation, mx, my)
        print (x1, y1)
        x, y = self.rotate(x1, y1, rotation, mx, my)
        print (x1, y1)

        x2, y2 = self.rotate(x2, y2, -rotation, mx, my)

        print("corner rotate:", x1, ',', y1, x2, ',', y2)

        for h in range(img_height):
            for w in range(img_width):
                real_x = (x1 * (img_width - w) + x2 * w) / img_width
                real_y = (y1 * (img_height - h) + y2 * h) / img_height
                # print(real_x, real_y)
                # real_x = (x1 + w * (real_width) / img_width)
                # real_y = (y1 - h * (real_height) / img_height)
                if real_x ** 2 + real_y ** 2 <= self.radius ** 2:
                    if pix[h][w] < thres:
                        # [TODO]
                        # if picture  pixel is small, when mapping to image_map should add more interval points
                        # but not gonna happen in near future?
                        x_on_map = int(round(self.radius * self.pixel_per_mm + real_x / self.pixel_per_mm))
                        y_on_map = int(round(self.radius * self.pixel_per_mm + real_y / self.pixel_per_mm))
                        # self.image_map[x_on_map][y_on_map] = pix[h][w]
                        self.image_map[x_on_map][y_on_map] = 0
        # alignment fail when float to int
        self.rotation = rotation

    def find_edges(self):
        """
        find the edge of 4 sides
        return left-bound, right-bound, up-bound, down-bound
        """
        x1 = -53.03  # = 75/(sqrt(2))  Cyclic quadrilateral
        for i in range(len(self.image_map)):
            if any(j == 0 for j in self.image_map[i]):
                x1 = i
                break

        x2 = 53.03
        for i in range(len(self.image_map) - 1, 0 - 1, -1):
            if any(j == 0 for j in self.image_map[i]):
                x2 = i
                break

        y1 = False
        for j in range(len(self.image_map[0])):
            for i in range(len(self.image_map)):
                if self.image_map[i][j] == 0:
                    y1 = j
                    break
            if y1 is not False:
                break
        if y1 is False:
            y1 = 53.03

        y2 = False
        for j in range(len(self.image_map[0]) - 1, 0 - 1, -1):
            for i in range(len(self.image_map) - 1, 0 - 1, -1):
                if self.image_map[i][j] == 0:
                    y2 = j
                    break
            if y2 is not False:
                break
        if y2 is False:
            y2 = 53.03
        self.edges = [x1, x2, y1, y2]
        print(self.edges)

    def alignment_process(self, times=3):
        gcode = []
        self.find_edges()
        gcode += self.turnHalf()
        for _ in range(times):
            gcode += self.bitmap_moveTo(self.edges[0], self.edges[2])
            gcode += ["G4 P300"]
            gcode += self.bitmap_moveTo(self.edges[0], self.edges[3])
            gcode += ["G4 P300"]
            gcode += self.bitmap_moveTo(self.edges[1], self.edges[3])
            gcode += ["G4 P300"]
            gcode += self.bitmap_moveTo(self.edges[1], self.edges[2])
            gcode += ["G4 P300"]
        gcode += self.turnOff()

        return gcode

    def export_to_stream(self, stream):
        stream.write(self.gcode_generate())

    def gcode_generate(self):
        gcode = []

        gcode += self.header('bitmap')
        # pix = cv2.imread('S.png')
        # pix = cv2.cvtColor(pix, cv2.COLOR_BGR2GRAY)

        # print pix.shape
        # input()

        # last_i = 0
        # # gcode += ["M104 S200"]
        # gcode += turnOff()
        # gcode += turnHalf()

        # gcode += self.alignment_process()

        #row iteration
        for h in range(0, len(self.image_map)):
            #column iteration
            itera = range(0, len(self.image_map))
            final_x = len(self.image_map)
            if h % 2 == 1:
                final_x = 0
                itera = reversed(range(0, len(self.image_map)))

            for w in itera:
                if self.image_map[h][w] < self.thres:
                    if not self.laser_on:
                        last_i = w
                        gcode += self.bitmap_moveTo(w, h)
                        gcode += self.turnOn()
                else:
                    if self.laser_on:
                        if abs(w - last_i) < 2:  # Single dot
                            pass
                            gcode += ["G4 P100"]
                        elif final_x > 0:
                            gcode += self.bitmap_drawTo(w, h)
                        else:
                            gcode += self.bitmap_drawTo(w, h)
                        gcode += self.turnOff()

            if self.laser_on:
                gcode += self.bitmap_drawTo(final_x, h)
                gcode += self.turnOff()

        # gcode += ["M104 S0"]
        gcode += ["G28"]

        store = True
        if store:
            with open('./S.gcode', 'w') as f:
                print("\n".join(gcode) + "\n", file=f)
                # print >> f, "\n".join(gcode) + "\n"
        else:
            pass

        return "\n".join(gcode) + "\n"

if __name__ == '__main__':
    a = laser_bitmap()
    print(a)

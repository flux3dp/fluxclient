class laser(object):
    """base class for all laser usage calss"""
    def __init__(self):
        self.laser_on = False

    def turnOn(self):
        if self.laser_on:
            return []
        self.laser_on = True
        return ["G4 P1", "@X9L0"]

    def turnOff(self):
        if not self.laser_on:
            return []
        self.laser_on = False
        return ["G4 P1", "@X9L255"]

    def turnHalf(self):
        self.laser_on = False
        return ["G4 P1", "@X9L220"]

    def to_image(self, buffer_data, img_width, img_height):
        """
        convert buffer_data(bytes) to a 2d array
        """
        int_data = list(buffer_data)
        assert len(int_data) == img_width * img_height, "data length != width * height, %d != %d * %d" % (len(int_data), img_width, img_height)
        image = [int_data[i * img_width: (i + 1) * img_width] for i in range(img_height)]

        return image


from math import pow


def svg2laser(proc, svg_factory, z_height, travel_speed=2400,
              engraving_speed=400, engraving_strength=1.0,
              focal_length=6.4, progress_callback=lambda p: None):
    proc.append_comment("FLUX Laser Svg Tool")
    proc.set_toolhead_pwm(0)
    proc.moveto(feedrate=5000, x=0, y=0, z=z_height + focal_length)
    current_xy = None

    for src_xy, dist_xy in svg_factory.walk(progress_callback):
        if current_xy != src_xy:
            proc.set_toolhead_pwm(0)
            src_x, src_y = src_xy
            proc.moveto(feedrate=travel_speed, x=src_x, y=src_y)
            proc.set_toolhead_pwm(engraving_strength)

        dist_x, dist_y = dist_xy
        proc.moveto(feedrate=engraving_speed, x=dist_x, y=dist_y)
        current_xy = dist_xy

    proc.set_toolhead_pwm(0)
    proc.moveto(feedrate=travel_speed, x=0, y=0)


def bitmap2laser(proc, bitmap_factory, z_height, one_way=True,
                 vertical=False, travel_speed=2400, engraving_speed=400,
                 shading=True, max_engraving_strength=1.0, focal_length=6.4,
                 progress_callback=lambda p: None):
    val255 = 0
    proc.append_comment("FLUX Laser Bitmap Tool")
    proc.set_toolhead_pwm(0)
    proc.moveto(feedrate=5000, x=0, y=0, z=z_height + focal_length)

    ptr_width = 0.5 / bitmap_factory.pixel_per_mm

    if shading:
        val2pwm = tuple(min(max_engraving_strength, pow(((i / 255.0)), 0.7))
                        for i in range(256))
    else:
        val2pwm = tuple(0 if i == 0 else max_engraving_strength
                        for i in range(256))

    for progress, y, enum in bitmap_factory.walk_horizon():
        progress_callback(progress)
        y_axis_moved = False
        for x, val in enum:
            if val == 0:
                if val255:
                    val255 = 0
                    proc.moveto(x=x - ptr_width)
                    proc.set_toolhead_pwm(0)
            else:
                if y_axis_moved is False:
                    proc.moveto(feedrate=travel_speed, x=(x - 3))
                    proc.moveto(y=y)
                    y_axis_moved = True
                if val != val255:
                    if val255:
                        proc.moveto(feedrate=engraving_speed,
                                    x=(x - ptr_width))
                    else:
                        proc.moveto(feedrate=travel_speed, x=(x - ptr_width))
                    val255 = val
                    proc.set_toolhead_pwm(val2pwm[val255])

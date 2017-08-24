
from fluxclient.toolpath import vinyl_utils

R2 = 85 ** 2  # temp


def bmp2drawing(proc, bitmap_factory, travel_speed=2400, travel_zheight=5.0,
                drawing_zheight=(0.0, 0.2), progress_callback=lambda p: None):
    proc.append_comment("FLUX Bitmap Drawing Tool")
    proc.moveto(feedrate=5000, x=0, y=0, z=travel_zheight)

    base = drawing_zheight[0]
    delta = drawing_zheight[1] - drawing_zheight[0]
    working_zheight = tuple((base + ((255 - i) / 255 * delta) for i in range(255)))

    for progress, y, enum in bitmap_factory.walk_horizon():
        progress_callback(progress)

        y_sync = False
        x_bound = (R2 - y * y) ** 0.5

        for x, val in enum:
            if -x > x_bound:
                pass
            elif x > x_bound:
                continue

            if not y_sync:
                y_sync = True
                proc.moveto(feedrate=travel_speed, x=x, y=y)
            else:
                proc.moveto(feedrate=travel_speed, x=x)

            proc.moveto(feedrate=5000, z=working_zheight[val])
            proc.moveto(feedrate=5000, z=travel_zheight)


def svg2drawing(proc, svg_factory, travel_speed=2400, drawing_speed=1200,
                travel_zheight=5.0, drawing_zheight=0.0,
                progress_callback=lambda p: None):
    proc.append_comment("FLUX Drawing Svg Tool")
    proc.moveto(feedrate=5000, x=0, y=0, z=travel_zheight)

    current_xy = None

    for src_xy, dist_xy in svg_factory.walk(progress_callback):
        if current_xy != src_xy:
            proc.moveto(feedrate=5000, z=travel_zheight)
            x, y = src_xy
            proc.moveto(feedrate=travel_speed, x=x, y=y)
            proc.moveto(feedrate=5000, z=drawing_zheight)
        x, y = dist_xy
        proc.moveto(feedrate=drawing_speed, x=x, y=y)
        current_xy = dist_xy

    proc.moveto(feedrate=5000, z=travel_zheight)
    proc.moveto(feedrate=travel_speed, x=0, y=0)


def svg2vinyl(proc, svg_factory, travel_speed=2400, cutting_speed=800,
              travel_zheight=13, cutting_zheight=11.4,
              blade_radius=0.24, overcut=2.0,
              precut_at=None, progress_callback=lambda p: None):
    proc.append_comment("FLUX Vinyl Svg Tool")
    proc.moveto(feedrate=5000, x=0, y=0, z=travel_zheight)

    current_vector = None
    current_xy = None

    g = svg_factory.walk(progress_callback)

    if precut_at:
        src_x, src_y = precut_at

        current_vector = 1, 0
        current_xy = src_x + 1, src_y

        proc.moveto(feedrate=travel_speed, x=src_x, y=src_y)
        proc.moveto(feedrate=5000, z=cutting_zheight)

        fix_x, fix_y = vinyl_utils.get_knife_compensation(
            current_xy, current_vector, radius=blade_radius)
        proc.moveto(feedrate=cutting_speed, x=fix_x, y=fix_y)

    else:
        try:
            src_xy, dist_xy = g.__next__()
            src_x, src_y = src_xy
            dist_x, dist_y = dist_xy

            vector = dist_x - src_x, dist_y - src_y

            proc.moveto(feedrate=travel_speed, x=src_x, y=src_y)
            proc.moveto(feedrate=5000, z=cutting_zheight)
            proc.moveto(feedrate=cutting_speed, x=dist_x, y=dist_y)

            current_vector = vector
            current_xy = dist_xy
        except StopIteration:
            pass

    for src_xy, dist_xy in g:
        vector = dist_xy[0] - src_xy[0], dist_xy[1] - src_xy[1]
        for fix_x, fix_y in vinyl_utils.fix_knife_direction(
                current_xy, current_vector, vector, radius=blade_radius):
            proc.moveto(feedrate=400, x=fix_x, y=fix_y)

        if current_xy != src_xy:
            # Travel from current_xy to src_xy and start cut
            proc.moveto(feedrate=5000, z=travel_zheight)
            fix_x, fix_y = vinyl_utils.get_knife_compensation(
                src_xy, (-vector[0], -vector[1]), radius=blade_radius)
            proc.moveto(feedrate=travel_speed, x=fix_x, y=fix_y)
            proc.moveto(feedrate=5000, z=cutting_zheight)

        fix_x, fix_y = vinyl_utils.get_knife_compensation(dist_xy, vector, radius=blade_radius)
        proc.moveto(feedrate=cutting_speed, x=fix_x, y=fix_y)
        current_vector = vector
        current_xy = dist_xy

    proc.moveto(feedrate=5000, z=travel_zheight)
    proc.moveto(feedrate=travel_speed, x=0, y=0)

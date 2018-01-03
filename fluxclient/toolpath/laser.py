
from math import pow

R2 = 85 ** 2  # temp


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
            #proc.moveto(feedrate=travel_speed, x=src_x, y=src_y)
            proc.moveto(feedrate=engraving_speed, x=src_x, y=src_y)
            proc.set_toolhead_pwm(engraving_strength)

        dist_x, dist_y = dist_xy
        proc.moveto(feedrate=engraving_speed, x=dist_x, y=dist_y)
        current_xy = dist_xy

    proc.set_toolhead_pwm(0)
    proc.moveto(feedrate=travel_speed, x=0, y=0)

def svgeditor2laser(proc, svg_factory, z_height, travel_speed=12000,
                    engraving_strength=1.0, focal_length=6.4,
                    progress_callback=lambda p: None):

    proc.append_comment("FLUX Laser Svgeditor Tool")
    proc.home()
    proc.set_toolhead_pwm(0)
    #proc.moveto(feedrate=12000, x=0, y=0, z=z_height + focal_length)
    current_pwm = 0
    current_y = -999
    current_speed = 0
    current_power_limit = 0
    ending_x = -1
    ACCELERATION_BUFFER_LENGTH = 5 # millimeter
    from_left = False
    power_limit = 1000
    is_bitmap = False
    shading = False

    for strength, args, dist_xy in svg_factory.walk(progress_callback):
        if strength < 0:
            from_left = args.get("from_left", from_left)
            power_limit = args.get("power_limit", power_limit)
            is_bitmap = args.get("is_bitmap", is_bitmap)
            shading = args.get("shading", shading)
            if "shading" in args:
                if shading:
                    print("Enable shading")
                    proc.set_toolhead_fan_speed(-1)
                else:
                    print("Disable shading")
                    proc.set_toolhead_fan_speed(1)
            continue
        else:
            speed = args
        
        speed = speed * 60
        speed = travel_speed if (current_pwm == 0 and not is_bitmap) else speed
        pwm = strength / 100.0
        need_to_change = False
        current_line_start_engraving_pts = 0

        if current_pwm != pwm:
            current_pwm = pwm
            need_to_change = True

        # Pass for non-engraving route
        if current_pwm == 0 and not need_to_change:
            continue

        if dist_xy == 'done' or dist_xy == 'line':
            proc.set_toolhead_pwm(0)
        else:
            dist_x, dist_y = dist_xy
            movement_args = dict(x=dist_x)
            
            if current_y != dist_y:
                # Do x-acceleration buffer
                if current_pwm > 0 and is_bitmap and ending_x > -1:
                    proc.set_toolhead_pwm(0)
                    if from_left:
                        buffer_current = dict(x = min(400, ending_x - ACCELERATION_BUFFER_LENGTH), y = current_y, feedrate = travel_speed)
                        buffer_next = dict(x = max(0, dist_x - ACCELERATION_BUFFER_LENGTH), y = dist_y)
                    else:
                        buffer_current = dict(x = min(400, ending_x + ACCELERATION_BUFFER_LENGTH), y = current_y, feedrate = travel_speed)
                        buffer_next = dict(x = max(0, dist_x + ACCELERATION_BUFFER_LENGTH), y = dist_y)
                    if buffer_next['x'] < 0:
                        buffer_next['x'] = 0
                    if buffer_next['x'] > 400:
                        buffer_next['x'] = 400
                    proc.moveto(**buffer_current)
                    proc.moveto(**buffer_next)
                    current_speed = travel_speed

                current_y = dist_y
                movement_args['y'] = dist_y
                
                    
            if current_speed != speed:
                current_speed = speed
                movement_args['feedrate'] = speed

            proc.moveto(**movement_args)
            ending_x = dist_x

            if need_to_change:
                proc.set_toolhead_pwm(pwm)
            if current_power_limit != power_limit:
                current_power_limit = power_limit
                proc.set_toolhead_pwm(-power_limit)
#================for testing============================================
#    pwm = 0
#    for dist_x, dist_y in svg_factory.walk_cal():
#        if dist_x == 'line':
#            pwm = 0
#        elif dist_x % 1 == 0:
#            pwm += 1
#            cpwm = cal_pwm(pwm)
#            proc.set_toolhead_pwm(cpwm)
#            print('result :', cpwm, dist_x, dist_y)
#            proc.moveto(feedrate=12000, x=dist_x, y=dist_y)
#=======================================================================

def bitmap2laser(proc, bitmap_factory, z_height, one_way=True,
                 vertical=False, travel_speed=6000, engraving_speed=400,
                 shading=True, max_engraving_strength=1.0, focal_length=6.4,
                 progress_callback=lambda p: None):
    def gen_val2pwm():
        if shading:
            val2pwm = tuple(max_engraving_strength * pow(((i / 255.0)), 0.7)
                            for i in range(256))
        else:
            val2pwm = tuple(0 if i == 0 else max_engraving_strength
                            for i in range(256))
        return val2pwm

    def moveto_first_enter_point(y, enum):
        nonlocal current_pwm
        for x, val in enum:
            if val:
                proc.set_toolhead_pwm(0)
                #proc.moveto(feedrate=travel_speed, x=x, y=y)
                proc.moveto(feedrate=engraving_speed, x=x, y=y)
                proc.set_toolhead_pwm(val2pwm[val])
                current_pwm = val
                break

    def draw_until_endpoint(y, enum):
        nonlocal current_pwm
        for x, val in enum:
            if val != current_pwm:
                proc.set_toolhead_pwm(val2pwm[current_pwm])
                feedrate = engraving_speed if current_pwm else travel_speed
                #proc.moveto(feedrate=feedrate, x=x)
                proc.moveto(x=x)
            current_pwm = val

    current_pwm = 0
    proc.append_comment("FLUX Laser Bitmap Tool")
    proc.set_toolhead_pwm(0)
    proc.moveto(feedrate=5000, x=0, y=0, z=z_height + focal_length)

    val2pwm = gen_val2pwm()
    #walk_method = bitmap_factory.walk_horizon
    walk_method = bitmap_factory.walk_spath

    for progress, y, enum in walk_method():
        progress_callback(progress)

        moveto_first_enter_point(y, enum)
        draw_until_endpoint(y, enum)


def laserCalibration(proc, bitmap_factory, z_height, one_way=True,
                     vertical=False, travel_speed=2400, engraving_speed=400,
                     shading=True, max_engraving_strength=1.0,
                     focal_length=6.4, progress_callback=lambda p: None):

    def isAnotherLine():
        return True if last_line - y > 0.1001 else False

    def moveZifNeeded():
        nonlocal last_line, z_height
        if isAnotherLine():
            z_height = z_height + 0.2
            proc.moveto(z=z_height)
        last_line = y

    #=============init setting===============================================
    current_pwm = 0
    last_line = -85
    proc.append_comment("FLUX Laser Calibration")
    proc.set_toolhead_pwm(0)
    z_height = z_height + focal_length - 2
    ptr_width = 0.5 / bitmap_factory.pixel_per_mm
    proc.moveto(feedrate=5000, x=0, y=0, z=z_height)
    val2pwm = tuple(max_engraving_strength * pow(((i / 255.0)), 0.7)
                    for i in range(256))

    for progress, y, enum in bitmap_factory.walk_horizon():
        progress_callback(progress)
        x_bound = (R2 - y * y) ** 0.5

        # Find first engrave point in row
        for x, val in enum:
            if x > -x_bound and val:
                proc.moveto(y=y)

                moveZifNeeded()

                if x - ptr_width > -x_bound:
                    proc.moveto(feedrate=travel_speed, x=max(x - 3 - ptr_width,
                                                             -x_bound))
                proc.moveto(feedrate=travel_speed, x=max(x - ptr_width,
                                                         -x_bound))
                proc.set_toolhead_pwm(val2pwm[val])
                current_pwm = val
                break

        # Draw until x over limit
        for x, val in enum:
            if x + ptr_width > x_bound:
                if val != current_pwm:
                    proc.moveto(feedrate=engraving_speed, x=(x - ptr_width))
                    proc.set_toolhead_pwm(val)

                if val:
                    proc.moveto(feedrate=engraving_speed, x=x_bound)
                    proc.set_toolhead_pwm(0)
                current_pwm = 0
                break

            else:
                if val != current_pwm:
                    feedrate = engraving_speed if current_pwm else travel_speed
                    proc.moveto(feedrate=feedrate, x=x - ptr_width)
                current_pwm = val
                proc.set_toolhead_pwm(val2pwm[current_pwm])

        if current_pwm:
            proc.set_toolhead_pwm(0)
            current_pwm = 0

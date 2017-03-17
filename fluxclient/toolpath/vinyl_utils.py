from math import sqrt, sin, cos, acos

KNIFE_RADIUS = 0.6
ROTATE_ANGLE = 0.1
ANGLE_THRESHOLD = 0.01


def rotate(v, angle):
    return (v[0] * cos(angle) - v[1] * sin(angle),
            v[0] * sin(angle) + v[1] * cos(angle))


def length(v):
    x, y = v
    return sqrt(x * x + y * y)


def get_angle(v1, v2):
    x1, y1 = v1
    x2, y2 = v2

    d = -1 if x1 * y2 - y1 * x2 < 0 else 1
    return d * acos(max(min((x1 * x2 + y1 * y2) / length(v1) / length(v2), 1),
                        -1))


def get_knife_compensation(position, vector, radius=KNIFE_RADIUS):
    r = radius / length(vector)
    return position[0] + vector[0] * r, position[1] + vector[1] * r


def fix_knife_direction(current_pos, vector, target_vector,
                        radius=KNIFE_RADIUS):
    angle = get_angle(vector, target_vector)

    while abs(angle) > ANGLE_THRESHOLD:
        dir = 1 if angle > 0 else -1
        vector = rotate(vector, dir * min(ROTATE_ANGLE, abs(angle)))
        v_scale = radius / length(vector)
        vector = (vector[0] * v_scale, vector[1] * v_scale)

        x, y = current_pos[0] + vector[0], current_pos[1] + vector[1]
        yield x, y

        angle = get_angle(vector, target_vector)

HW_PROFILE = {
    'model-1': {
        'plate_shape': 'elliptic',
        'radius': 86.,
        'scan_full_len': 360.,
        'height': 240.,
        'nozzle_height': 4.45,
        # 'step_setting': {400: (3, 2.7), 800: (7, 3.15)}
        'step_setting': {100: (1, 3.6), 200: (1, 1.8), 400: (1, 0.9), 800: (1, 0.45), 1200: (1, 0.3)}
        # {N: (alpha, step_len)}
        # find min alpha that gcd(alpha, N) == 1 and scan_full_len / N * alpha > min_len_that_step_motor_can_move
        # min_len_that_step_motor_can_move = 2.5
        # step_len = alpha * scan_full_len / N
    },
    'beambox': {
        'plate_shape': 'rectangular',
        'width': 400,
        'length': 400,
        'height': 10,
        'radius': 400
        # 'nozzle_height': 4.45,
        # 'step_setting': {400: (3, 2.7), 800: (7, 3.15)}
        # 'step_setting': {100: (1, 3.6), 200: (1, 1.8), 400: (1, 0.9), 800: (1, 0.45), 1200: (1, 0.3)}
    }
}
# HW_PROFILE['model-1']['radius']

class HardwareData(object):
    def __init__(self, model):
        self.model = model
        self._genattr()

    def _genattr(self):
        profile = HW_PROFILE.get(self.model)
        if not profile:
            return None

        for key, val in profile.items():
            setattr(self, key, val)

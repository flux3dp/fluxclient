HW_PROFILE = {
    'model-1': {
        'radius': 85.,
        'scan_full_len': 360.,
        'height': 240.,
        # 'step_setting': {400: (3, 2.7), 800: (7, 3.15)}
        'step_setting': {100: (1, 3.6), 200: (1, 1.8), 400: (1, 0.9), 800: (1, 0.45), 1200: (1, 0.3)}
        # {N: (alpha, step_len)}
        # find min alpha that gcd(alpha, N) == 1 and scan_full_len / N * alpha > min_len_that_step_motor_can_move
        # min_len_that_step_motor_can_move = 2.5
        # step_len = alpha * scan_full_len / N
    },
}

HW_PROFILE = {
    "model-1": {
        "radius": 17,
        'scan_full_len': 115.2,
        'step_setting': {400: (3, 0.864), 800: (7, 1.008)}
        # {N: (a, step_len)}
        # find min a that gcd(a, N) == 1 and scan_full_len / N * a > min_len_that_step_motor_can_move
        # min_len_that_step_motor_can_move = 0.8
    },
}

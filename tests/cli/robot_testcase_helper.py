

def with_uuid(robot, device=None):
    assert(robot)
    assert(device)
    robot.report_play()


def with_ipaddr(robot, device=None):
    assert(robot)
    assert(device is None)
    robot.report_play()


class RobotError(Exception):
    pass


class RobotNotReadyError(RobotError):
    pass


class RobotDisconnected(RobotError):
    pass

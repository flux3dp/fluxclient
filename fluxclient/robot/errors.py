
class RobotSessionError(Exception):
    """RobotSessionError raised while connection error or protocel error \
occour. Connection will be closed directly."""

    error_symbol = None

    def __init__(self, *args, **kw):  # noqa
        super(RobotSessionError, self).__init__(*args)
        if "error_symbol" in kw:
            self.error_symbol = kw["error_symbol"]
        else:
            self.error_symbol = ("DISCONNECTED", )


class RobotError(RuntimeError):
    """RobotError raised while device return an error."""

    error_symbol = None

    def __init__(self, *args, **kw):  # noqa
        super(RobotError, self).__init__(*args)
        if "error_symbol" in kw:
            self.error_symbol = kw["error_symbol"]
        else:
            self.error_symbol = ("UNKNOWN_ERROR", )


class RobotNotReadyError(RobotError):
    """Raise when session not ready"""

    error_symbol = ("OPERATION_ERROR", )

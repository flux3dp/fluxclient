
from stat import S_ISCHR
import os


def is_serial_port(filepath):
    if os.path.exists(filepath):
        st_mode = os.stat(filepath).st_mode
        return S_ISCHR(st_mode)
    else:
        return False

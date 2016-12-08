
from .base import (ManagerError, ManagerException, NotSupportError, AuthError,
                   TimeoutError, ConnectionBroken)
from .ssl1 import SSL1Backend
from .udp1 import Udp1Backend

__all__ = ["ManagerError", "ManagerException", "NotSupportError", "AuthError",
           "TimeoutError", "ConnectionBroken", "NETWORK_BACKENDS"]

NETWORK_BACKENDS = (SSL1Backend, Udp1Backend)

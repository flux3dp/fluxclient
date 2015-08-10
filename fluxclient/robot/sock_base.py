

class RobotSocketBase(object):
    def __init__(self, sock):
        self.sock = sock

    def fileno(self):
        return self.sock.fileno()

    @property
    def family(self):
        return self.sock.family

    def getpeername(self, *args, **kw):
        return self.sock.getpeername(*args, **kw)

    def getsockname(self, *args, **kw):
        return self.sock.getsockname(*args, **kw)

    def getsockopt(self, *args, **kw):
        return self.sock.getsockopt(*args, **kw)

    def gettimeout(self, *args, **kw):
        return self.sock.gettimeout(*args, **kw)

    def shutdown(self, *args, **kw):
        return self.sock.shutdown(*args, **kw)

    def close(self):
        return self.sock.close()

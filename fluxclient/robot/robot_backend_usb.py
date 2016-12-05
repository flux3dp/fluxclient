
from threading import Thread
from time import time
import logging
import socket

from .robot_backend_2 import RobotBackend2, RobotError, raise_error

logger = logging.getLogger(__name__)


class RobotBackendUSB(RobotBackend2):
    def __init__(self, usbprotocol):
        self.channel = usbprotocol.open_channel("robot")

    def send_cmd(self, cmd):
        self.channel.send_object((cmd, ))

    def get_resp(self, timeout=3.0):
        return self.channel.get_object(timeout)

    def _upload_stream(self, instance, cmd, stream, size,
                       process_callback=None):
        if cmd:
            upload_ret = self.make_cmd(cmd.encode()).decode("ascii", "ignore")

            if upload_ret == "continue":
                logger.info(upload_ret)
            else:
                raise_error(upload_ret)

        logger.debug("Upload stream length: %i" % size)
        sent = 0
        ts = 0

        if process_callback:
            process_callback(instance, sent, size)

        while sent < size:
            buf = stream.read(1020)
            lbuf = len(buf)
            if lbuf == 0:
                raise RobotError("Upload file error")
            sent += lbuf
            self.channel.send_binary(buf)

            if process_callback and time() - ts > 1.0:
                ts = time()
                process_callback(instance, sent, size)

        if process_callback:
            process_callback(instance, sent, size)

    def recv_binary_into(self, binary_header, stream, callback=None):
        mn, mimetype, ssize = binary_header.split(" ")
        if mn != "binary":
            raise RobotError("Protocol Error")
        size = int(ssize)
        logger.debug("Recv %s %i" % (mimetype, size))
        if size == 0:
            return mimetype
        left = size
        while left > 0:
            buf = self.channel.get_buffer()
            left -= stream.write(buf)
            if callback:
                callback(left, size)
        return mimetype

    def begin_raw(self):
        ret = self.make_cmd(b"task raw")
        if ret == b"continue":
            self.raw_stream = USB2Stream(self.channel)
            return self.raw_stream.public
        else:
            raise_error(ret.decode("ascii", "ignore"))

    def quit_raw_mode(self):
        self.channel.send_binary(b"quit")
        ret = self.get_resp().decode("ascii", "ignore")
        if ret != "ok":
            raise_error(ret)

        self.raw_stream.running = False
        del self.raw_stream

    def close(self):
        self.channel.close()


class USB2Stream(object):
    def __init__(self, channel):
        self.running = True
        self.channel = channel
        self.internal, self.public = socket.socketpair()

        self.channel.binary_stream = self.internal
        self.thread_tx = Thread(target=self.run_tx)
        self.thread_tx.daemon = True
        self.thread_tx.start()

    def run_tx(self):
        while self.running:
            self.channel.send_binary(self.internal.recv(128))

    def close(self):
        self.channel.binary_stream = None

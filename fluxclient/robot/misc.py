
import re


def is_uuid(input):
    return True if re.match("[0-9a-fA-F]{32}", input) else False


def msg_waitall(sock, length):
    buf = b""

    while len(buf) < length:
        buf += sock.recv(length - len(buf))

    return buf


from time import sleep
from uuid import UUID
import sys
import re

from fluxclient.upnp.task import UpnpTask


def is_uuid(input):
    return True if re.match("[0-9a-fA-F]{32}", input) else False


def parse_ipaddr(target):
    if ":" in target:
        addr, port = target.split(":")
        return (addr, int(port))
    else:
        return (target, 23811)


def require_robot(target, logstream=sys.stdout):
    def lookup_callback(discover):
        logstream.write(".")
        logstream.flush()

    if is_uuid(target):
        logstream.write("Discover...")
        logstream.flush()

        task = UpnpTask(UUID(hex=target), lookup_callback=lookup_callback)
        ipaddr = task.endpoint[0]
        logstream.write(" OK\n")
        logstream.write("Name: %s\nUUID: %s\nModel: %s\nIP Addr: %s\n" %
                        (task.name, task.uuid.hex, task.model_id, ipaddr))
        logstream.flush()

        while True:
            try:
                task.require_auth()
                break
            except RuntimeError as e:
                logstream.write("Error: %s, retry...\n" % e.args[0])


        logstream.write("Wakeup Robot: ")
        logstream.flush()

        while True:
            try:
                resp = task.require_robot()
                if resp:
                    st = resp.get("status")
                    if st == "initial":
                        logstream.write("+")
                        logstream.flush()
                        sleep(0.3)
                    elif st == "launching":
                        logstream.write(".")
                        logstream.flush()
                        sleep(0.3)
                    elif st == "launched":
                        logstream.write(" :-)")
                        if "info" in resp:
                            logstream.write(" (%s)" % resp["info"])
                        logstream.write("\n")
                        logstream.flush()
                        return (ipaddr, 23811), task.slave_key
                else:
                    logstream.write("?")
            except RuntimeError as e:
                if e.args[0] == "TIMEOUT":
                    logstream.write("Error: %s\n" % e.args[0])
                    sleep(3)
                    logstream.write("Retry require robot\n")
                elif e.args[0] == "AUTH_ERROR":
                    if task.timedelta < -15:
                        logstream.write("Auth error, try fix time delta\n")
                        logstream.flush()
                        old_td = task.timedelta
                        task.reload_remote_profile(
                            lookup_timeout=30.,
                            lookup_callback=lookup_callback)
                        if task.timedelta - old_td < 0.5:
                            raise
                        else:
                            # Fix timedelta issue let's retry
                            continue
                    else:
                        logstream.write("Nothing can do: %s\n" % task.timedelta)
                        logstream.flush()
                    raise

    else:
        return parse_ipaddr(target), None


def kill_robot(target):
    if is_uuid(target):
        uuid = UUID(hex=target)
        task = UpnpTask(uuid)
        ipaddr = task.endpoint[0]
        print("UUID: %s\nSerial: %s\nModel: %s\nIP Addr: %s\n" %
              (task.uuid.hex, task.serial, task.model_id, ipaddr))

        task.require_auth()
        task.kill_control()
        print("Kill signal sent.")
    else:
        raise RuntimeError("Kill must give serial, not IP addr")


def msg_waitall(sock, length):
    buf = b""

    while len(buf) < length:
        buf += sock.recv(length - len(buf))

    return buf

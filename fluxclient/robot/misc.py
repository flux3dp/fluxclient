
from time import sleep
import sys

from fluxclient.upnp.task import UpnpTask
from fluxclient.upnp.misc import is_serial


def select_ipaddr(remote_addrs):
    return (remote_addrs[0][0], 23811)


def parse_ipaddr(target):
    if ":" in target:
        addr, port = target.split(":")
        return (addr, int(port))
    else:
        return (target, 23811)


def require_robot(target, logstream=sys.stdout):
    def lookup_callback(discover):
        logstream.write(".")

    if is_serial(target):
        logstream.write("Discover...")
        task = UpnpTask(target, lookup_callback=lookup_callback)
        ipaddr = select_ipaddr(task.remote_addrs)
        logstream.write(" OK\n")
        logstream.write("Serial: %s\nModel: %s\nIP Addr: %s\n" %
                        (task.serial.hex, task.model_id, ipaddr[0]))

        task.require_auth()

        try:
            resp = task.require_robot()
            if resp is not None:
                logstream.write("Robot launching...\n")
                sleep(0.5)
        except RuntimeError as e:
            if e.args[0] == "ALREADY_RUNNING":
                logstream.write("Robot already running\n")
            else:
                raise

        return ipaddr, task.remote_keyobj
    else:
        return parse_ipaddr(target), None


def kill_robot(serial):
    if is_serial(serial):
        task = UpnpTask(serial)
        ipaddr = select_ipaddr(task.remote_addrs)
        print("Serial: %s\nModel: %s\nIP Addr: %s\n" %
              (task.serial.hex, task.model_id, ipaddr[0]))

        task.require_auth()
        task.kill_control()
        print("Kill signal sent.")
    else:
        raise RuntimeError("Kill must give serial, not IP addr")


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
        logstream.flush()

    if is_serial(target):
        logstream.write("Discover...")
        logstream.flush()

        task = UpnpTask(target, lookup_callback=lookup_callback)
        ipaddr = select_ipaddr(task.remote_addrs)
        logstream.write(" OK\n")
        logstream.write("Name: %s\nSerial: %s\nModel: %s\nIP Addr: %s\n" %
                        (task.name, task.serial.hex, task.model_id, ipaddr[0]))
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
                        return ipaddr, task.remote_keyobj
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
                        task.update_remote_infomation(
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

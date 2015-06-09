
import threading
import argparse
import logging
import sys
import re

from fluxclient.robot_console import RobotConsole
from fluxclient.robot import connect_robot


def config_logger(stdout=sys.stderr, level=logging.DEBUG):
    logging.basicConfig(format="%(message)s", stream=stdout)
    logger = logging.getLogger('')
    logger.setLevel(level)
    return logger

def is_serial(target):
    return True if re.match("[0-9A-Z]{25}", target) else False


def parse_ipaddr(target):
    if ":" in target:
        addr, port = target.split(":")
        return (addr, int(port))
    else:
        return (target, 23811)


def select_ipaddr(remote_addrs):
    return (remote_addrs[0][0], 23811)


def parse_target(options, logstream=sys.stdout):
    from fluxclient.upnp_task import UpnpTask
    from time import sleep

    def lookup_callback(discover):
        logstream.write(".")

    if is_serial(options.target):
        logstream.write("Discover...")
        task = UpnpTask(options.target, lookup_callback=lookup_callback)
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
        return parse_ipaddr(options.target), None


def robot_shell(options):
    from fluxclient.console import Console

    with Console() as console:
        console.setup()

        def conn_callback(*args):
            console.write(".")
            return True

        try:
            logger = config_logger(console)
            ipaddr, keyobj = parse_target(options, logstream=console)
            client = connect_robot(ipaddr=ipaddr, server_key=keyobj,
                                   conn_callback=conn_callback)
            robot_client = RobotConsole(client)
            logger.info("----> READY")
            while True:
                try:
                    cmd = console.read_cmd()
                    if cmd:
                        robot_client.on_cmd(cmd)
                except Exception:
                    logger.exception("Unhandle Error")

        except Exception:
            logger.exception("Unhandle Error")
            console.append_log("Press Control + C to quit")
            while True:
                cmd = console.read_cmd()


def ipython_shell(options):
    logger = config_logger()
    import IPython

    def conn_callback(*args):
        sys.stdout.write(".")
        sys.stdout.flush()
        return True

    ipaddr, keyobj = parse_target(options)
    robot_client = connect_robot(ipaddr=ipaddr, server_key=keyobj,
                                 conn_callback=conn_callback)

    logger.info("----> READY")
    logger.info(">> robot_client")
    IPython.embed()


def simple_shell(options):
    from fluxclient.console import Console
    logger = config_logger()

    def conn_callback(*args):
        sys.stdout.write(".")
        sys.stdout.flush()
        return True

    ipaddr, keyobj = parse_target(options)
    client = connect_robot(ipaddr=ipaddr, server_key=keyobj,
                           conn_callback=conn_callback)
    robot_client = RobotConsole(client)
    print("----> READY")

    while True:
        r = sys.stdin.readline().strip()
        if not r:
            continue
        try:
            robot_client.on_cmd(r.strip())
        except Exception:
            logger.exception("Unhandle Error")


def do_kill(options):
    if is_serial(options.target):
        from fluxclient.upnp_task import UpnpTask

        task = UpnpTask(options.target)
        ipaddr = select_ipaddr(task.remote_addrs)
        print("Serial: %s\nModel: %s\nIP Addr: %s\n" %
              (task.serial.hex, task.model_id, ipaddr[0]))

        task.require_auth()
        task.kill_control()
        print("Kill signal sent.")
    else:
        raise RuntimeError("Kill must give serial, not IP addr")

def connecting_callback(*args):
    print("...")
    return True


def main():
    parser = argparse.ArgumentParser(description='flux robot')

    parser.add_argument(dest='target', type=str,
                        help="Printer connect with. It can be printer serial "
                             "or IP address like 192.168.1.1 or "
                             "192.168.1.1:23811")
    parser.add_argument('--kill', dest='do_kill', action='store_const',
                        const=True, default=False, help='Use python shell')
    parser.add_argument('--ipython', dest='ipython', action='store_const',
                        const=True, default=False, help='Use python shell')
    parser.add_argument('--simple', dest='simple', action='store_const',
                        const=True, default=False, help='Use python shell')
    options = parser.parse_args()

    if options.do_kill:
        do_kill(options)
    elif options.ipython:
        ipython_shell(options)
    elif options.simple:
        simple_shell(options)
    else:
        robot_shell(options)

    return 0

if __name__ == "__main__":
    sys.exit(main())

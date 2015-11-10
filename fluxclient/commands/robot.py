
import readline
import argparse
import logging
import atexit
import sys
import re
import os

from fluxclient.robot_console import RobotConsole
from fluxclient.robot.misc import parse_ipaddr, kill_robot, require_robot
from fluxclient.robot import connect_robot
from fluxclient.upnp.misc import is_serial


def setup_logger(stdout=sys.stderr, level=logging.DEBUG):
    logging.basicConfig(format="%(message)s", stream=stdout)
    logger = logging.getLogger('')
    logger.setLevel(level)
    return logger


def setup_readline():
    histfile = os.path.join(os.path.expanduser("~"), ".flux_robot_history")
    try:
        readline.read_history_file(histfile)
    except IOError:
        pass
    atexit.register(readline.write_history_file, histfile)


def robot_shell(options):
    from fluxclient.console import Console

    with Console() as console:
        console.setup()
        setup_readline()

        def conn_callback(*args):
            console.write(".")
            return True

        try:
            logger = setup_logger(console)
            ipaddr, keyobj = require_robot(options.target, console)
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
    logger = setup_logger()
    import IPython

    def conn_callback(*args):
        sys.stdout.write(".")
        sys.stdout.flush()
        return True

    ipaddr, keyobj = require_robot(options.target)
    robot_client = connect_robot(ipaddr=ipaddr, server_key=keyobj,
                                 conn_callback=conn_callback)

    logger.info("----> READY")
    logger.info(">> robot_client")
    IPython.embed()


def simple_shell(options):
    logger = setup_logger()
    setup_readline()

    def conn_callback(*args):
        sys.stdout.write(".")
        sys.stdout.flush()
        return True

    ipaddr, keyobj = require_robot(options.target)
    client = connect_robot(ipaddr=ipaddr, server_key=keyobj,
                           conn_callback=conn_callback)
    robot_client = RobotConsole(client)
    logger.info("----> READY")

    while True:
        r = input("> ")
        if not r:
            continue
        try:
            robot_client.on_cmd(r.strip())
        except Exception:
            logger.exception("Unhandle Error")


def main():
    parser = argparse.ArgumentParser(description='flux robot')

    parser.add_argument(dest='target', type=str,
                        help="Printer connect with. It can be printer uuid "
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
        kill_robot(options.target)
    elif options.ipython:
        ipython_shell(options)
    elif options.simple:
        simple_shell(options)
    else:
        robot_shell(options)

    return 0

if __name__ == "__main__":
    sys.exit(main())

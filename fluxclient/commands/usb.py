
from getpass import getpass
import argparse
import sys

from fluxclient.utils.network_config import parse_network_config
from fluxclient.usb.task import UsbTask, UsbTaskError
from .misc import get_or_create_default_key, setup_logger, network_config_helper

PROG_DESCRIPTION = "Flux device usb configuration console."
PROG_EPILOG = ""
logger = None


def do_auth(task):
    try:
        return task.auth()
    except UsbTaskError as err:
        if err.args[0] == "BAD_PASSWORD":
            pwd = getpass("Password: ")
            return task.auth(pwd)
        else:
            raise


def printhelp():
    logger.info("""Help:
    3 - Modify general settings
    4 - Modify network settings
    5 - Check current wifi SSID
    6 - Set password
    7 - Get IP Address
    8 - Scan wifi""")


def do_general_config(task):
    name = input("Device Name: ")
    task.config_general({"name": name})


def do_network_config(task):
    kw = network_config_helper.run()
    options = parse_network_config(**kw)
    task.config_network(options)


def do_set_password(task):
    new_pwd = getpass("New password: ")
    if getpass("Confirm new password: ") != new_pwd:
        raise RuntimeError("New password not match")

    logger.info(task.set_password(new_pwd))


def do_get_ssid(task):
    logger.info(task.get_ssid())


def do_get_ipaddr(task):
    logger.info(task.get_ipaddr())


def do_scan_wifi(task):
    ret = task.list_ssid()
    logger.info("%-30s %-4s %s" % ("SSID", "RSSI", "SECURITY"))
    for l in ret:
        logger.info("%-30s %-4s %s" % (l.get("ssid"), l.get("rssi"),
                                       l.get("security")))


def cmdline(task):
    while True:
        printhelp()
        sys.stdout.write("> ")
        sys.stdout.flush()
        cmd = sys.stdin.readline().strip()
        if cmd == "3":
            do_general_config(task)
        elif cmd == "4":
            do_network_config(task)
        elif cmd == "5":
            do_get_ssid(task)
        elif cmd == "6":
            do_set_password(task)
        elif cmd == "7":
            do_get_ipaddr(task)
        elif cmd == "8":
            do_scan_wifi(task)
        else:
            logger.info("Unknow command id: %s" % cmd)


def main():
    global logger
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION,
                                     epilog=PROG_EPILOG)
    parser.add_argument('--verbose', dest='verbose', action='store_const',
                        const=True, default=False, help='Verbose output')
    parser.add_argument(dest='serial', type=str,
                        help="Device serial port")

    options = parser.parse_args()
    logger = setup_logger(__name__, debug=options.verbose)

    task = UsbTask(options.serial, get_or_create_default_key())
    logger.info("""Device:
    Nickname: %(name)s
    Serial: %(serial)s
    UUID: %(uuid)s
    Version: %(ver)s
    Has Password: %(pwd)s
    """ % {"name": task.name, "serial": task.serial, "uuid": task.uuid.hex,
           "ver": task.remote_version, "pwd": task.has_password})

    # resp = do_auth(task).decode("ascii", "ignore")
    # if resp not in ("OK", "ALREADY_TRUSTED"):
    #     raise RuntimeError(resp)

    cmdline(task)

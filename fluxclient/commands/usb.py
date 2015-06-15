
from getpass import getpass
import argparse
import logging
import sys

from fluxclient.usb.task import UsbTask, UsbTaskError

logging.basicConfig(format="%(message)s", stream=sys.stdout)
logger = logging.getLogger('')


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
    print("""Help:
    3 - Modify general settings
    4 - Modify network settings
    5 - Check current wifi SSID
    6 - Set password""")


def do_network_config(task):
    print("""Give config string syntax like:
    Wifi has password and using DHCP: "wifi ssid name","wifi password",dhcp
    """)


def do_set_password(task):
    pwd = getpass("Old password: ")
    new_pwd = getpass("New password: ")
    if getpass("Confirm new password: ") != new_pwd:
        raise RuntimeError("New password not match")

    print(task.set_password(pwd, new_pwd))


def do_get_ssid(task):
    print(task.get_ssid())


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
        else:
            print("Unknow command id: %s" % cmd)


def main():
    parser = argparse.ArgumentParser(description='flux usb tool')
    parser.add_argument('-d', dest='debug', action='store_const',
                        const=True, default=False, help='Print debug log')
    parser.add_argument('-b', dest='baudrate', type=str, default=115200,
                        help="Baudrate")
    parser.add_argument(dest='serial', type=str,
                        help="Device serial port")

    options = parser.parse_args()
    if options.debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    task = UsbTask(options.serial, options.baudrate)
    logger.info("""Device:
    Nickname: %(name)s
    Serial: %(serial)s
    Has Password: %(pwd)s
    """ % {"name": task.nickname, "serial": task.serial,
           "pwd": task.has_password})

    resp = do_auth(task).decode("ascii", "ignore")
    if resp not in ("OK", "ALREADY_TRUSTED"):
        raise RuntimeError(resp)

    cmdline(task)

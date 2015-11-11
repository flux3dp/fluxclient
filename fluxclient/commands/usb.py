
from ipaddress import IPv4Interface, IPv4Address
from getpass import getpass
import argparse
import logging
import sys

from fluxclient.usb.task import UsbTask, UsbTaskError
from fluxclient.upnp.misc import parse_network_config

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


def do_general_config(task):
    name = input("Device Name: ")
    task.config_general({"name": name})


def do_network_config(task):
    kw = {}
    ret = input("Wifi Mode (0=host,1=client)[1]: ").strip("\n")
    if ret == "0":
        kw["wifi_mode"] = "host"
    else:
        kw["wifi_mode"] = "client"

    kw["ssid"] = input("SSID: ").strip("\n")

    ret = input("Wifi-Protection (0=None,1=WEP,2=WPA2-PSK)[2]: ").strip("\n")
    if ret == "0":
        kw["security"] = "None"
    elif ret == "1":
        kw["security"] = "WEP"
        kw["wepkey"] = input("Wifi password: ").strip("\n")
    else:
        kw["security"] = "WPA2-PSK"
        kw["psk"] = input("Wifi password: ").strip("\n")

    ret = input("Use DHCP? (0=YES,1=NO)[0]: ").strip("\n")
    if ret == "1":
        kw["method"] = "static"
        ipaddr = IPv4Interface(
            input("IP Address (example: \"192.168.0.1/24\"): ").strip("\n"))
        kw["ipaddr"] = str(ipaddr.ip)
        kw["mask"] = int(ipaddr.with_prefixlen.split("/")[-1])
        gw = IPv4Address(input("Gateway: ").strip("\n"))
        kw["route"] = str(gw)
        kw["ns"] = input("DNS: ").strip("\n")
    else:
        kw["method"] = "dhcp"

    options = parse_network_config(**kw)
    task.config_network(options)


def do_set_password(task):
    new_pwd = getpass("New password: ")
    if getpass("Confirm new password: ") != new_pwd:
        raise RuntimeError("New password not match")

    print(task.set_password(new_pwd))


def do_get_ssid(task):
    try:
        print(task.get_ssid())
    except UsbTaskError:
        print("Wifi not connected")


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
    """ % {"name": task.name, "serial": task.serial,
           "pwd": task.has_password})

    resp = do_auth(task).decode("ascii", "ignore")
    if resp not in ("OK", "ALREADY_TRUSTED"):
        raise RuntimeError(resp)

    cmdline(task)

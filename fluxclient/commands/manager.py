
from getpass import getpass, getuser
from platform import platform
from uuid import UUID
import argparse
import readline  # noqa
import sys

from .misc import (get_or_create_default_key, setup_logger,
                   network_config_helper, start_usb_daemon)

PROG_DESCRIPTION = "Flux device manage tool allow user change device settings."
PROG_EPILOG = """Device manage tool support
  'Grant access permission from password'
  'Manage access control list'
  'Change device password'
  'Change device network'
"""


class AbortException(Exception):
    pass


def is_uart(target):
    if platform().startswith("Windows"):
        return target.startswith("COM") and target[3:].isdigit()
    else:
        return target.startswith("/")


def input_helper(txt):
    try:
        return input(txt)
    except KeyboardInterrupt:
        raise AbortException()


def quit_program(manager, logger):
    """Quit"""
    sys.exit(0)


def set_nickname(manager, logger):
    """Change device name"""
    name = input_helper("New device name: ").strip()
    if name:
        manager.set_nickname(name)
        logger.info("Done.")
    elif name == "":
        logger.error("Name can not be blank")


def set_password(manager, logger):
    """Change device password"""
    try:
        old_pass = getpass("Old password: ")
        new_pass = getpass("New password: ")
    except KeyboardInterrupt:
        raise AbortException()

    if getpass("Confirm new password: ") != new_pass:
        logger.error("New password not match")
        return
    if len(new_pass) < 3:
        logger.error("Password too short")
        return
    manager.modify_password(old_pass, new_pass)


def set_network(manager, logger):
    """Change network settings"""
    try:
        settings = network_config_helper.run()
    except KeyboardInterrupt:
        raise AbortException
    manager.modify_network(**settings)


def list_wifi(manager, logger):
    """Get wifi list"""
    logger.info("%17s %5s %10s %s", "bssid", "rssi", "security", "ssid")
    for r in manager.get_wifi_list():
        logger.info("%17s %5s %10s %s", r["bssid"], r["rssi"], r["security"],
                    r["ssid"])

    logger.info("--\n")


def add_trust(manager, logger):
    """Add an ID to trusted list"""

    filename = input_helper("Keyfile (keep emptry to use current session "
                            "key): ")
    if filename:
        with open(filename, "r") as f:
            manager.add_trust(getuser(), f.read())
    else:
        manager.add_trust(getuser(),
                          manager.client_key.public_key_pem.decode())

    logger.info("Key added.")


def list_trust(manager, logger):
    """List trusted ID"""

    logger.info("=" * 79)
    logger.info("%40s  %s", "access_id", "label")
    logger.info("-" * 79)
    for meta in manager.list_trust():
        logger.info("%40s  %s", meta["access_id"], meta.get("label"))
    logger.info("=" * 79 + "\n")


def remove_trust(manager, logger):
    """Remove trusted ID"""

    access_id = input_helper("Access id to remove (input 'all' to delete all "
                             "keys': ")

    if access_id == "all":
        for r in manager.list_trust():
            manager.remove_trust(r["access_id"])
            logger.info("Access ID %s REMOVED.\n", r["access_id"])
    elif access_id:
        manager.remove_trust(access_id)
        logger.info("Access ID %s REMOVED.\n", access_id)
    else:
        logger.info("Abort")


def get_network(manager, logger):
    """Get network informations"""
    ssid = manager.get_wifi_ssid()
    ipaddrs = manager.get_ipaddr()
    logger.info("\nWifi SSID: %s, IP Address: %s\n", ssid, ", ".join(ipaddrs))


def run_commands(manager, logger):
    from fluxclient.device.manager import ManagerError

    itasks = [
        ("nickname", set_nickname),
        ("password", set_password),
        ("trust", list_trust),
        ("addtrust", add_trust),
        ("rmtrust", remove_trust),
        ("network", set_network),
        ("listwifi", list_wifi),
        ("netstat", get_network),
        ("quit", quit_program),
    ]

    while True:
        logger.info("%10s   %s", "Command", "Help")
        logger.info("-" * 36)
        for cmd, fn in itasks:
            logger.info("%10s   %s", cmd, fn.__doc__)
        logger.info("")

        try:
            r = input("> ").strip()
            if not r:
                continue

            task = [fn for cmd, fn in itasks if cmd.startswith(r)]
            if len(task) == 1:
                try:
                    task[0](manager, logger)
                except AbortException:
                    logger.info("\n == Abort task, nothing changed. ==")
            elif len(task) > 1:
                logger.error("Command %r is ambiguous", r)
            else:
                logger.error("Unknow command: '%s'", r)
        except ManagerError as e:
            logger.error("Error '%s'", e)
        except KeyboardInterrupt as e:
            logger.info("\n")
            return

        logger.info("")


def fast_add_trust(manager, logger):
    from fluxclient.device.manager import ManagerError

    try:
        manager.add_trust(getuser(),
                          manager.client_key.public_key_pem.decode())
        logger.info("authorized.")
        return 0
    except ManagerError as e:
        logger.error("Error '%s'", e)
        return 1


def main(params=None):
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION,
                                     epilog=PROG_EPILOG)
    parser.add_argument(dest='target', type=str,
                        help="Device uuid or ipaddress to connect to. "
                             "IP address can be '192.168.1.1' or "
                             "'192.168.1.1:23811'.")
    parser.add_argument('--key', dest='client_key', type=str, default=None,
                        help='Client identify key (RSA key with pem format)')
    parser.add_argument('-a', '--auth-only', dest='auth_only',
                        action='store_const', const=True, default=False,
                        help='Do a quick authorize and exit rather then enter '
                             'the interaction shell')
    parser.add_argument('-p', '--password', dest='password', type=str,
                        help='Use password in argument instead. A password '
                             'prompt will not appear.')
    parser.add_argument('--verbose', dest='verbose', action='store_const',
                        const=True, default=False, help='Verbose output')

    options = parser.parse_args(params)
    logger = setup_logger(__name__, debug=options.verbose)

    from fluxclient.robot.misc import is_uuid
    from fluxclient.device import DeviceManager

    client_key = get_or_create_default_key(options.client_key)

    if is_uuid(options.target):
        manager = DeviceManager.from_uuid(client_key, UUID(hex=options.target))
    elif options.target == "usb":
        usbprotocol = start_usb_daemon(sys.stderr)
        manager = DeviceManager.from_usb(client_key, usbprotocol)
    elif is_uart(options.target):
        manager = DeviceManager.from_uart(client_key, options.target)
    else:
        manager = DeviceManager.from_ipaddr(client_key, options.target)

    if not manager.authorized:
        if options.password is None:
            password = getpass("Device Password: ")
        else:
            password = options.password
        manager.authorize_with_password(password)

    logger.info("\n"
                "Serial: %s (uuid={%s})\n"
                "Model: %s\n"
                "Version: %s\n"
                "Endpoint: %s\n", manager.serial, manager.uuid,
                manager.model_id, manager.version, manager.endpoint)

    if options.auth_only:
        return fast_add_trust(manager, logger)
    else:
        run_commands(manager, logger)


if __name__ == "__main__":
    sys.exit(main())


from getpass import getpass, getuser
from uuid import UUID
import argparse
import sys

from .misc import (get_or_create_default_key, setup_logger,
                   network_config_helper)

PROG_DESCRIPTION = "Flux device manage tool allow user change device settings."
PROG_EPILOG = """Device manage tool support
  'Grant access permission from password'
  'Manage access control list'
  'Change device password'
  'Change device network'
"""


def quit_program(manager, logger):
    """Quit"""
    sys.exit(0)


def change_device_name(manager, logger):
    """Change device name"""
    name = input("New device name: ").strip()
    if name:
        manager.rename(name)
        logger.error("Done.")
    else:
        logger.error("No name given.")


def change_device_password(manager, logger):
    """Change device password"""
    old_pass = getpass("Old password: ")
    new_pass = getpass("New password: ")
    if getpass("Confirm new password: ") != new_pass:
        logger.error("New password not match")
        return
    if len(new_pass) < 3:
        logger.error("Password too short")
        return
    manager.modify_password(old_pass, new_pass)


def change_network_settings(manager, logger):
    """Change network settings"""
    settings = network_config_helper.run()
    manager.modify_network(**settings)


def get_wifi_list(manager, logger):
    """Get wifi list"""
    logger.info("%17s %5s %23s %s", "bssid", "rssi", "security", "ssid")
    for r in manager.get_wifi_list():
        logger.info("%17s %5s %23s %s", r["bssid"], r["rssi"], r["security"],
                    r["ssid"])

    logger.info("--\n")


def add_trust(manager, logger):
    """Add an ID to trusted list"""

    filename = input("Keyfile (keep emptry to use current session key): ")
    if filename:
        with open(filename, "r") as f:
            aid = manager.add_trust(getuser(), f.read())
    else:
        aid = manager.add_trust(getuser(),
                                manager.client_key.public_key_pem.decode())

    logger.info("Key added with Access ID: %s\n", aid)


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
    access_id = input("Access id to remove: ")
    manager.remove_trust(access_id)
    logger.info("Access ID %s REMOVED.\n", access_id)


def run_commands(manager, logger):
    from fluxclient.device.manager import ManagerError

    tasks = [
        quit_program,
        change_device_name,
        change_device_password,
        change_network_settings,
        get_wifi_list,
        add_trust,
        list_trust,
        remove_trust,
    ]

    while True:
        logger.info("Manage tool: Choose task id")
        for i, t in enumerate(tasks):
            logger.info("  %i: %s", i, t.__doc__)
        logger.info("")

        try:
            r = input("> ").strip()
            if not r:
                continue

            try:
                i = int(r, 10)
                t = tasks[i]
            except (IndexError, ValueError):
                logger.error("Unknow task: '%s'", r)
                continue
            t(manager, logger)
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
        manager = DeviceManager(UUID(hex=options.target), client_key)
    else:
        manager = DeviceManager(UUID(int=0), client_key, ipaddr=options.target)

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
                "IP Address: %s\n", manager.serial, manager.uuid,
                manager.model_id, manager.version, manager.ipaddr)

    if options.auth_only:
        return fast_add_trust(manager, logger)
    else:
        run_commands(manager, logger)


if __name__ == "__main__":
    sys.exit(main())


from getpass import getpass
from uuid import UUID
import argparse
import sys

from fluxclient.upnp import UpnpTask, UpnpError

from .misc import (get_or_create_default_key, setup_logger, is_uuid,
                   network_config_helper)


def quit_program(upnp, logger):
    """Quit"""
    sys.exit(0)


def change_device_name(upnp, logger):
    """Change device name"""
    name = input("New device name: ").strip()
    if name:
        upnp.rename(name)
        logger.error("Done.")
    else:
        logger.error("No name given.")


def change_device_password(upnp, logger):
    """Change device password"""
    old_pass = getpass("Old password: ")
    new_pass = getpass("New password: ")
    if getpass("Confirm new password: ") != new_pass:
        logger.error("New password not match")
        return
    if len(new_pass) < 3:
        logger.error("Password too short")
        return
    upnp.modify_password(old_pass, new_pass)


def change_network_settings(upnp, logger):
    """Change network settings"""
    settings = network_config_helper.run()
    upnp.modify_network(**settings)


def get_wifi_list(upnp, logger):
    """Get wifi list"""
    logger.info("%17s %5s %23s %s", "bssid", "rssi", "security", "ssid")
    for r in upnp.get_wifi_list():
        logger.info("%17s %5s %23s %s", r["bssid"], r["rssi"], r["security"],
                    r["ssid"])

    logger.info("--\n")


def run_commands(upnp, logger):
    tasks = [
        quit_program,
        change_device_name,
        change_device_password,
        change_network_settings,
        get_wifi_list,
    ]

    while True:
        logger.info("Upnp tool: Choose task id")
        for i, t in enumerate(tasks):
            logger.info("  %i: %s", i, t.__doc__)
        logger.info("")

        try:
            r = input("> ").strip()
            if not r:
                continue

            i = int(r, 10)
            t = tasks[i]
            t(upnp, logger)
        except UpnpError as e:
            logger.error("Error %s", e)
        except KeyboardInterrupt as e:
            logger.info("\n")
            return


def main():
    parser = argparse.ArgumentParser(description='Flux upnp tool')
    parser.add_argument(dest='target', type=str,
                        help='Device UUID or IP Address')
    parser.add_argument('--debug', dest='debug', action='store_const',
                        const=True, default=False, help='Print debug message')
    parser.add_argument('--key', dest='client_key', type=str, default=None,
                        help='Client identify key (RSA key with pem format)')

    options = parser.parse_args()

    logger = setup_logger(__name__, debug=options.debug)
    client_key = get_or_create_default_key(options.client_key)

    backend_options = {
        "password": lambda _: getpass("Device Password: ")
    }

    if is_uuid(options.target):
        upnp = UpnpTask(UUID(hex=options.target), client_key,
                        backend_options=backend_options)
    else:
        upnp = UpnpTask(UUID(int=0), client_key, ipaddr=options.target,
                        backend_options=backend_options)

    logger.info("\n"
                "Serial: %s (uuid={%s})\n"
                "Model: %s\n"
                "Version: %s\n"
                "IP Address: %s", upnp.serial, upnp.uuid, upnp.model_id,
                upnp.version, upnp.ipaddr)
    logger.debug("Backend: %s", upnp._backend)
    logger.info("\n")

    run_commands(upnp, logger)


if __name__ == "__main__":
    sys.exit(main())

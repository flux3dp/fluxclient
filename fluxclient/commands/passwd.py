
from getpass import getpass
from uuid import UUID
import argparse
import sys

from fluxclient.commands.misc import get_or_create_default_key
from fluxclient.upnp.task import UpnpTask


def main():
    parser = argparse.ArgumentParser(description='flux printer config tool')
    parser.add_argument(dest='uuid', type=str, help='Printer UUID')
    parser.add_argument('--key', dest='clientkey', type=str, default=None,
                        help='Client identify key (A RSA pem)')

    options = parser.parse_args()
    options.clientkey = get_or_create_default_key(options.clientkey)

    uuid = UUID(hex=options.uuid)
    task = UpnpTask(uuid, options.clientkey)

    sys.stdout.write("""UUID: %s
Serial: %s
Model: %s
Has Password: %s
""" % (task.uuid.hex, task.serial, task.model_id,
       task.has_password and "YES" or "NO"))
    sys.stdout.flush()

    task.require_auth()

    if task.has_password:
        old_password = getpass("Old password required: ")
    else:
        old_password = ""

    new_password = getpass("New password: ")
    confirm_password = getpass("Conrim new password: ")

    assert len(new_password) > 3, "Password too short"
    assert new_password == confirm_password, "Password not match"

    for i in range(3):
        resp = task.passwd(new_password, old_password)
        if "ts" in resp:
            print("Password changed.")
            return 0

    print("Remote no response")
    return 2


if __name__ == "__main__":
    sys.exit(main())


from getpass import getpass
import argparse
import sys

from fluxclient.upnp_task import UpnpTask


def main():
    parser = argparse.ArgumentParser(description='flux printer config tool')
    parser.add_argument(dest='serial', type=str, help='Printer Serial')

    options = parser.parse_args()

    serial = options.serial
    task = UpnpTask(serial)

    sys.stdout.write("""Serial: %s
Model: %s
Has Password: %s
""" % (task.serial.hex, task.model_id, task.has_password and "YES" or "NO"))
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
        if resp:
            if resp.get("status") == "ok":
                print("Password changed.")
                return 0
            else:
                print("Password change failed: %s" % resp.get("message", ""))
                return 1

    print("Remote no response")
    return 2


if __name__ == "__main__":
    sys.exit(main())

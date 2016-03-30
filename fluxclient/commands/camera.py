
from datetime import datetime
import argparse
import sys

from fluxclient.commands.misc import (get_or_create_default_key,
                                      get_camera_endpoint)
from fluxclient.robot import connect_camera


def connect(options):
    endpoint, device = get_camera_endpoint(options.target, options.clientkey)

    def conn_callback(*args):
        sys.stdout.write(".")
        sys.stdout.flush()
        return True

    camera = connect_camera(endpoint=endpoint, device=device,
                            client_key=options.clientkey,
                            conn_callback=conn_callback)

    return device, camera


def serve_forever(options, device, camera):
    if device:
        dataset = {"name": device.name, "uuid": device.uuid.hex,
                   "ip": device.endpoint[0], "model": device.model_id,
                   "serial": device.serial, "target": options.target}
    else:
        dataset = {"name": "unknown", "uuid": "unknown", "ip": "unknown",
                   "model": "unknown", "serial": "unknown",
                   "target": options.target}

    def callback(c, imagebuf):
        ts = datetime.now().strftime(options.strftime)
        dataset["time"] = ts
        filename = options.filename % dataset
        with open(filename, "wb") as f:
            f.write(imagebuf)
            print(">> %s" % filename)

    camera.capture(callback)


def main():
    parser = argparse.ArgumentParser(description='flux camera live')

    parser.add_argument(dest='target', type=str,
                        help="Printer connect with. It can be printer uuid "
                             "or IP address like 192.168.1.1 or "
                             "192.168.1.1:23811")
    parser.add_argument('--filename', dest='filename', type=str,
                        default="%(target)s-%(time)s.jpg",
                        help='Save photo as filename')
    parser.add_argument('--strftime', dest='strftime', type=str,
                        default="%Y-%m-%d-%H-%M-%S-%f",
                        help='Formate datetime for filename')
    parser.add_argument('--key', dest='clientkey', type=str, default=None,
                        help='Client identify key (A RSA pem)')
    options = parser.parse_args()
    options.clientkey = get_or_create_default_key(options.clientkey)

    device, camera = connect(options)

    try:
        serve_forever(options, device, camera)
    finally:
        camera.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

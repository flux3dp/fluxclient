
from datetime import datetime
import argparse
import sys

from fluxclient.commands.misc import (get_or_create_default_key,
                                      connect_camera_helper)


def serve_forever(options, camera, metadata={}):
    dataset = {
        "name": metadata.get("name", "unknown"),
        "uuid": metadata["uuid"].hex if "uuid" in metadata else "unknown",
        "ip": metadata.get("ipaddr", "unknown"),
        "model": metadata.get("model_id", "unknown"),
        "serial": metadata.get("serial", "unknown"),
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
                             "192.168.1.1:23812")
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

    camera, device = connect_camera_helper(options.target, options.clientkey)

    try:
        if device:
            metadata = device.to_dict()
        else:
            metadata = {"ipaddr": options.target}
        serve_forever(options, camera, metadata)
    finally:
        camera.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

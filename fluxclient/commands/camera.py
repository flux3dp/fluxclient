
from collections import deque
from datetime import datetime
from time import time, sleep
import argparse
import sys
import os

from fluxclient.commands.misc import (get_or_create_default_key,
                                      connect_camera_helper)
from .misc import setup_logger

PROG_DESCRIPTION = "Fetch flux device camera stream and save as file"
PROG_EPILOG = ""


class Static(object):
    def __init__(self):
        self.total = 0
        self.timestamp = deque(maxlen=64)

    @property
    def fps(self):
        now = time()
        edge = None
        total = 0
        count = 0
        for t in self.timestamp:
            if edge is None:
                edge = t
            total += t
            count += 1
        if count > 2:
            return count / (now - edge)
        else:
            return float("nan")

    def log(self, filename):
        self.total += 1
        self.timestamp.append(time())
        sys.stdout.write(
            "\rRecived=%i FPS=%4.1f %s" %
            (self.total, self.fps, filename))


def request_frame_sender(camera, period):
    from threading import Thread

    def loop():
        try:
            while camera.fileno():
                camera.require_frame()
                sleep(period)
        except OSError:
            pass

    t = Thread(target=loop)
    t.daemon = True
    t.start()
    return t


def serve_forever(options, camera, logger, metadata={}, oneshot=False):
    dataset = {
        "name": metadata.get("name", "unknown"),
        "uuid": metadata["uuid"].hex if "uuid" in metadata else "unknown",
        "ip": metadata.get("ipaddr", "unknown"),
        "model": metadata.get("model_id", "unknown"),
        "serial": metadata.get("serial", "unknown"),
        "target": options.target}

    static = Static()

    def callback(c, imagebuf):
        ts = datetime.now().strftime(options.strftime)
        dataset["time"] = ts
        dataset["index"] = static.total

        filename = os.path.join(options.path, options.filename % dataset)

        with open(filename, "wb") as f:
            f.write(imagebuf)
            static.log(filename)
        if oneshot:
            c.abort()

    if options.fps:
        p = 1 / options.fps
        if p > 0:
            request_frame_sender(camera, p)
        else:
            raise RuntimeError("Bad fps: %s" % options.fps)
    else:
        camera.enable_streaming()

    sys.stdout.write("Start streaming...\n\n")

    try:
        camera.capture(callback)
    except KeyboardInterrupt:
        sys.stdout.write("\nStopping...\n")
    finally:
        camera.close()


def main(params=None, oneshot=False):
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION,
                                     epilog=PROG_EPILOG)

    parser.add_argument(dest='target', type=str,
                        help="Printer connect with. It can be printer uuid "
                             "or IP address like 192.168.1.1 or "
                             "192.168.1.1:23812")
    parser.add_argument('--key', dest='clientkey', type=str, default=None,
                        help='Client identify key (A RSA pem)')
    parser.add_argument('-p', '--fps', dest='fps', type=float, default=None,
                        help='Limit photo FPS')
    parser.add_argument('--strftime', dest='strftime', type=str,
                        default="%Y-%m-%d-%H-%M-%S-%f",
                        help='Formate datetime for filename')
    parser.add_argument('-f', '--filename', dest='filename', type=str,
                        default="%(target)s-%(time)s.jpg",
                        help='Save photo as filename')
    parser.add_argument('--path', dest='path', type=str, default="",
                        help='Path to save photo')
    options = parser.parse_args(params)
    options.clientkey = get_or_create_default_key(options.clientkey)

    logger = setup_logger(__name__, debug=False)
    camera, device = connect_camera_helper(options.target, options.clientkey)

    try:
        if device:
            metadata = device.to_dict()
        else:
            metadata = {"ipaddr": options.target}
        serve_forever(options, camera, logger, metadata, oneshot=oneshot)
    finally:
        camera.close()

    return 0

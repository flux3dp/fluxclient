
from tempfile import NamedTemporaryFile
import datetime
import argparse
import logging
import sys
import os

from fluxclient.robot.misc import require_robot
from fluxclient.robot import connect_robot


logging.basicConfig(format="%(message)s", stream=sys.stdout)
logger = logging.getLogger('')


def prepare_robot(endpoint, server_key):
    def conn_callback(*args):
        sys.stdout.write(".")
        sys.stdout.flush()
        return True
    robot = connect_robot(endpoint, server_key, conn_callback)
    pos = robot.position()
    if pos == "CommandTask":
        robot.begin_scan()
    elif pos == "ScanTask":
        pass
    else:
        raise RuntimeError("Unknow position: %s" % pos)

    return robot


def interactive(robot):
    logger.info("Type 'i' (image) to get a screenshot")
    logger.info("Type 'g' (go) to start progress")

    while True:
        sys.stdout.write("> ")
        sys.stdout.flush()
        l = sys.stdin.readline()
        if l.startswith("i"):
            logger.info("Get an image...")
            images = robot.oneshot()
            tempfiles = []
            for mime, buf in images:
                ntf = NamedTemporaryFile(suffix=".jpg", delete=False)
                ntf.write(buf)
                tempfiles.append(ntf)

            os.system("open " + " ".join([n.name for n in tempfiles]))
        elif l.startswith("g"):
            logger.info("Go!")
            return
        else:
            logger.info("Type 'i' (image) to get a screenshot")
            logger.info("Type 'g' (go) to start progress")


def print_progress(step, total):
    left = int((step / total) * 70)
    right = 70 - left
    sys.stdout.write("\r[%s>%s] Step %3i\n" % ("=" * left, " " * right, step))
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description='flux scanner')

    parser.add_argument('-a', dest='auto', action='store_const',
                        const=True, default=False, help='Start with no asking')
    parser.add_argument('-d', dest='debug', action='store_const',
                        const=True, default=False, help='Print debug log')
    parser.add_argument('--path', dest='dist', type=str, default=".",
                        help="Where to save images")
    parser.add_argument('--prefix', dest='prefix', type=str, default=None,
                        help="Image filename prefix")
    parser.add_argument(dest='target', type=str,
                        help="Printer connect with. It can be printer serial "
                             "or IP address like 192.168.1.1 or "
                             "192.168.1.1:23811")
    options = parser.parse_args()

    if options.debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    endpoint, server_key = require_robot(options.target)
    robot = prepare_robot(endpoint, server_key)

    if not options.auto:
        interactive(robot)

    filename_prefix = options.prefix or \
        datetime.datetime.now().strftime("scan_%Y%m%d_%H%M")

    filename_prefix = os.path.join(options.dist, filename_prefix)

    logger.info("Image will save to %s*" % filename_prefix)

    for step in range(400):
        print_progress(step, 400)
        images = robot.scanimages()
        robot.scan_next()
        for i in range(len(images)):
            mime, buf = images[i]
            filename = "%s_%03i_%i.jpg" % (filename_prefix, step, i)
            with open(filename, "wb") as f:
                f.write(buf)

    sys.stdout.write("\n Done.")
    return 0


from tempfile import NamedTemporaryFile
import datetime
import argparse
import logging
import sys
from time import sleep
import os

from fluxclient.robot.misc import require_robot
from fluxclient.robot import connect_robot
from fluxclient.hw_profile import HW_PROFILE


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


def logger_info(logger):
    logger.info("Type 'i' (image) to get a screenshot")
    logger.info("Type 'g' (go) to start progress")
    logger.info("Type 'L' (Left) toggle Left Laser")
    logger.info("Type 'R' (Right) toggle Right Laser")
    logger.info("Type 'S[number]' (Step) to mave [number] step")
    logger.info("Type 'C[length]' (Change) change step length for each step")
    logger.info("Type 'T[steps]' Set total steps")
    logger.info("Type 'check' to check proper camera setting")
    logger.info("Type 'quit', 'exit' to leave")


def interactive(robot):
    logger_info(logger)
    total_steps = None
    laser_Left = False
    laser_Right = False
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
            return total_steps

        elif l.startswith('S'):
            l = l.rstrip('\n')
            if len(l) == 1:
                robot.scan_next()

            else:
                try:
                    step = int(l[1:])
                except:
                    step = 1
                for i in range(step):
                    robot.scan_next()

        elif l.startswith('T'):
            l = l.rstrip("\n")
            total_steps = int(l[1:])

        elif l.startswith('C'):
            l = l.rstrip('\n')
            try:
                step_len = float(l[1:])
            except:
                step_len = 180

            robot.set_scanlen(step_len)

        elif l.startswith('L'):
            laser_Left = not laser_Left
            robot.scan_laser(laser_Left, laser_Right)
        elif l.startswith('R'):
            laser_Right = not laser_Right
            robot.scan_laser(laser_Left, laser_Right)
        elif l.startswith('q') or l.startswith('e'):
            robot.quit_task()
            robot.close()
            os._exit(0)
        elif l.startswith('c'):
            res = robot.scan_check()
            print(res)
        else:
            logger_info(logger)


def print_progress(step, total):
    left = int((step / total) * 70)
    right = 70 - left
    sys.stdout.write("\r[%s>%s] Step %3i" % ("=" * left, " " * right, step))
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
                        help="Printer connect with. It can be printer UUID "
                             "or IP address like 192.168.1.1 or "
                             "192.168.1.1:23811")
    options = parser.parse_args()

    if options.debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    endpoint, server_key = require_robot(options.target)
    robot = prepare_robot(endpoint, server_key)
    robot.set_scanlen(HW_PROFILE['model-1']['scan_full_len'] / 400.)

    if not options.auto:
        total_steps = interactive(robot)

    filename_prefix = options.prefix or \
        datetime.datetime.now().strftime("scan_%Y%m%d_%H%M")
    filename_prefix = os.path.join(options.dist, filename_prefix)
    filename_prefix = os.path.join(options.dist, '')
    print(filename_prefix)

    logger.info("Image will save to %s*.jpg" % filename_prefix)
    suffix = ['L', 'R', 'O']
    images = robot.scanimages()
    for step in range(total_steps):
        print_progress(step, total_steps)
        images = robot.scanimages()
        robot.scan_next()
        for i in range(len(images)):
            mime, buf = images[i]
            filename = "%s_%03i_%i.jpg" % (filename_prefix, step, i)
            filename = "%s%03i_%s.jpg" % (filename_prefix, step, suffix[i])
            with open(filename, "wb") as f:
                f.write(buf)

    sys.stdout.write("\n Done.")
    return 0

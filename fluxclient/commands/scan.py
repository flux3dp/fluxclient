
from tempfile import NamedTemporaryFile
import datetime
import argparse
import sys
import os

from fluxclient.commands.misc import (get_or_create_default_key,
                                      connect_robot_helper, setup_logger)
from fluxclient.hw_profile import HW_PROFILE

PROG_DESCRIPTION = "Flux raw scan tool"
PROG_EPILOG = ""
logger = None


def prepare_robot(options):
    robot, _ = connect_robot_helper(options.target, options.clientkey)

    status = robot.report_play()
    if status["st_id"] == 0:
        return robot.scan()
    else:
        raise RuntimeError("Unknow status: %s" % status)


def print_help():
    logger.info("""
Type 'i' (image) to get a screenshot
Type 'g' (go) to start progress
Type 'L' (Left) toggle Left Laser
Type 'R' (Right) toggle Right Laser
Type 'S[number]' (Step) to mave [number] step
Type 'C[length]' (Change) change step length for each step
Type 'T[steps]' Set total steps
Type 'B' show calibration
Type 'check' to check proper camera setting
Type 'do calibrate' to do calibrate
Type 'quit', 'exit' to leave""")


def interactive(scan):
    print_help()

    total_steps = None
    laser_left = False
    laser_right = False

    while True:
        l = input("> ")
        if l.startswith("i"):
            logger.info("Get an image...")
            images = scan.oneshot()
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
                scan.forward()

            else:
                try:
                    step = int(l[1:])
                except:
                    step = 1
                if step > 0:
                    for i in range(step):
                        scan.forward()
                else:
                    for i in range(abs(step)):
                        scan.backward()

        elif l.startswith('T'):
            l = l.rstrip("\n")
            total_steps = int(l[1:])

        elif l.startswith('C'):
            l = l.rstrip('\n')
            try:
                step_len = float(l[1:])
            except:
                step_len = 180

            scan.step_length(step_len)

        elif l.startswith('L'):
            laser_left = not laser_left
            scan.laser(laser_left, laser_right)
        elif l.startswith('R'):
            laser_right = not laser_right
            scan.laser(laser_left, laser_right)
        elif l.startswith('q') or l.startswith('e'):
            scan.quit()
            os._exit(0)
        elif l.startswith('c'):
            logger.info("Checking camera...")
            logger.info("Done: %s", scan.check_camera())
            logger.info('(0: closed, 1: open, 3: open and find chess board)')

        elif l.startswith('d'):
            logger.info("Calibrating...")
            logger.info("Done: %s", scan.calibrate())

        elif l.startswith('B'):
            logger.info("Calibrate: %s", scan.get_calibrate())

        else:
            logger.info("Unknow command")
            print_help()


def print_progress(step, total):
    left = int((step / total) * 70)
    right = 70 - left
    sys.stdout.write("\r[%s>%s] Step %3i" % ("=" * left, " " * right, step))
    sys.stdout.flush()


def main(params=None):
    global logger
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION,
                                     epilog=PROG_EPILOG)
    parser.add_argument(dest='target', type=str,
                        help="Printer connect with. It can be printer UUID "
                             "or IP address like 192.168.1.1 or "
                             "192.168.1.1:23811")
    parser.add_argument('--key', dest='clientkey', type=str, default=None,
                        help='Client identify key (A RSA pem)')
    parser.add_argument('--auto', dest='auto', action='store_const',
                        const=True, default=False, help='Start with no asking')
    parser.add_argument('--verbose', dest='verbose', action='store_const',
                        const=True, default=False, help='Verbose output')
    parser.add_argument('--steps', dest='steps', type=int, default=400,
                        help='Set steps for scan')
    parser.add_argument('--path', dest='dist', type=str, default=".",
                        help="Where to save images")
    parser.add_argument('--prefix', dest='prefix', type=str, default=None,
                        help="Image filename prefix")
    options = parser.parse_args(params)
    logger = setup_logger(__name__, debug=options.verbose)
    options.clientkey = get_or_create_default_key(options.clientkey)

    scan = prepare_robot(options)
    scan.step_length(HW_PROFILE['model-1']['scan_full_len'] / options.steps)

    if not options.auto:
        total_steps = interactive(scan)

    filename_prefix = options.prefix or \
        datetime.datetime.now().strftime("scan_%Y%m%d_%H%M")
    filename_prefix = os.path.join(options.dist, filename_prefix)
    filename_prefix = os.path.join(options.dist, '')

    logger.info("Image will save to %s*.jpg" % filename_prefix)
    suffix = ['L', 'R', 'O']
    images = scan.scanimages()
    total_steps = options.steps

    for step in range(total_steps):
        print_progress(step, total_steps)
        images = scan.scanimages()
        for i in range(len(images)):
            mime, buf = images[i]
            filename = "%s_%03i_%i.jpg" % (filename_prefix, step, i)
            filename = "%s%03i_%s.jpg" % (filename_prefix, step, suffix[i])
            with open(filename, "wb") as f:
                f.write(buf)
        if step > 0:
            scan.forward()

    scan.quit()
    print_progress(total_steps, total_steps)
    sys.stdout.write("\n Done.")
    return 0

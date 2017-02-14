
import readline
import argparse
import atexit
import sys
import os

from fluxclient.commands.misc import (setup_logger,
                                      get_or_create_default_key,
                                      connect_robot_helper)


PROG_DESCRIPTION = "Flux device control console."
PROG_EPILOG = ""
PROG_SHELL_HELP = """
There are three default shell options for user:

  'simple' - This is a simple interactive shell for user.
  'curses' - Curses is an advance interactive shell but it has some bug.
  'ipython' - If you have IPython installed, this shell will invoke IPython \
shell after connected. You can use this shell to test your codes with device \
control.

Shell also support user custom shell. Here is an example:

1. Source code layout:

    myproject/
        __init__.py
        my_robot_shell.py

2. my_robot_shell.py source code:

    def my_shell(robot, device=None):
        # Note:
        # 'robot' argument is an instance of \
:class:`fluxclient.robot.robot.Robot`
        # 'device' argument is an instance of \
:class:`fluxclient.device.device.Device`. This argument maybe None if user \
give an IPAddress rather then an UUID.
        print("Here is connected robot object: %s" % robot)
        return 0

3. Run 'flux_robot {UUID} --shell myproject.my_robot_shell.my_shell'

"""


def setup_readline():
    histfile = os.path.join(os.path.expanduser("~"), ".flux_robot_history")
    try:
        readline.read_history_file(histfile)
    except IOError:
        pass
    atexit.register(readline.write_history_file, histfile)


def simple_shell(options):
    logger = setup_logger("Robot", debug=options.verbose)
    setup_readline()

    def conn_callback(*args):
        sys.stdout.write(".")
        sys.stdout.flush()
        return True

    client, _ = connect_robot_helper(options.target, options.clientkey)

    from fluxclient.commands.misc.robot_console import RobotConsole
    robot_client = RobotConsole(client)
    logger.info("----> READY")
    logger.info("Use command 'help' to get available commands.")

    while True:
        try:
            r = input("ROBOT> ")
        except KeyboardInterrupt:
            logger.info("Quit")
            return

        if not r:
            continue
        try:
            robot_client.on_cmd(r.strip())
        except Exception:
            logger.exception("Unhandle Error")
            return


def curses_shell(options):
    from fluxclient.commands.misc.console import Console

    with Console() as console:
        console.setup()
        setup_readline()

        def conn_callback(*args):
            console.write(".")
            return True

        try:
            logger = setup_logger("Robot", console, options.verbose)
            client, _ = connect_robot_helper(options.target, options.clientkey,
                                             console)

            from fluxclient.commands.misc.robot_console import RobotConsole
            robot_client = RobotConsole(client)
            logger.info("----> READY")
            logger.info("Use command 'help' to get available commands.")
            while True:
                try:
                    cmd = console.read_cmd()
                    if cmd:
                        robot_client.on_cmd(cmd)
                except Exception:
                    logger.exception("Unhandle Error")

        except Exception:
            logger.exception("Unhandle Error")
            console.append_log("Press Control + C to quit")
            while True:
                cmd = console.read_cmd()


def python_shell(options):
    logger = setup_logger("Robot", debug=options.verbose)

    def conn_callback(*args):
        sys.stdout.write(".")
        sys.stdout.flush()
        return True

    if options.shell == "ipython":
        import IPython
    else:
        import importlib
        sys.path.append(os.path.abspath(""))
        module_name, entrance_name = options.shell.rsplit(".", 1)
        module_instance = importlib.import_module(module_name)
        entrance = module_instance.__getattribute__(entrance_name)

    robot, device = connect_robot_helper(options.target, options.clientkey)

    if options.shell == "ipython":
        logger.info("----> READY")
        logger.info("""
      * Hint: Try 'robot?' and 'dir(robot)' to get more informations)\n""")
        IPython.embed()
        return 0
    else:
        return entrance(robot, device)


def help_shell(options):
    logger = setup_logger("Robot", debug=options.verbose)
    logger.info(PROG_SHELL_HELP)


def main(params=None):
    parser = argparse.ArgumentParser(description=PROG_DESCRIPTION,
                                     epilog=PROG_EPILOG)

    parser.add_argument(dest='target', type=str,
                        help="Device uuid or ipaddress to connect to. "
                             "IP address can be '192.168.1.1' or "
                             "'192.168.1.1:23811'.")
    parser.add_argument('--key', dest='clientkey', type=str, default=None,
                        help='Client identify key (A RSA pem)')
    parser.add_argument('-s', '--shell', dest='shell', default='simple',
                        help="Shell will active after connect to device. "
                             "use 'flux_robot --shell help' to get more "
                             "informations.")
    parser.add_argument('--verbose', dest='verbose', action='store_const',
                        const=True, default=False, help='Verbose output')
    options = parser.parse_args(params)
    options.clientkey = get_or_create_default_key(options.clientkey)

    if options.shell == "simple":
        simple_shell(options)
    elif options.shell == "curses":
        curses_shell(options)
    elif options.shell == "help":
        help_shell(options)
    else:
        python_shell(options)

    return 0

if __name__ == "__main__":
    sys.exit(main())

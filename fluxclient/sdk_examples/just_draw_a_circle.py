from math import sin, cos, pi
from io import BytesIO, StringIO
from time import sleep

from fluxclient.fcode.g_to_f import GcodeToFcode
from fluxclient.robot import FluxRobot
from fluxclient.commands.misc import get_or_create_default_key


def circle_generate(r):
    gcode = []
    for i in range(101):
        theta = 2 * pi / 100 * i
        gcode.append("G1 F500 X{} Y{}\n".format(round(r * cos(theta), 4), round(r * sin(theta), 4)))
    return gcode


def send_task(fcode_stream):
    # open the rsa key
    client_key = get_or_create_default_key("./sdk_connection.pem")

    # connect to delta, don't change the port 23811
    robot = FluxRobot(('192.168.18.114', 23811), client_key)

    # callback for uploading
    def upload_progress(*args):
        print(args)

    # upload fcode in delta
    robot.upload_stream(fcode_stream, 'application/fcode', size=len(fcode_stream.getvalue()), process_callback=upload_progress)

    # start the task
    robot.start_play()

    # report progress
    report = robot.report_play()
    while report['st_id'] != 64:
        report = robot.report_play()
        print('report:', report)
        sleep(1)

    # complete, quit the task
    robot.quit_play()


def main():
    r = 50
    gcode = StringIO()
    fcode_output = BytesIO()

    gcode.write('G1 F5000 X{} Y0 Z10\n'.format(r))
    for line in circle_generate(r):
        gcode.write(line)
    gcode.seek(0)

    m_GcodeToFcode = GcodeToFcode(head_type='N/A')
    m_GcodeToFcode.process(gcode, fcode_output)
    fcode_output.seek(0)

    send_task(fcode_output)

if __name__ == '__main__':
    main()


import unittest
from fluxclient.toolpath import _toolpath


class TestGCodeParser(unittest.TestCase):
    def setUp(self):
        self.proc = _toolpath.PyToolpathProcessor(self.append_command)
        self.parser = _toolpath.GCodeParser()
        self.parser.set_processor(self.proc)
        self.calllist = None

    def append_command(self, cmd, **kw):
        if self.calllist:
            ccmd, ckw = self.calllist.pop(0)
            if ccmd != cmd:
                raise AssertionError("%r != %r (Recv=%s, Act=%s)" % (
                    cmd, ccmd, kw, ckw))
            self.assertDictContainsSubset(ckw, kw)
        else:
            raise AssertionError("Recv unexcept command: %r: %s" % (cmd, kw))

    def test_g0g1(self):
        self.calllist = [
            ("moveto", {'flags': 112, 'feedrate': 9000.0,
                        'x': 50.5, 'y': 50.0}),
            ("append_comment", {'message': "YAHOO"})
        ]
        self.parser.parse_command(b"G1F9000 X50.5 Y50 ;YAHOO\n")
        self.assertEqual([], self.calllist)

    def test_x2(self):
        self.calllist = [
            ("set_toolhead_pwm", {'strength': 0}),
            ("append_comment", {'message': "YAHOO"}),
            ("set_toolhead_pwm", {'strength': 1}),
            ("append_comment", {'message': "YAHOO"}),
        ]
        self.parser.parse_command(b"X2O0  ;YAHOO\n")
        self.parser.parse_command(b"X2O255;YAHOO\n")
        self.assertEqual([], self.calllist)

    def test_comment(self):
        self.calllist = [
            ("append_comment", {'message': "YAHOO"}),
        ]
        self.parser.parse_command(b";YAHOO\n")
        self.assertEqual([], self.calllist)


class TestGCodeWriter(unittest.TestCase):
    def setUp(self):
        self.proc = _toolpath.GCodeMemoryWriter()

    def test_movement(self):
        self.proc.moveto(feedrate=6000, x=128)
        self.proc.moveto(x=64)
        self.proc.append_comment("DATATA")
        self.proc.terminated()
        self.assertEqual(
            self.proc.get_buffer(),
            b'G1 F6000.0000 X128.0000\nG1 X64.0000\n;DATATA\n')

    def test_sleep_2100p(self):
        self.proc.sleep(2.1)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'G4 P2100\n')

    def test_sleep_2s(self):
        self.proc.sleep(2.0)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'G4 S2\n')

    def test_sleep_1p(self):
        self.proc.sleep(0.001)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'G4 P1\n')

    def test_enable_motor(self):
        self.proc.enable_motor()
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'M17\n')

    def test_disable_motor(self):
        self.proc.disable_motor()
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'M84\n')

    def test_pause_m25(self):
        self.proc.pause(False)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'M25\n')

    def test_pause_m226(self):
        self.proc.pause(True)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'M226\n')

    def test_home(self):
        self.proc.home()
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'G28\n')

    def test_set_toolhead_heater_temperature_nowait(self):
        self.proc.set_toolhead_heater_temperature(200, False)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'M104 S200.0\n')

    def test_set_toolhead_heater_temperature_wait(self):
        self.proc.set_toolhead_heater_temperature(200, True)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'M109 S200.0\n')

    def test_set_toolhead_fan_speed_0(self):
        self.proc.set_toolhead_fan_speed(0)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'M107\n')

    def test_set_toolhead_fan_speed_0_01(self):
        self.proc.set_toolhead_fan_speed(0.01)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'M106 S2\n')

    def test_set_toolhead_fan_speed_1(self):
        self.proc.set_toolhead_fan_speed(1.0)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'M106 S255\n')

    def test_set_toolhead_pwm_0(self):
        self.proc.set_toolhead_pwm(0)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'X2O0\n')

    def test_set_toolhead_pwm_0_01(self):
        self.proc.set_toolhead_pwm(0.01)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'X2O2\n')

    def test_set_toolhead_pwm_1(self):
        self.proc.set_toolhead_pwm(1)
        self.proc.terminated()
        self.assertEqual(self.proc.get_buffer(), b'X2O255\n')

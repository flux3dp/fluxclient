
import tempfile
import unittest

from fluxclient.commands import fcode as fcode_cli


class FcodeCliTest(unittest.TestCase):
    def test_g28_g2f_f2g(self):
        source_swap = tempfile.NamedTemporaryFile()
        source_swap.write(b"\n\nG28\n\n")
        source_swap.flush()

        fcode_swap = tempfile.NamedTemporaryFile()
        gcode_swap = tempfile.NamedTemporaryFile()
        fcode_cli.gcode_2_fcode(["-i", source_swap.name,
                                 fcode_swap.name])

        fcode_cli.fcode_2_gcode(["-i", fcode_swap.name, gcode_swap.name])
        self.assertTrue("G28" in gcode_swap.read().decode("utf8").split("\n"))

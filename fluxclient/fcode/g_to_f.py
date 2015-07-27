import logging

from fluxclient.fcode.fcode_base import fcode_base
from fluxclient.hw_profile import HW_PROFILE


logger = logging.getLogger("g_to_f")


class gcode_to_fcode(fcode_base):
    """transform from gcode to fcode
    fcode format: https://github.com/flux3dp/fluxmonitor/wiki/Flux-Device-Control-Describe-File-V1

    this should done several thing:
      check boundary problem
      get metadata
      long path will split into many different command in order to support emergency stop

    """
    def __init__(self, model):
        super(gcode_to_fcode, self).__init__()

        if model not in HW_PROFILE:
            logger.info("Undefine model:%d , using 'model-1'instead" % (model))
            model = "model-1"

        self.r = HW_PROFILE[model]["radius"]
        self.split_thres = 1
        a = self.r[1:1 + 2]
        x = x * 2 + 1

    def function(self):
        pass


from __future__ import absolute_import

from mimetypes import add_type, guess_type

add_type("text/gcode", ".gcode")
add_type("application/fcode", ".fcode")


def validate_ext(filename, match_mimetype):
    real_mimetype, _ = guess_type(filename)
    return real_mimetype == match_mimetype

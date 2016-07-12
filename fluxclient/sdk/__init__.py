# TCP Connection part:
#   Send Command:
#     (INDEX, CMD_CODE, params_0, params_1, ..., keywords_dict)
#   Recv Message:
#     Cmd error
#     *NOTE: command will not be queued, cmd index will not shift either
#       (0xff, CMD_INDEX, ERROR_CODE)
#     Exec result:
#       (0x00 ~ 0xfe, params_0, params_1, ..., keywords_dict)
#
# Example:
# SEND -> (0, CMD_G028)
# SEND -> (1, CMD_G001, {"X": 3.2, "F": 10})     # Correct command
# RECV <- (0xff, 1, 0x01)                        # 0xff: ERROR occour
#                                                # 1: Command index 1
#                                                # 0x01: OPERATION_ERROR
# RECV <- (CMD_G028, 0, 20.0, 30.0, 20.0)        # G28 result
#
# SEND -> (1, CMD_G001, {"X": 3.2, "F": 10})     # Correct command
# SEND -> (2, CMD_G001, {"X": 100000, "F": 10})  # X overlimit
# RECV <- (0xff, 1, 0x01)
#
# SEND -> (2, CMD_G028)
# RECV <- (CMD_G028, 1, nan, nan, nan)           # G28 failed
#
#
# UDP Connection part
#   (i:0, s:SALT, i:cmd_index, i:queued_size)
#       * First 0 means it is a status message
#       * SALT: not for use right now
#       * cmd_index: next command index should given
#       * queued_size: command is waitting to be execute in system
#
#   (i:1, s:SALT, s:module_type, i:errno, )


CMD_G001 = 0x01
# Move position
# ({i:F, f:X, f:Y, f:Z, f:E1, f:E2 , f:E3})
#
# E1, E2, E3 can not set at same time
# Dict must contains at least 1 key/value, otherwhise will get operation error

CMD_G004 = 0x02
# Sleep (seconds)
# (f:seconds)
#
# seconds must be positive

CMD_SLSR = 0x03
# Control red scan laser (On/Off)
# (i:flags)
#
# flags is a composite bit
# Bit 0: Left laser On/Off
# Bit 1: Right lasser On/Off

CMD_G028 = 0x10
# Home and return position
# ()
# TCP => (0x10, i:flag, f:X, f:Y, f:Z)
#
# if flag == 0:
#    Home successed
# else:
#    Error occour and X/Y/Z will be nan
#
# ATTENTION: G001 and G030 can not be used before G028 return successed

CMD_M017 = 0x11
# Lock all motors
# ()

CMD_M084 = 0x12
# Release all motors
# ()
#
# ATTENTION: G028 is required before using G001 and G030

CMD_G030 = 0x13
# Z-probe
# (f:x, f:y)
#            => (0x13, i:flag, f:z)
#
# if flag == 0:
#   Z-probe sucessed and the third value is z probe value.
# else:
#   Z-probe error and third value will always be nan.

CMD_M666 = 0x14
# Adjust
# ({f:X, f:Y, f:Z, f:H})
#
# X/Y/Z value must given at same time and one of them must be 0
# operation error will be raised if any value over hardware limit

CMD_VALU = 0x50
# Get device values, F: require flags
# (i:flags)
#           => (0x50, {f:X, f:Y, f:Z, b:F0, b:F1, b:thERR, b:MB})
#
# flags:
#  1: FSR           2: Filament 0      4: Filament 1
#  8: Master button


CMD_SYNC = 0xf0
# Set sync endpoint
# (s:ipv4address, i:port, s:salt)


CMD_QUIT = 0xfe
# Quit iContrl
#              => (0xfe, i:ST)
#
# Quit iControl, operation error will be raised if queue is not empty

CMD_KILL = 0xff
# KILL, iControl will quit and mainboard will be reset
#              => (0xfe, i:ST)


MSG_OPERATION_ERROR = 0x01
MSG_QUEUE_FULL = 0x02
MSG_UNKNOWN_ERROR = 0xff


# UDP Message
#   (i:1, s:SALT, i:timestemp, i:head_error_code, obj:headstatus)
#       * First 1 means it is a head status message
#       * SALT: not for use right now
#       * timestemp: not for use right now
#       * head_error_code:
#           == -2: Head Offline
#           == -1: Not ready
#            == 0: Ready
#             > 0: Follow toolhead error table


CMD_THPF = 0x51
# Get toolhead profile
# ()
#    => (0x51, {"module": "EXTRUDER", "vendor": "FLUX .inc", "id": "...", }, )

# CMD_THST = 0x52
# # Get toolhead status
# # ()
# #    => (0x52, {"tt": [210, ], "rt": [150, ], "tf": [0.9]})

CMD_M104 = 0x60
# Set toolhead extruder temperature
# (i:index, i:temperature)
#
# index: toolhead index, 0 or 1
# temperature should be positive
# operation error raised if index out of range or temperature over limit

CMD_M106 = 0x61
# Set fandspeed
# (i:index, f:speed)
#
# index: toolhead index, 0 or 1
# speed is a value from 0.0 to 1.0

CMD_HLSR = 0x62
# Toolhead pwm
# (f:pwm)
# use for head laser
# pwm is a value from 0.0 to 1.0

CMD_REQH = 0xf1
# Set required toolhead type
# (s:toolhead symbol)
#
# Toolhead must be "EXTRUDER" or "LASER" or "N/A", default is "N/A"
# After CMD_REQH, A CMD_BSTH command is required to enable head, otherwise
# toolhead will keep status at -2 (offline)

CMD_BSTH = 0xf2
# bootstrap toolhead if toolhead status is == -2 (offline)

CMD_CLHE = 0xf3
# Clear toolhead error code
# ()
#
# When toolhead raise an error, this error will appear in UDP message frame
# until this command send.

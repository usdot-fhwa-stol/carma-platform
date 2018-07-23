# EngineBrakeMode.msg

# This message contains the state of engine braking Mode

uint8 mode

uint8 PASS_THRU = 0
uint8 BYPASS = 1
uint8 MODE_ERROR = 2
uint8 NO_MODE_CHANGE = 3
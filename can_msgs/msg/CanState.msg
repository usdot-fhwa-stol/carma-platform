# CanState.msg
#
# Message denoting the state of the CAN Bus
#
# @author Joe Adkisson
# @version 0.1

uint8  driver_state

# enumeration values for status:
uint8 CLOSED=0
uint8 OPEN=1
uint8 READY=2
#
# Guidance Action Message
# 
# The indicator of current guidance state
#
# @author Shuwei Qiang
# @version 0.1
#

uint8 action

# enumeration values for all actions:
uint8 INTIALIZE = 1
uint8 ACTIVATE = 2
uint8 ENGAGE = 3
uint8 RESTART = 4
uint8 DEACTIVATE = 5
uint8 PANIC_SHUTDOWN = 6
uint8 SHUTDOWN = 0

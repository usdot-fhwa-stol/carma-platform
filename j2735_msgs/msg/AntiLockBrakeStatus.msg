#
# AntiLockBrakeStatus.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1

#AntiLockBrakeStatus ::= ENUMERATED {
#   unavailable (0), -- B'00  Vehicle Not Equipped with ABS Brakes 
#                    --       or ABS Brakes status is unavailable
#   off         (1), -- B'01  Vehicle's ABS are Off
#   on          (2), -- B'10  Vehicle's ABS are On ( but not Engaged )
#   engaged     (3)  -- B'11  Vehicle's ABS control is Engaged on any wheel
#   }
#

uint8  anti_lock_brake_status 

# enumeration values for status:
uint8  UNAVAILABLE=0
uint8  OFF=1
uint8  ON=2
uint8  ENGAGED=3

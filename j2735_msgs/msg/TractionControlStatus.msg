#
# TractionControlStatus.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1

#TractionControlStatus ::= ENUMERATED {
#   unavailable (0), -- B'00  Not Equipped with traction control 
#                    --       or traction control status is unavailable
#   off         (1), -- B'01  traction control is Off
#   on          (2), -- B'10  traction control is On (but not Engaged)
#   engaged     (3)  -- B'11  traction control is Engaged
#   }
#

uint8  traction_control_status 

# enumeration values for status:
uint8  UNAVAILABLE=0
uint8  OFF=1
uint8  ON=2
uint8  ENGAGED=3



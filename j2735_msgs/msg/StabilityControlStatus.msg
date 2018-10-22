#
# StabilityControlStatus.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1

#StabilityControlStatus ::= ENUMERATED {
#   unavailable (0), -- B'00  Not Equipped with SC
#                    --       or SC status is unavailable
#   off         (1), -- B'01  Off
#   on          (2), -- B'10  On or active (but not engaged)
#   engaged     (3)  -- B'11  stability control is Engaged 
#   }
#   

uint8  stability_control_status 

# enumeration values for status:
uint8  UNAVAILABLE=0
uint8  OFF=1
uint8  ON=2
uint8  ENGAGED=3

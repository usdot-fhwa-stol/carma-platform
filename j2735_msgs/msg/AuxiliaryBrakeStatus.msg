#
# AuxiliaryBrakeStatus.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1

#AuxiliaryBrakeStatus ::= ENUMERATED {
#   unavailable (0), -- B'00  Vehicle Not Equipped with Aux Brakes 
#                    --       or Aux Brakes status is unavailable
#   off         (1), -- B'01  Vehicle's Aux Brakes are Off
#   on          (2), -- B'10  Vehicle's Aux Brakes are On ( Engaged )
#   reserved    (3)  -- B'11 
#   }   

uint8  auxiliary_brake_status

# enumeration values for status:
uint8  UNAVAILABLE=0
uint8  OFF=1
uint8  ON=2
uint8  RESERVED=3

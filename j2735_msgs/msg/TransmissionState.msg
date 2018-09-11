#
# TransmissionState.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# Provide the current state of the vehicle transmission.

#TransmissionState ::= ENUMERATED {
#   neutral      (0), -- Neutral
#   park         (1), -- Park 
#   forwardGears (2), -- Forward gears
#   reverseGears (3), -- Reverse gears 
#   reserved1    (4),      
#   reserved2    (5),      
#   reserved3    (6),      
#   unavailable  (7)  -- not-equipped or unavailable value,
#   -- Any related speed is relative to the vehicle reference frame used
#   }

uint8  transmission_state

# enumeration values for status:
uint8 NEUTRAL=0
uint8 PARK=1
uint8 FORWARDGEARS=2
uint8 REVERSEGEARS=3
uint8 RESERVED1=4
uint8 RESERVED2=5
uint8 RESERVED3=6
uint8 UNAVAILABLE=7
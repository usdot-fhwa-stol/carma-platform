#
# BrakeAppliedStatus.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
# 

#BrakeAppliedStatus ::= BIT STRING {
#   unavailable (0),  -- When set, the brake applied status is unavailable
#   leftFront   (1),  -- Left Front Active
#   leftRear    (2),  -- Left Rear Active
#   rightFront  (3),  -- Right Front Active
#   rightRear   (4)   -- Right Rear Active
#   } (SIZE (5))
#   

uint8  brake_applied_status

uint8 UNAVAILABLE=0
uint8 LEFT_FRONT=1
uint8 LEFT_REAR=2
uint8 RIGHT_FRONT=3
uint8 RIGHT_REAR=4


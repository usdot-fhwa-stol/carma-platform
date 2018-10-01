#
# BrakeSystemStatus.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# Conveys a variety of information about the current brake and system control activity of the vehicle.

#BrakeSystemStatus ::= SEQUENCE { 
#   wheelBrakes        BrakeAppliedStatus,
#   traction           TractionControlStatus,
#   abs                AntiLockBrakeStatus, 
#   scs                StabilityControlStatus,
#   brakeBoost         BrakeBoostApplied, 
#   auxBrakes          AuxiliaryBrakeStatus
#   }

#BrakeAppliedStatus ::= BIT STRING {
#   unavailable (0),  -- When set, the brake applied status is unavailable
#   leftFront   (1),  -- Left Front Active
#   leftRear    (2),  -- Left Rear Active
#   rightFront  (3),  -- Right Front Active
#   rightRear   (4)   -- Right Rear Active
#   } (SIZE (5))
#   
j2735_msgs/BrakeAppliedStatus wheelBrakes

#TractionControlStatus ::= ENUMERATED {
#   unavailable (0), -- B'00  Not Equipped with traction control 
#                    --       or traction control status is unavailable
#   off         (1), -- B'01  traction control is Off
#   on          (2), -- B'10  traction control is On (but not Engaged)
#   engaged     (3)  -- B'11  traction control is Engaged
#   }
#
j2735_msgs/TractionControlStatus traction

#AntiLockBrakeStatus ::= ENUMERATED {
#   unavailable (0), -- B'00  Vehicle Not Equipped with ABS Brakes 
#                    --       or ABS Brakes status is unavailable
#   off         (1), -- B'01  Vehicle's ABS are Off
#   on          (2), -- B'10  Vehicle's ABS are On ( but not Engaged )
#   engaged     (3)  -- B'11  Vehicle's ABS control is Engaged on any wheel
#   }
#
j2735_msgs/AntiLockBrakeStatus abs

#StabilityControlStatus ::= ENUMERATED {
#   unavailable (0), -- B'00  Not Equipped with SC
#                    --       or SC status is unavailable
#   off         (1), -- B'01  Off
#   on          (2), -- B'10  On or active (but not engaged)
#   engaged     (3)  -- B'11  stability control is Engaged 
#   }
#   
j2735_msgs/StabilityControlStatus scs

#BrakeBoostApplied ::= ENUMERATED {
#   unavailable   (0), -- Vehicle not equipped with brake boost
#                      -- or brake boost data is unavailable
#   off           (1), -- Vehicle's brake boost is off
#   on            (2)  -- Vehicle's brake boost is on (applied)
#   }
#   
j2735_msgs/BrakeBoostApplied brakeBoost

#AuxiliaryBrakeStatus ::= ENUMERATED {
#   unavailable (0), -- B'00  Vehicle Not Equipped with Aux Brakes 
#                    --       or Aux Brakes status is unavailable
#   off         (1), -- B'01  Vehicle's Aux Brakes are Off
#   on          (2), -- B'10  Vehicle's Aux Brakes are On ( Engaged )
#   reserved    (3)  -- B'11 
#   }   

j2735_msgs/AuxiliaryBrakeStatus auxBrakes
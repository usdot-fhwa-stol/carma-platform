#
# BrakeBoostApplied.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1

#BrakeBoostApplied ::= ENUMERATED {
#   unavailable   (0), -- Vehicle not equipped with brake boost
#                      -- or brake boost data is unavailable
#   off           (1), -- Vehicle's brake boost is off
#   on            (2)  -- Vehicle's brake boost is on (applied)
#   }
#   

uint8  brake_boost_applied

# enumeration values for status:
uint8  UNAVAILABLE=0
uint8  OFF=1
uint8  ON=2

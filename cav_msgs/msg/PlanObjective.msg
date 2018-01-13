# PlanObjective.msg
#
# The enum for plan objective which is used by MobilityPlan message

# Variable which can be used to store a specific plan objective enum value.
uint8 type

uint8 UNKNOWN = 0
uint8 MOVE_OUT_OF_DISAPPEARING_LANE = 1
uint8 PLATOONING = 2
uint8 UPDATE_TO_EXISTING_PLAN = 3

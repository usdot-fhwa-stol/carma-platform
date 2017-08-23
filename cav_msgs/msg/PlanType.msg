# PlanType.msg
#
# Helper message for MobilityIntro.msg to define the specifics of a type of coordinated plan.
#

# Variable which can be used to store a specific plan type value.
uint8 type

# enumeration values for plan type:
uint8       CHANGE_LANE_LEFT = 1
uint8       CHANGE_LANE_RIGHT = 2
uint8       FORM_PLATOON = 3
uint8       SEARCHING_FOR_PLATOON = 4
uint8       JOIN_PLATOON_REAR = 5
uint8       DEPART_PLATOON = 6
uint8       SLOWDOWN = 7
uint8       SPEEDUP = 8
uint8       PASSING = 9 #includes lane change and speedup
# ideas for future additions: 
# pulling onto shoulder, need multi-lane change, join platoon in middle, merging platoons
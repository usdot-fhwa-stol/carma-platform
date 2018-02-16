# LaneChangeStatus.msg
#
# Announces the status of planning of a lane change maneuver, which may or may not involve other vehicles.
# If no other vehicle is involved then it should transition from PLAN_SENT to TIMED_OUT.
#

uint8 status

# enumeration values for status:
uint8   PLAN_SENT=1
uint8   ACCEPTANCE_RECEIVED=2
uint8   REJECTION_RECEIVED=3
uint8   OTHER_RECEIVED=4
uint8   PLANNING_SUCCESS=5
uint8   TIMED_OUT=6

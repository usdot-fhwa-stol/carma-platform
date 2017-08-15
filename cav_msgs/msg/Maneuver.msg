# Maneuver.msg
#
# Helper message for NewPlan.msg describes the details of a given type of maneuver,
# specifically to be used in definition of a new plan for future coordinated movement.
#

# the type of maneuver
uint8       type

# enum values for type:
uint8       CHANGE_LANE_LEFT = 1
uint8       CHANGE_LANE_RIGHT = 2
uint8       MAINTAIN_LANE = 3
uint8       TURN_CORNER_LEFT = 4
uint8       TURN_CORNER_RIGHT = 5
uint8       SPEEDUP = 6
uint8       SLOWDOWN = 7
uint8       CONSTANT_SPEED = 8

# Bitflags indicating which participants in the plan are performing this maneuver
# (possibly multiple vehicles are performing the same maneuver simultaneously).
# Bit 0 (LSb) is plan originator, bit positions 1-31 represent index to NewPlan
# participant_ids array.
uint32      performers

# unique ID of the roadway link in which this maneuver will begin
string      start_roadway_link

# Location on the link, according to the linear reference model (typically
# centerline distance from one end of the link) at which the maneuver will begin, meters.
# This refers to the position of the plan originator (other participants will 
# obviously be at a different position at that time).
float32     start_roadway_originator_position

# lane that the originator will be in (on the indicated start_roadway_link) when this
# maneuver is to begin.
uint8       start_roadway_lane_id

# anticipated length of the maneuver - how much distance will be covered from 
# beginning to end, meters
float32     length
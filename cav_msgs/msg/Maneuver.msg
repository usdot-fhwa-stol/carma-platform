# Maneuver.msg
#
# Helper message for MobilityPlan.msg describes the details of a given type of maneuver,
# specifically to be used in definition of a new plan for future coordinated movement.
#

# the type of maneuver
uint8       type

# enum values for type:
uint8       UNKNOWN = 0
uint8       CONSTANT_SPEED = 1
uint8       SPEEDUP = 2
uint8       SLOWDOWN = 3
uint8       MAINTAIN_LANE = 4
uint8       CHANGE_LANE_LEFT = 5
uint8       CHANGE_LANE_RIGHT = 6
# the rest value in not defined in J2735 specification
uint8       TURN_CORNER_LEFT = 4
uint8       TURN_CORNER_RIGHT = 5

# Bitflags indicating which participants in the plan are performing this maneuver
# (possibly multiple vehicles are performing the same maneuver simultaneously).
# Bit 0 (LSb) is plan originator, bit positions 1-31 represent index to MobilityPlan
# participant_ids array.
uint8[]      performers

# unique ID of the roadway link in which this maneuver will begin
string      start_roadway_link

# Location on the link, according to the linear reference model (typically
# centerline distance from one end of the link) at which the maneuver will begin, meters.
# This refers to the position of the plan originator (other participants will 
# obviously be at a different position at that time).
uint16     start_roadway_originator_position

# lane that the originator will be in (on the indicated start_roadway_link) when this
# maneuver is to begin.
uint8       start_roadway_lane_id

# anticipated length of the maneuver - how much distance will be covered from 
# beginning to end, meters
uint16      length

# maneuver parameters which may indicate target speed, lane ID, etc
# the format should be <startDistance>:<endDistance>:<startSpeed>:<endSpeed>
string      parameters

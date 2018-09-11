#
# Connection.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The Connection data structure is used in the ConnectsToList data frame to provide data about how the stop line at
# the end of a single lane connects to another lane beyond its stop point. The ConnectingLane entry ties an outbound
# (egress) lane by its index to a valid single maneuver required to reach that outbound lane. The SignalGroupID maps this
# to a single SPAT index. (Note that more than one entry can exist for any given lane to handle admissive and protected
# conditions).
#
# Connection ::= SEQUENCE {
# -- The subject lane connecting to this lane is:
# connectingLane ConnectingLane,
# -- The index of the connecting lane and also
# -- the maneuver from the current lane to it
# remoteIntersection IntersectionReferenceID OPTIONAL,
# -- This entry is only used when the
# -- indicated connecting lane belongs
# -- to another intersection layout. This
# -- provides a means to create meshes of lanes
# -- SPAT mapping details at the stop line are:
# signalGroup SignalGroupID OPTIONAL,
# -- The matching signal group send by
# -- the SPAT message for this lane/maneuver.
# -- Shall be present unless the connectingLane
# -- has no signal group (is un-signalized)
# userClass RestrictionClassID OPTIONAL,
# -- The Restriction Class of users this applies to
# -- The use of some lane/maneuver and SignalGroupID
# -- pairings are restricted to selected users.
# -- When absent, the SignalGroupID applies to all
# -- Movement assist details are given by:
# connectionID LaneConnectionID OPTIONAL
# -- An optional connection index used to
# -- relate this lane connection to any dynamic
# -- clearance data in the SPAT. Note that
# -- the index may be shared with other
# -- connections if the clearance data is common
# }

j2735_msgs/ConnectingLane connecting_lane

j2735_msgs/IntersectionReferenceID remote_intersection
bool remote_intersection_exists

# SignalGroupID ::= INTEGER (0..255)
uint8 signal_group
bool signal_group_exists

# RestrictionClassID ::= INTEGER (0..255)
uint8 user_class
bool user_class_exists

# LaneConnectionID ::= INTEGER (0..255)
uint8 connection_id
bool connection_id_exists

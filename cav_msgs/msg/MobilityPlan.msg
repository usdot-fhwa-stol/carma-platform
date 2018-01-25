# MobilityPlan.msg
#
# This message describes a detailed proposed plan between two vehicles.
# NOTE: This message is only a placeholder and will be changed or removed if not needed

# MsgCRC ::= OCTET STRING (SIZE(2)) -- created with the CRC-CCITT polynomial
uint8[]  checksum

# Mobility message header
cav_msgs/MobilityHeader header

# Mobility plan message sequence number
uint8 sequence

# Plan objective
cav_msgs/PlanObjective objective

# plan detail for plugin usage
string strategy

# the GUID of the first participant
uint8[] participant_1

# the GUID of the first participant
uint8[] participant_2

# the GUID of the first participant
uint8[] participant_3

# the GUID of the first participant
uint8[] participant_4

# the GUID of the first participant
uint8[] participant_5

# the maneuver list in the plan
cav_msgs/Maneuver[]  maneuvers

# the expiration time of the current plan
# A UNIX timestamp in milliseconds, which indicates the time elapsed since UNIX epoch
uint64    expiration

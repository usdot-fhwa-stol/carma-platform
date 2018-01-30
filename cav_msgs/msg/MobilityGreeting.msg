# MobilityGreeting.msg
#
# This is a message sent in response to a MobilityIntro, either communicating intent to
# engage in conversation about possible cooperation or desire to be dropped from consideration.
#

# MsgCRC ::= OCTET STRING (SIZE(2)) -- created with the CRC-CCITT polynomial
uint8[]     checksum

# the standard header for all mobility messages
cav_msgs/MobilityHeader header

# the host's entity type
cav_msgs/BasicVehicleClass my_entity_type

# unique ID of the roadway link that host vehicle is currently on
string      my_roadway_link

# Host's current location on the link, according to the linear reference model
# (typically centerline distance from one end of the link)
uint16     my_roadway_link_position

# ID of the lane that host currently occupies
uint8       my_lane_id

# Host's current forward speed
float32  forward_speed

# indicator of my degree of agreement with the tentative plan
cav_msgs/MobilityAgreementType agreement

# a generic variable whose use is dependent upon the value of agreement
uint16      agreement_param

# a string which show system brand/version, list of strategies it can perform
string capabilities

# a hex string representing a new session key that is to be used to encrypt the
# remaining messages in this conversation (about this plan_id) between these two entities.
uint8[]      session_key

# absolute time of expiration of the session_key
# A UNIX timestamp in milliseconds, which indicates the time elapsed since UNIX epoch
uint64         expiration

# a random string of 8 printable chars - this is used to verify correct receipt & use of
# the session key
string      verification_code


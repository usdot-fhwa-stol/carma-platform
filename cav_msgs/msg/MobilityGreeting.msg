# MobilityGreeting.msg
#
# This is a message sent in response to a MobilityIntro, either communicating intent to
# engage in conversation about possible cooperation or desire to be dropped from consideration.
#

# the standard header for all mobility messages
MobilityHeader header

# the host's entity type
MobilityEntityType my_entity_type

# unique ID of the roadway link that host vehicle is currently on
string      my_roadway_link

# Host's current location on the link, according to the linear reference model (typically
# centerline distance from one end of the link)
float16     my_roadway_link_position

# ID of the lane that host currently occupies
uint8       my_lane_id

# indicator of my degree of agreement with the tentative plan
MobilityAgreementType agreement

### note: if agreement = DISAGREE_TERMINATE then the remaining values should be ignored

# a generic variable whose use is dependent upon the value of agreement
uint16      agreement_param

# a bit array with each bit inidicating a different capability relative to the proposed
# plan or relative to the counter-proposal if one is made (doesn't require exposing all
# of the sender's capabilities).
uint32      my_capabilities

# bitflag values for my_capabilities
# TODO: define these!

# a hex string representing a new session key that is to be used to encrypt the
# remaining messages in this conversation (about this plan_id) between these two entities.
string      session_key

# a random string of 8 printable chars - this is used to verify correct receipt & use of
# the session key
string      verification_code

# sender's current forward speed, m/s
float32     my_forward_speed

# absolute time of expiration of the session_key
time        session_key_expiration

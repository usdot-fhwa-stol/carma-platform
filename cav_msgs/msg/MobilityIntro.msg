# MobilityIntro.msg
#
# This is the first message a CAV will use to blindly introduce itself to the other CAVs
# in the vicinity, in the interest of starting a conversation about a particular
# suggested coordinated activity.  It is intended to guage others' interest and prompt
# for identification of those who are interested.
#

# the standard header for all mobility messages
MobilityHeader header

# the host's entity type
MobilityEntityType my_entity_type

# enumeration values for my_entity_type:
uint8       VEHICLE = 1
uint8       INFRASTRUCTURE = 100

# unique ID of the roadway link that host vehicle is currently on
string      my_roadway_link

# Host's current location on the link, according to the linear reference model (typically
# centerline distance from one end of the link)
float16     my_roadway_link_position

# ID of the lane that host currently occupies
uint8       my_lane_id

# type of the proposal being suggested to neighbors - refers to activity the host would
# like to perform
PlanType    plan_type

# a generic variable whose use is dependent upon the plan_type
uint16      proposal_param

# a hex string representing the PKI public key of the host
string      my_public_key

# absolute time that this proposed plan will expire and become meaningless
time        expiration

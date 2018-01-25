# MobilityIntro.msg
#
# This is the first message a CAV will use to blindly introduce itself to the other CAVs
# in the vicinity, in the interest of starting a conversation about a particular
# suggested coordinated activity.  It is intended to guage others' interest and prompt
# for identification of those who are interested.
#

# MobilityChecksum ::= INTEGER (0..65535)
uint16  checksum

# the standard header for all mobility messages
cav_msgs/MobilityHeader  header

# the host's entity type
cav_msgs/BasicVehicleClass  my_entity_type

# unique ID of the roadway link that host vehicle is currently on
string  my_roadway_link

# Host's current location on the link, according to the linear reference model (typically
# centerline distance from one end of the link)
uint16  my_roadway_link_position

# ID of the lane that host currently occupies
uint8  my_lane_id

# Host's current forward speed in m/s
float32  forward_speed

# type of the proposal being suggested to neighbors - refers to activity the host would
# like to perform
cav_msgs/PlanType  plan_type

# a generic variable whose use is dependent upon the plan_type
uint16   proposal_param

# a hex string representing the PKI public key of the host
uint8[]  my_public_key

# absolute time that this proposed plan will expire and become meaningless
# A UNIX timestamp in milliseconds, which indicates the time elapsed since UNIX epoch
uint64     expiration

# a string which show system brand/version, list of strategies it can perform
string   capabilities

#
# PlatooningInfo.msg
# 
# The details of the current platoon information
#
# @author Shuwei Qiang
# @version 0.1
#

# Platoon state
uint8    state
uint8    DISABLED=0
uint8    SEARCHING=1
uint8    CONNECTING_TO_NEW_FOLLOWER=2
uint8    CONNECTING_TO_NEW_LEADER=3
uint8    LEADING=4
uint8    FOLLOWING=5

# Current platoon ID (GUID)
string   platoon_id

# Platoon current size
uint8    size

# Platoon size upper limit
uint8    size_limit

# Leader vehicle static Id (license plate)
string   leader_id

# Leader current downtrack distance
float32  leader_downtrack_distance

# Leader command speed
float32  leader_cmd_speed

# Host position in platoon
uint8    host_platoon_position

# Host adjusted command speed
float32  host_cmd_speed

# Desired gap with leader
float32  desired_gap

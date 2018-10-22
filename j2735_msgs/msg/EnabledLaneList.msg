#
# EnabledLaneList.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The Enabled Lane List data frame is a sequence of lane IDs for lane objects that are activated in the current map
# configuration. These lanes, unlike most lanes, have their RevocableLane bit set to one (asserted). Such lanes are not
# considered to be part of the current map unless they are in the Enabled Lane List. This concept is used to describe all the
# possible regulatory states for a given physical lane.
#
# EnabledLaneList ::= SEQUENCE (SIZE(1..16)) OF LaneID
#-- The unique ID numbers for each
#-- lane object which is 'active'
#-- as part of the dynamic map contents.

# LaneID ::= INTEGER (0..255)
# -- the value 0 shall be used when the lane ID is not available or not known
# -- the value 255 is reserved for future use
uint16[] lane_id_list
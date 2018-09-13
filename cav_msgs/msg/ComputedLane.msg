#
# NodeListXY.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# ComputedLane ::= SEQUENCE {
# -- Data needed to created a computed lane
# referenceLaneId LaneID,
# 	-- the lane ID upon which this
# 	-- computed lane will be based
# 	-- Lane Offset in X and Y direction
# offsetXaxis CHOICE {
# 		small DrivenLineOffsetSm,
# 		large DrivenLineOffsetLg
# 		},
# offsetYaxis CHOICE {
# 		small DrivenLineOffsetSm,
# 		large DrivenLineOffsetLg
# 		},
# 	-- A path X offset value for translations of the
# 	-- path's points when creating translated lanes.
# 	-- The values found in the reference lane are
# 	-- all offset based on the X and Y values from
# 	-- the coordinates of the reference lane's
# 	-- initial path point.
#
# -- Lane Rotation
# rotateXY Angle OPTIONAL,
# 	-- A path rotation value for the entire lane
# 	-- Observe that this rotates the existing orientation
# 	-- of the referenced lane, it does not replace it.
# 	-- Rotation occurs about the initial path point.
# -- Lane Path Scale (zooming)
# scaleXaxis Scale-B12 OPTIONAL,
# scaleYaxis Scale-B12 OPTIONAL,
# 	-- value for translations or zooming of the path's
# 	-- points. The values found in the reference lane
# 	-- are all expanded or contracted based on the X
# 	-- and Y and width values from the coordinates of
# 	-- the reference lane's initial path point.
# 	-- The Z axis remains untouched.
# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-ComputedLane}} OPTIONAL,
# ...
# }

# LaneID ::= INTEGER (0..255)
# -- the value 0 shall be used when the lane ID is
# -- not available or not known
# -- the value 255 is reserved for future use
uint16 reference_lane_id

# The offset axis for the computed lane
cav_msgs/OffsetAxis offset_x_axis

cav_msgs/OffsetAxis offset_y_axis

# Deviates from J2735 standard:
# DrivenLineOffsetSm in this message are conveyed as deg rather than 1.5 deg units to avoid redundant unnecessary conversion by subscribers
# Angle ::= FLOAT (0..359)
#  -- Unsigned units of 1 degree, in 1 octet
#  -- the true north is 0, positive is clockwise
#  -- the values 360 to 381 shall not be sent
float32 rotateXY
bool rotatexy_exists

# Scale-B12 ::= INTEGER (-2048..2047)
# A 12-bit signed scaling factor supporting scales from zero (which is not used) to >200%.
# In this data element, the value zero is taken to represent a value of one (scale 1:1).
int16 scale_x_axis
bool scale_x_axis_exists

int16 scale_y_axis
bool scale_y_axis_exists

# regional #TODO: RegionalExtensions are not yet implemented in asn1c

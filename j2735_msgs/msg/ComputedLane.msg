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

# The DrivenLineOffsetSmall data element is an integer value expressing the offset in a defined axis from a reference
# lane number from which a computed lane is offset.
# DrivenLineOffsetSm ::= INTEGER (-2047..2047)
# -- LSB units are 1 cm.
# The DE_DrivenLineOffsetLarge data element is an integer value expressing the offset in a defined axis from a
# reference lane number from which a computed lane is offset.
# DrivenLineOffsetLg ::= INTEGER (-32767..32767)
# -- LSB units are 1 cm.

j2735_msgs/OffsetXaxis offset_x_axis

j2735_msgs/OffsetYaxis offset_y_axis

# Angle ::= INTEGER (0..239)
#  -- Unsigned units of 1.5 degree, in 1 octet
#  -- the true north is 0, positive is clockwise
#  -- the values 240 to 254 shall not be sent
#  -- the value 255 (0xFF) indicates an invalid value
# NOTE: use invalid value to indicate unavailability as well.
uint16 rotateXY
uint16 ROTATEXY_INVALID=255
bool rotatexy_exists

# Scale-B12 ::= INTEGER (-2048..2047)
# A 12-bit signed scaling factor supporting scales from zero (which is not used) to >200%.
# In this data element, the value zero is taken to represent a value of one (scale 1:1).
int16 scale_x_axis
bool scale_x_axis_exists

int16 scale_y_axis
bool scale_y_axis_exists

# regional #TODO: RegionalExtensions are not yet implemented in asn1c

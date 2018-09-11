#
# OffsetYaxis.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# 	-- A path X offset value for translations of the
# 	-- path's points when creating translated lanes.
# 	-- The values found in the reference lane are
# 	-- all offset based on the X and Y values from
# 	-- the coordinates of the reference lane's
# 	-- initial path point.
# offsetYaxis CHOICE {
# 		small DrivenLineOffsetSm,
# 		large DrivenLineOffsetLg
# 		},

uint8 choice
uint8 SMALL=0
uint8 LARGE=1

# The DrivenLineOffsetSmall data element is an integer value expressing the offset in a defined axis from a reference
# lane number from which a computed lane is offset.
# DrivenLineOffsetSm ::= INTEGER (-2047..2047)
# -- LSB units are 1 cm.
int16 small


# The DE_DrivenLineOffsetLarge data element is an integer value expressing the offset in a defined axis from a
# reference lane number from which a computed lane is offset.
# DrivenLineOffsetLg ::= INTEGER (-32767..32767)
# -- LSB units are 1 cm.
int16 large
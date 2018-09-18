#
# OffsetXaxis.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# 	-- A path offset value along the X or Y axis for translations of the
# 	-- path's points when creating translated lanes.
# 	-- The values found in the reference lane are
# 	-- all offset based on the X and Y values from
# 	-- the coordinates of the reference lane's
# 	-- initial path point.
# offsetXaxis CHOICE {
# 		small DrivenLineOffsetSm,
# 		large DrivenLineOffsetLg
# 		},


uint8 choice
uint8 SMALL=0
uint8 LARGE=1

# Deviates from J2735 standard:
# DrivenLineOffsetSm in this message are conveyed as m rather than cm to avoid redundant unnecessary conversion by subscribers
# The DrivenLineOffsetSmall data element is a value expressing the offset in a defined axis from a reference
# lane number from which a computed lane is offset.
# DrivenLineOffsetSm ::= INTEGER (-20.47-20.47)
# -- units are m.
float32 small

# Deviates from J2735 standard:
# DrivenLineOffsetLg in this message are conveyed as m rather than cm to avoid redundant unnecessary conversion by subscribers
# The DrivenLineOffsetLg data element is a value expressing the offset in a defined axis from a
# reference lane number from which a computed lane is offset.
# DrivenLineOffsetLg ::= INTEGER (-327.67-327.67)
# -- units are m.
float32 large
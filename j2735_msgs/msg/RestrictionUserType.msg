#
# RestrictionUserType.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_RestrictionUserType data frame is used to provide a means to select one, and only one, user type or class
# from a number of well-known lists. The selected entry is then used in the overall Restriction Class assignment process to
# indicate that a given GroupID (a way of expressing a movement in the SPAT/MAP system) applies to (is restricted to) this
# class of user.

# RestrictionUserType ::= CHOICE {
# basicType RestrictionAppliesTo,
# -- a set of the most commonly used types
# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-RestrictionUserType}},
# ...
# }

uint8 choice
uint8 BASIC_TYPE=0
uint8 REGIONAL=1 #TODO: cannot select at this time but we shoudl move forward with treating it as a choice.

j2735_msgs/RestrictionAppliesTo basicType

#regional #TODO: RegionalExtensions are not yet implemented in asn1c

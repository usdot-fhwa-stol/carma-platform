#
# RestrictionClassAssignment.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_RestrictionClassAssignment data frame is used to assign (or bind) a single RestrictionClassID data
# element to a list of all user classes to which it applies. A collection of these bindings is conveyed in the
# RestrictionClassList data frame in the MAP message to travelers.
#
# RestrictionClassAssignment ::= SEQUENCE {
# id RestrictionClassID,
# -- the unique value (within an intersection or local region)
# -- that is assigned to this group of users
# users RestrictionUserTypeList
# -- The list of user types/classes
# -- to which this restriction ID applies
# }

# RestrictionClassID ::= INTEGER (0..255)
uint8 id



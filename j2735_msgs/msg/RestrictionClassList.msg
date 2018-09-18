#
# RestrictionClassList.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_RestrictionClassList data frame is used to enumerate a list of user classes which belong to a given
# assigned index. The resulting collection is treated as a group by the signal controller when it issues movement data
# (signal phase information) with the GroupID for this group. This data frame is typically static for long periods of time
# (months) and conveyed to the user by means of the MAP message.

# RestrictionClassList ::= SEQUENCE (SIZE(1..254)) OF RestrictionClassAssignment

j2735_msgs/RestrictionClassAssignment[] restriction_class_list
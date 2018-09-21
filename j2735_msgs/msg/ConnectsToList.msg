#
# ConnectsToList.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The ConnectsToList data structure is used in the generic lane descriptions to provide a sequence of other defined
# lanes to which each lane connects beyond its stop point. See the Connection data frame entry for details. Note that this
# data frame is not used in some lane object types.
#
# ConnectsToList ::= SEQUENCE (SIZE(1..16)) OF Connection

j2735_msgs/Connection[] connect_to_list

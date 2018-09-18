#
# NodeAttributeXYList.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
#The DF_SegmentAttributeXYList data frame consists of a list of SegmentAttributeXY entries.

#SegmentAttributeXYList ::= SEQUENCE (SIZE(1..8)) OF SegmentAttributeXY

j2735_msgs/SegmentAttributeXY[] segment_attribute_xy


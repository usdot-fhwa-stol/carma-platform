#
# RoadSegmentReferenceID.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_RoadSegmentReferenceID data frame is used to convey theRoadSegmentID which is unique to a given
# road segment of interest, and also the RoadRegulatorID assigned to the region in which it is operating (when required).

# RoadSegmentReferenceID ::= SEQUENCE {
# region RoadRegulatorID OPTIONAL,
# -- a globally unique regional assignment value
# -- typically assigned to a regional DOT authority
# -- the value zero shall be used for testing needs
# id RoadSegmentID
# -- a unique mapping to the road segment
# -- in question within the above region of use
# -- during its period of assignment and use
# -- note that unlike intersectionID values,
# -- this value can be reused by the region
# }

# RoadRegulatorID ::= INTEGER (0..65535)
uint16 region
bool region_exists

# RoadSegmentID ::= INTEGER (0..65535)
# -- The values zero to 255 shall be used for testing only
# -- Note that the value assigned to an RoadSegment will be
# -- unique within a given regional ID only during its use
uint16 id
bool id_exists




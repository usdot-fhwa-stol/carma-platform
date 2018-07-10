#
# LaneDirection.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The LaneDirection data element is used to denote the allowed direction of travel over a lane object. By convention,
# the lane object is always described from the stop line outwards away from the intersection. Therefore, the ingress
# direction is from the end of the path to the stop line and the egress direction is from the stop line outwards. It should be
# noted that some lane objects are not used for travel and that some lane objects allow bi-directional travel.
#
# LaneDirection ::= BIT STRING {
# -- With bits as defined:
# -- Allowed directions of travel in the lane object
# -- All lanes are described from the stop line outwards
# ingressPath (0),
# -- travel from rear of path to front is allowed
# egressPath (1)
# -- travel from front of path to rear is allowed
# -- Notes: No Travel, i.e. the lane object type does not support travel (medians, curbs, etc.)
# is indicated by not asserting any bit value
# - Bi-Directional Travel (such as a ped crosswalk) is indicated by asserting both of the bits
# } (SIZE (2))

uint8 lane_direction
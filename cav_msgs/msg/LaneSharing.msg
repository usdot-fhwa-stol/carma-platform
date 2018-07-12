#
# LaneSharing.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
#
# LaneSharing ::= BIT STRING {
# -- With bits as defined:
# overlappingLaneDescriptionProvided (0),
# -- Assert when another lane object is present to describe the
# -- path of the overlapping shared lane
# -- this construct is not used for lane objects which simply cross
# multipleLanesTreatedAsOneLane(1),
# -- Assert if the lane object path and width details represents
# -- multiple lanes within it that are not further described
# -- Various modes and type of traffic that may share this lane:
# otherNonMotorizedTrafficTypes (2), -- horse drawn etc.
# individualMotorizedVehicleTraffic (3),
# busVehicleTraffic (4),
# taxiVehicleTraffic (5),
# pedestriansTraffic (6),
# cyclistVehicleTraffic (7),
# trackedVehicleTraffic (8),
# pedestrianTraffic (9)
# } (SIZE (10))
# -- All zeros would indicate 'not shared' and 'not overlapping'

uint16 lane_sharing
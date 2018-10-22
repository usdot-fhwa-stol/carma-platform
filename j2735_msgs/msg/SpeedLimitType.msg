# SpeedLimitType.msg
#
# The SpeedLimitType data element relates the type of speed limit to which a given speed refers.
#
# SpeedLimitType ::= ENUMERATED {
#
# unknown, -- Speed limit type not available
# maxSpeedInSchoolZone, -- Only sent when the limit is active
# maxSpeedInSchoolZoneWhenChildrenArePresent, -- Sent at any time
# maxSpeedInConstructionZone, -- Used for work zones, incident zones, etc. where a reduced speed is present
#
# vehicleMinSpeed,
# vehicleMaxSpeed, -- Regulatory speed limit for general traffic
# vehicleNightMaxSpeed,
#
# truckMinSpeed,
# truckMaxSpeed,
# truckNightMaxSpeed,
#
# vehiclesWithTrailersMinSpeed,
# vehiclesWithTrailersMaxSpeed,
# vehiclesWithTrailersNightMaxSpeed,
# ...
# }

uint8 speed_limit_type

uint8 UNKNOWN=0
uint8 MAXSPEEDINSCHOOLZONE=1
uint8 MAXSPEEDINSCHOOLZONEWHENCHILDRENAREPRESENT=2
uint8 MAXSPEEDINCONSTRUCTIONZONE=3
uint8 VEHICLEMINSPEED=4
uint8 VEHICLEMAXSPEED=5
uint8 VEHICLENIGHTMAXSPEED=6
uint8 TRUCKMINSPEED=7
uint8 TRUCKMAXSPEED=8
uint8 TRUCKNIGHTMAXSPEED=9
uint8 VEHICLESWITHTRAILERSMINSPEED=10
uint8 VEHICLESWITHTRAILERSMAXSPEED=11
uint8 VEHICLESWITHTRAILERSNIGHTMAXSPEED=12
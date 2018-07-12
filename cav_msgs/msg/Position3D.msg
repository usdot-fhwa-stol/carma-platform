#
# Position3D.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
# Provides a precise location in the WGS-84 coordinate system, from which short offsets may be used
# to create additional data using a flat earth projection centered on this location.

# Position3D ::= SEQUENCE {

# lat Latitude, -- in 1/10th micro degrees
# Latitude ::= INTEGER (-900000000..900000001)
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 90 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
float64 latitude

float64 LATITUDE_UNAVAILABLE = 90.0000001
float64 LATITUDE_MAX = 90.0
float64 LATITUDE_MIN = -90.0

# long Longitude, -- in 1/10th micro degrees
# Longitude ::= INTEGER (-1799999999..1800000001)
#  -- LSB = 1/10 micro degree
#  -- Providing a range of plus-minus 180 degrees
#  -- Convert to degree with factor 0.0000001 when field is used
float64 longitude

float64 LONGITUDE_UNAVAILABLE = 180.0000001
float64 LONGITUDE_MAX = 180.0
float64 LONGITUDE_MIN = -179.9999999

# elevation Elevation OPTIONAL, -- in 10 cm units
# Elevation ::= INTEGER (-4096..61439)
#  -- In units of 10 cm steps above or below the reference ellipsoid
#  -- Providing a range of -409.5 to + 6143.9 meters
#  -- The value -4096 shall be used when Unknown is to be sent
#  -- Convert to meter with factor 0.1 when field is used
float32 elevation
bool elevation_exists

float32 ELEVATION_UNAVAILABLE = -409.6
float32 ELEVATION_MAX = 6143.9
float32 ELEVATION_MIN = -409.5

#TODO: RegionalExtensions are not yet implemented in asn1c
# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-Position3D}} OPTIONAL,

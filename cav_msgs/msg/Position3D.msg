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

# Deviates from J2735 standard:
# Latitude in this message are conveyed as deg rather than 1/10th micro deg to avoid redundant unnecessary conversion by subscribers
# lat Latitude, -- in degrees
# Latitude ::= DOUBLE (-90.0000000-90.0000001)
#  -- Providing a range of plus-minus 90 degrees
float64 latitude

float64 LATITUDE_UNAVAILABLE = 90.0000001
float64 LATITUDE_MAX = 90.0
float64 LATITUDE_MIN = -90.0

# Deviates from J2735 standard:
# Latitude in this message are conveyed as deg rather than 1/10th micro deg to avoid redundant unnecessary conversion by subscribers
# long Longitude, -- in degrees
# Longitude ::= INTEGER (-179.9999999-180.0000001)
#  -- Providing a range of plus-minus 180 degrees
float64 longitude

float64 LONGITUDE_UNAVAILABLE = 180.0000001
float64 LONGITUDE_MAX = 180.0
float64 LONGITUDE_MIN = -179.9999999

# Deviates from J2735 standard:
# Elevation in this message are conveyed as m rather than 10 cm to avoid redundant unnecessary conversion by subscribers
# elevation Elevation OPTIONAL, -- in 1 m units
# Elevation ::= INTEGER (-409.6-6143.9)
#  -- In units of m steps above or below the reference ellipsoid
#  -- Providing a range of -409.5 to + 6143.9 meters
#  -- The value -409.6 shall be used when Unknown is to be sent
float32 elevation
bool elevation_exists

float32 ELEVATION_UNAVAILABLE = -409.6
float32 ELEVATION_MAX = 6143.9
float32 ELEVATION_MIN = -409.5

#TODO: RegionalExtensions are not yet implemented in asn1c
# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-Position3D}} OPTIONAL,

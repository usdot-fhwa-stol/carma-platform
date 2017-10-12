#
# PositionalAccuracy.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# Used to model the accuracy of the positional determination with respect to each given axis.

#PositionalAccuracy ::= SEQUENCE {
#   -- NMEA-183 values expressed in strict ASN form
#   semiMajor     SemiMajorAxisAccuracy,
#   semiMinor     SemiMinorAxisAccuracy,
#   orientation   SemiMajorAxisOrientation
#   }

#SemiMajorAxisAccuracy ::= INTEGER (0..255)
#   -- semi-major axis accuracy at one standard dev 
#   -- range 0-12.7 meter, LSB = .05m
#   -- 254 = any value equal or greater than 12.70 meter
#   -- 255 = unavailable semi-major axis value
float32 semiMajor   

#SemiMinorAxisAccuracy ::= INTEGER (0..255)
#   -- semi-minor axis accuracy at one standard dev 
#   -- range 0-12.7 meter, LSB = .05m
#   -- 254 = any value equal or greater than 12.70 meter
#   -- 255 = unavailable semi-minor axis value
float32 semiMinor

#SemiMajorAxisOrientation ::= INTEGER (0..65535)
#   -- orientation of semi-major axis 
#   -- relative to true north (0~359.9945078786 degrees)
#   -- LSB units of 360/65535 deg  = 0.0054932479
#   -- a value of 0 shall be 0 degrees
#   -- a value of 1 shall be 0.0054932479 degrees 
#   -- a value of 65534 shall be 359.9945078786 deg
#   -- a value of 65535 shall be used for orientation unavailable
float64 orientation

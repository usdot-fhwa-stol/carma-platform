#
# AdvisorySpeed.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DF_AdvisorySpeed data frame is used to convey a recommended traveling approach speed to an intersection
# from the message issuer to various travelers and vehicle types. Besides support for various eco-driving applications, this
# allows transmitting recommended speeds for specialty vehicles such as transit buses.

# AdvisorySpeed ::= SEQUENCE {
# type AdvisorySpeedType,
# -- the type of advisory which this is.
# speed SpeedAdvice OPTIONAL,
# -- See Section 11 for converting and translating speed
# -- expressed in mph into units of m/s
# -- This element is optional ONLY when superceded
# -- by the presence of a regional speed element found in
# -- Reg-AdvisorySpeed entry
# confidence SpeedConfidence OPTIONAL,
# -- A confidence value for the above speed
# distance ZoneLength OPTIONAL,
# -- Unit = 1 meter,
# -- The distance indicates the region for which the advised speed
# -- is recommended, it is specified upstream from the stop bar
# -- along the connected egressing lane
# class RestrictionClassID OPTIONAL,
# -- the vehicle types to which it applies
# -- when absent, the AdvisorySpeed applies to
# -- all motor vehicle types
# regional SEQUENCE (SIZE(1..4)) OF RegionalExtension {{REGION.Reg-AdvisorySpeed}} OPTIONAL,
# ...
# }

cav_msg/AdvisorySpeedType type

# SpeedAdvice ::= INTEGER (0..500)
# This data element represents the recommended velocity of an object, typically a vehicle speed along a roadway,
# expressed in unsigned units of 0.1 meters per second.
#-- LSB units are 0.1 m/s^2
#-- the value 499 shall be used for values at or greater than 49.9 m/s
#-- the value 500 shall be used to indicate that speed is unavailable
float32 speed

cav_msg/SpeedConfidence confidence

# ZoneLength ::= INTEGER (0..10000)
# -- Unit = 1 meter, 0 = unknown,
# -- The value 10000 to be used for Distances >=10000 m
# -- (e.g. from known point to another point along a
# -- known path, often against traffic flow direction
# -- when used for measuring queues)
uint16 distance

# RestrictionClassID ::= INTEGER (0..255)
uint8 class

# regional #TODO: RegionalExtensions are not yet implemented in asn1c



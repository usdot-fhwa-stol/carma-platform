#
# RestrictionAppliesTo.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The RestrictionAppliesTo data element provides a short list of common vehicle
# types which may have one or more special movements at an intersection. I
#
# RestrictionAppliesTo ::= ENUMERATED {
# none, -- applies to nothing
# equippedTransit, -- buses etc.
# equippedTaxis,
# equippedOther, -- other vehicle types with
# 		-- necessary signal phase state
# 		-- reception equipment
# emissionCompliant, -- regional variants with more
# 		-- definitive items also exist
# equippedBicycle,
# weightCompliant,
# heightCompliant,
# -- Items dealing with traveler needs serviced by the infrastructure
# -- These end users (which are not vehicles) are presumed to be suitably equipped
# pedestrians,
# slowMovingPersons,
# wheelchairUsers,
# visualDisabilities,
# audioDisabilities, -- hearing
# otherUnknownDisabilities,
# ...
# }

uint8 NONE=0
uint8 EQUIPPEDTRANSIT=1
uint8 EQUIPPEDTAXIS=2
uint8 EQUIPPEDOTHER=3
uint8 EMISSIONCOMPLIANT=4
uint8 EQUIPPEDBICYCLE=5
uint8 WEIGHTCOMPLIANT=6
uint8 HEIGHTCOMPLIANT=7
uint8 PEDESTRIANS=8
uint8 SLOWMOVINGPERSONS=9
uint8 WHEELCHAIRUSERS=10
uint8 VISUALDISABILITIES=11
uint8 AUDIODISABILITIES=12
uint8 OTHERUNKNOWNDISABILITIES=13

# Variable which can be used to store a type enum value.
uint8 restriction_applies_to




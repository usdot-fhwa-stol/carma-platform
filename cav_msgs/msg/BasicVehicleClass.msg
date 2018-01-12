# BasicVehicleClass.msg
#
# Enumeration of possible types of transportation entities participating in a
# conversation about cooperative activities.
#

# Variable which can be used to store a specific entity type value.
uint8 type

# enumeration values for my_entity_type:
uint8       UNKNOWN = 0
uint8       DEFAULT_PASSENGER_VEHICLE = 10
uint8       DEFAULT_TRUCK_VEHICLE = 20
uint8       DEFAULT_INFRASTRUCTURE = 90
# need to add more type based on J2735

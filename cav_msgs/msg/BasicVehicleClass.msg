# BasicVehicleClass.msg
#
# Enumeration of possible types of transportation entities participating in a
# conversation about cooperative activities.
#

# Variable which can be used to store a specific entity type value.
uint8 type

# enumeration values for my_entity_type:
uint8       UNKNOWN = 0
uint8       DEFAULTPASSENGERVEHICLE = 10
uint8       DEFAULTTRUCKVEHICLE = 20
uint8       DEFAULTINFRASTRUCTURE = 90

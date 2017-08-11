# MobilityEntityType.msg
#
# Enumeration of possible types of transportation entities participating in a
# conversation about cooperative activities.
#

# enumeration values for my_entity_type:
uint8       VEHICLE = 1
uint8       INFRASTRUCTURE = 100
# we may want to break down either vehicle or infrastructure types in the future,
# hence the gap between them.

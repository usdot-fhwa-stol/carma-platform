# ActiveManeuvers.msg
#
# Lists the details of the lateral and longitudinal plugins currently in control of the vehicle
#

### Longitudinal Maneuver details ###
# The plugin responsible for planning the active longitudinal maneuver
string longitudinal_plugin

# The type of the longitudinal maneuver
string longitudinal_maneuver

# The start distance of the active maneuver
# Meters
float64 longitudinal_start_dist

# The end distance of the active maneuver
# Meters
float64 longitudinal_end_dist

### Lateral Maneuver details ###
# These fields can be empty

# The plugin responsible for planning the active lateral maneuver
string lateral_plugin

# The type of the lateral maneuver
string lateral_maneuver

# The start distance of the active maneuver
# Meters
float64 lateral_start_dist

# The end distance of the active maneuver
# Meters
float64 lateral_end_dist

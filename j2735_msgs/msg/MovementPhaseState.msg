#
# MovementEvent.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DE_MovementPhaseState data element provides the overall current state of the movement (in many cases a
# signal state), including its core phase state and an indication of whether this state is permissive or protected.
#
# MovementPhaseState ::= ENUMERATED {
# -- Note that based on the regions and the operating mode not every
# -- phase will be used in all transportation modes and that not
# -- every phase will be used in all transportation modes
# unavailable (0),
# -- This state is used for unknown or error
# dark (1),
# -- The signal head is dark (unlit)
# -- Reds
# stop-Then-Proceed (2),
# -- Often called 'flashing red' in US
# -- Driver Action:
# -Stop vehicle at stop line.
# -Do not proceed unless it is safe.
# -- Note that the right to proceed either right or left
# -- it is safe may be contained in the lane description
# -- handle what is called a 'right on red'
# stop-And-Remain (3),
# -- e.g. called 'red light' in US
# -- Driver Action:
# -Stop vehicle at stop line.
# -Do not proceed.
# -- Note that the right to proceed either right or left
# -- it is safe may be contained in the lane description
# -- handle what is called a 'right on red'
# -- Greens
# pre-Movement (4),
# -- Not used in the US, red+yellow partly in EU
# -- Driver Action:
# -Stop vehicle.
# -Prepare to proceed (pending green)
# -(Prepare for transition to green/go)
# permissive-Movement-Allowed (5),
# -- Often called 'permissive green' in US
# -- Driver Action:
# -Proceed with caution,
# -must yield to all conflicting traffic
# -- Conflicting traffic may be present
# -- in the intersection conflict area
# protected-Movement-Allowed (6),
# -- Often called 'protected green' in US
# -- Driver Action:
# -Proceed, tossing caution to the wind,
# -in indicated (allowed) direction.
# -- Yellows / Ambers
# -- The vehicle is not allowed to cross the stop bar if it is possible
# -- to stop without danger.
# permissive-clearance (7),
# -- Often called 'permissive yellow' in US
# -- Driver Action:
# -Prepare to stop.
# -Proceed if unable to stop,
# -Clear Intersection.
# -- Conflicting traffic may be present
# -- in the intersection conflict area
# protected-clearance (8),
# -- Often called 'protected yellow' in US
# -- Driver Action:
# -Prepare to stop.
# -Proceed if unable to stop,
# -in indicated direction (to connected lane)
# -Clear Intersection.
# caution-Conflicting-Traffic (9)
# -- Often called 'flashing yellow' in US
# -- Often used for extended periods of time
# -- Driver Action:
# -Proceed with caution,
# -- Conflicting traffic may be present
# -- in the intersection conflict area
# }
# -- The above number assignments are not used with UPER encoding
# -- and are only to be used with DER or implicit encoding

uint8 movement_phase_state

uint8 UNAVAILABLE=0
uint8 DARK=1
uint8 STOP_THEN_PROCEED=2
uint8 STOP_AND_REMAIN=3
uint8 PRE_MOVEMENT=4
uint8 PERMISSIVE_MOVEMENT_ALLOWED=5
uint8 PROTECTED_MOVEMENT_ALLOWED=6
uint8 PERMISSIVE_CLEARANCE=7
uint8 PROTECTED_CLEARANCE=8
uint8 CAUTION_CONFLICTING_TRAFFIC=9


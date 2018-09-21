#
# AdvisorySpeedType.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DE_AdvisorySpeedType data element relates the type of travel to which a given speed refers. This element is
# typically used as part of an AdvisorySpeed data frame for signal phase and timing data.
#
# AdvisorySpeedType ::= ENUMERATED {
# none (0),
# greenwave (1),
# ecoDrive (2),
# transit (3),
# ...
# } -- Note: subject to further growth

uint8 advisory_speed_type

uint8 NONE=0
uint8 GREENWAVE=1
uint8 ECODRIVE=2
uint8 TRANSIT=3

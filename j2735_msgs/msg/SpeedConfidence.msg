#
# SpeedConfidence.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DE_SpeedConfidence data element is used to provide the 95% confidence level for the currently reported
# value of DE_Speed, taking into account the current calibration and precision of the sensor(s) used to measure and/or
# calculate the value.

# SpeedConfidence ::= ENUMERATED {
# unavailable (0), -- Not Equipped or unavailable
# prec100ms (1), -- 100 meters / sec
# prec10ms(2), -- 10 meters / sec
# prec5ms (3), -- 5 meters / sec
# prec1ms (4), -- 1 meters / sec
# prec0-1ms (5), -- 0.1 meters / sec
# prec0-05ms (6), -- -- 0.05 meters / sec
# prec0-01ms (7) -- -- 0.01 meters / sec
# }

uint8 speed_confidence

uint8 UNAVAILABLE=0
uint8 PREC100MS=1
uint8 PREC10MS=2
uint8 PREC5MS=3
uint8 PREC1MS=4
uint8 PREC0_1MS=5
uint8 PREC0_05MS=6
uint8 PREC0_01MS=7
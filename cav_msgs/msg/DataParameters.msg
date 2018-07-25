#
# MapData.msg
#
# J2735 2016 message format.
#
# @author Mae Fromm
# @version 0.1
#
# The DataParameters data frame is used to provide basic (static) information
# on how a map fragment was processed or determined.
#
# DataParameters ::= SEQUENCE {
# processMethod IA5String(SIZE(1..255))
# processAgency IA5String(SIZE(1..255))
# lastCheckedDate IA5String(SIZE(1..255))
# geoidUsed IA5String(SIZE(1..255))
# ...
# }
#

string process_method
string process_agency
string last_checked_date
string geoid_used
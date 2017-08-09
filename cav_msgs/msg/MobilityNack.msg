# MobilityNack.msg
#
# This is a negative acknowledgement message that specifically indicates a suspected
# problem with the communication of a prior message because it came across garbled
# (e.g. the checksum could not be verified).  It is NOT intended to be used to 
# indicate rejection of a plan.
#

# this is the standard header for all mobility messages, but in this case the
# checksum field is ignored
MobilityHeader header

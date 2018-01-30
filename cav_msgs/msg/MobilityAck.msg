# MobilityAck.msg
#
# General acknowledgement of some previously received message.
#

# MsgCRC ::= OCTET STRING (SIZE(2)) -- created with the CRC-CCITT polynomial
uint8[]  checksum

# the standard header for all mobility messages
cav_msgs/MobilityHeader header

# indicates the type of acknowledgement
cav_msgs/MobilityAckType agreement

# regurgitated verification string received in a previous MobilityGreeting message
# this field is used only if agreement = ACK_SESSION_VERIFICATION
string      verification_code

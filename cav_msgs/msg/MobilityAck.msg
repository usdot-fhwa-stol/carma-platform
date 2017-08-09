# MobilityAck.msg
#
# General acknowledgement of some previously received message.
#

# the standard header for all mobility messages
MobilityHeader header

# indicates the type of acknowledgement
MobilityAgreementType agreement

# regurgitated verification string received in a previous MobilityGreeting message
# this field is used only if agreement = ACK_SESSION_VERIFICATION
string      verification_code

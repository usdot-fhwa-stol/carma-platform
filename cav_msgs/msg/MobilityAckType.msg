# MobilityAckType.msg
#
# Enumeration values for the types of acknowledgement
#

# Variable which can be used to store a specific agreement type value.
uint8 type

uint8 UNKNOWN = 0
uint8 ACCEPT_WITH_PROPOSAL_COMING = 1
uint8 ACCEPT_WITH_EXECUTE = 2
uint8 EXECUTION_COMPLETE = 3
uint8 REJECT_WITH_COUNTERPROPOSAL_COMING = 4
uint8 REJECT = 5
uint8 REJECT_AND_TERMINATE_RELATIONSHIP = 6
uint8 REJECT_WITH_SAFETY_CONCERN = 7

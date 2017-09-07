# PlanStatus.msg
#
# This message describes the current state of an ongoing mobility negotiation.
#

# The negotiation id which this status reports on
uint16 negotiation_id

# Enum describing the current state of a of negotiation. Can be one of the constants below
uint8 status

# Request for negotiation has been broadcast and host vehicle is awaiting connection
uint8 REQUESTING_PEER=0
# Establishing a session between this and another vehicle
uint8 ESTABLISHING_SESSION=1
# This vehicle is awaiting a proposal
uint8 AWAITING_PROPOSAL=2
# Remote vehicle is processing the proposed plan
uint8 AWAITING_PLAN_RESPONSE=2
# Plan has been accepted
uint8 ACCEPTED=2
# Plan has been accepted and is now actively being run
uint8 ACTIVE=3
# Plan was rejected but a counter proposal is incoming
uint8 AWAITING_COUNTER_PLAN=4
# Plan rejected
uint8 FAILED=254
# Plan completed
uint8 SUCCESS=255
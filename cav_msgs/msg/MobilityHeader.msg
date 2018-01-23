# MobilityHeader.msg
#
# This is a helper message used by all mobility messages.  It defines the basic
# information about the sender and receiver and the "plan" that they are talking about.
#

# sender's static GUID (represents that vehicle throughout the current relationship)
string      sender_id

# intended recipient's static GUID (constant throughout current relationship)
string      recipient_id

# a random value that identifies this particular plan for future reference
string      plan_id

# the date and time when this message is created
uint64        timestamp

# MobilityHeader.msg
#
# This is a helper message used by all mobility messages.  It defines the basic
# information about the sender and receiver, the plan that they are
#  talking about and the message creation timestamp.

# sender's static ID which is its license plate
# Example: "USDOT-49096" (the maximum length is 14)
string  sender_id

# recipient's static ID
# Empty string indicates a broadcast message
string  recipient_id

# sender's dynamic ID which is its BSM id in hex string
# Example: "FFFFFFFF"
string sender_bsm_id

# random GUID that identifies this particular plan for future reference
# Example: "b937d2f6-e618-4867-920b-c1f74f98ef1f"
string  plan_id

# time when this message is created
# A UNIX timestamp in milliseconds, which indicates the time elapsed since UNIX epoch
# Example: "9223372036854775807"
uint64  timestamp

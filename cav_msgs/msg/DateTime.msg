# DateTime.msg
#
# This is a message of date and time in DSRC style.
# It is a compound value consisting of finite-length sequences of
# integers (not characters) of the form: "yyyy, mm, dd, hh, mm, ss (sss+)"

# a integer value, representing the year according to the Gregorian calendar date syatem
uint16  year

# a integer value, representing the month within a year
uint8  month

# a integer value, representing the dates within a month
uint8 day

# a integer value, representing the hours within a day
uint8 hour

# a integer value, representing the minutes within a hour
uint8 minute

# a integer value, representing the milliseconds within a minut
uint32 second

# a signed integer values, representing an hour and minute value set
# from -14:00 to 14:00, representing all the world's local time zones in units of minutes
int16 offset

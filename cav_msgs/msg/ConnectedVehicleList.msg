# ConnectVehicleList.msg
#
# This message connects vehicles by BSM temp ids to ExternalObjects
#
# The two lists in this message are parrallel in that objects[i]
# corresponds to bsm_temp_ids[i]

cav_msgs/ExternalObject[] objects
uint16[] bsm_temp_ids
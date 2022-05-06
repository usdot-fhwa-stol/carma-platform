# roadway_objects

The roadway_objects node is responsibe for processing objects contained within a received carma_perception_msgs/msg/ExternalObjectsList message and mapping each object to a lanelet that it is located within. After mapping each object to a lanelet, this information is published to the CARMA system in a carma_perception_msgs/msg/RoadwayObstacleList message.
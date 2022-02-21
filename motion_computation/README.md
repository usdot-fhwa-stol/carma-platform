# motion_computation

The motion_computation package contains a node that subscribes to ROS topics containing position, velocity, and other relevant information for external objects. Using this received data, this node predicts the future location of each of these external objects, and publishes those predictions for other nodes within the CARMA system to utilize.

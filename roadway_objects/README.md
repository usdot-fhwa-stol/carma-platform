# Roadway objects Package

The roadway_objects node is responsible for processing objects contained within a received `carma_perception_msgs/msg/ExternalObjectsList` message and mapping each object to a lanelet that it is located within. After mapping each object to a lanelet, this information is published to the CARMA system in a `carma_perception_msgs/msg/RoadwayObstacleList` message.

Link to detailed design document on Confluence: [Click Here][confluence_link]

[confluence_link]: https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/2172551387/Carma-system-3.10.0+Detailed+Design+-+World+Model+-+Roadway+Object


## Documentation

For information about the Package's Nodes, Launch configurations, and other ROS 2 specific items, check out the
[package design][package_design_link] page.

[package_design_link]: docs/package_design.md

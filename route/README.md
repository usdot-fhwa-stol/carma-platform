# route

The Route package is one of the nodes that is part of the CARMA Platform Guidance system, and it provides the following functionality:

'Route Generation’: This provides the list of available routes and provides vehicle travel route description and management. It is a standalone ROS node in the CARMA 3 platform. For development and research purpose, it either loads a destination from a file system, or receives destination information from a ROS service call (which is useful when a route is received from infrastructure), then utilizes the Lanelet2 library routing module to generate a route and exposes it to the rest of the vehicle system. It tracks the progress of the current route, and handles re-routing if an active route is invalidated.

‘Route State Management’: This provides the current state of route following, including tracking and publishing vehicle cross track and down track distances along the active route.

Link to detailed design document on Confluence: [Click Here](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/1324122258/Detailed+Design+-+Route)
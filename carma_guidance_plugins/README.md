# carma_guidance_plugins

This package provides a set of base classes to implement CARMA Platform Guidance Plugins API. You can read about plugins in the [CARMA Platform Architecture](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/89587713/CARMA+Platform+System+Architecture). The design of these base classes can be found [here](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/2182545409/Detailed+Design+-+Plugin+Library). Using this library is not required as the plugin API is implemented entirely through ROS interfaces, however, using this package will minimize implementation errors.

NOTE: At the moment these bases classes are single threaded only.

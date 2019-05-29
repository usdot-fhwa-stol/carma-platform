#!/bin/sh

rosservice call /carma/route/set_active_route "routeID: 'TFHRC Circle'"
rosservice call /carma/route/start_active_route
rosservice call /carma/guidance/set_guidance_active True

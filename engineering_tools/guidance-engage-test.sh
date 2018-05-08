#!/bin/sh

rosservice call /saxton_cav/route/set_active_route "routeID: 'TFHRC Circle'"
rosservice call /saxton_cav/route/start_active_route
rosservice call /saxton_cav/guidance/set_guidance_active True

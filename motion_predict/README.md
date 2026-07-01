# motion_predict

This library implements some basic motion prediction functions meant to support external object handling under the motion_predict namespace. Currently there are two motion models implements a Constant Velocity (CV) model and a Constant Turn-Rate and Velocity (CTRV) model. See the motion_predict.h and predict_ctrv.h files for the relevant interfaces.

> [!NOTE]
> Both motion models expect the objects' twist.linear data to be in a map frame as opposed to object's base_link commonly used in ROS https://www.ros.org/reps/rep-0105.html#base-link.
> For example:
> - pose.pose.x and pose.pose.y expected to be the location of the object in a map frame
> - twist.linear.x and twist.linear.y indicate the heading of the object in a map frame. Therefore, yaw value in CTRV or CV is the angle of the vector from this x,y, which is respect to the map frame.
>
> This means that:
> - pose.orientation is not meaningfully used except to pass it on. Usually this value aligns with the yaw calculated before, but can be different.
>
> See this open issue for the reasoning and current limitation: https://github.com/usdot-fhwa-stol/carma-platform/issues/2401

Link to detailed design on Confluence: https://usdot-carma.atlassian.net/l/c/e3U5DXZi

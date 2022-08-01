#include "itm_service.h"
#include <ros/ros.h>
#include <vector>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <cav_srvs/PlanTrajectory.h>
namespace intersection_transit_maneuvering
{

    Servicer::Servicer(){};

    bool Servicer::call(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
    {
       return client.call(req,resp);

    }
    void Servicer::set_client(ros::ServiceClient& srv_client)
    {
        client = srv_client;
    }
    
}
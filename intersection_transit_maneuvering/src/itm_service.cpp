#include "itm_service.h"
#include <ros/ros.h>
#include <vector>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
namespace itm_servicer
{

    Servicer::Servicer();

    bool Servicer::call(const std::string& service_name,cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
    {
        client.call(service_name,req,resp);

    }
    void Servicer::setClient(ros::ServiceClient srv_client)
    {
        client = srv_client;
    }
    
}
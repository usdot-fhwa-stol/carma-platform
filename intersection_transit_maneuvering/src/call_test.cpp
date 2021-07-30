#include "call_test.h"
#include <ros/ros.h>

namespace call_test
{

    bool CallTest::call(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
    {
        request = req;
        response = resp;
        
        return true;
    }

    cav_srvs::PlanTrajectoryRequest CallTest::getRequest()
    {
        return request;
    }
    
    cav_srvs::PlanTrajectoryResponse CallTest::getResponse()
    {
        return response;
    }



}
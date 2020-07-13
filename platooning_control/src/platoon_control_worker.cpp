#include "platoon_control_worker.hpp"


namespace platoon_control
{
   
	PlatoonControlWorker::PlatoonControlWorker(){}

	double PlatoonControlWorker::getLastSpeedCommand() {
        return lastCmdSpeed;
    }

    void PlatoonControlWorker::generateSpeed(double timeStamp) {
    	PlatoonMember leader = getLeader();
    	if(leader.staticId != "") {
            double controllerOutput = 0.0;


	        double leaderCurrentPosition = leader.vehiclePosition;
	        ROS_DEBUG("The current leader position is " , leaderCurrentPosition);
	        double hostVehiclePosition = getCurrentDowntrackDistance();
	        double hostVehicleSpeed = getCurrentSpeed();

	        ROS_DEBUG("The host vehicle speed is + " , hostVehicleSpeed , " and its position is " , hostVehiclePosition);
	        // If the host vehicle is the fifth vehicle and it is following the third vehicle, the leader index here is 2
	        // vehiclesInFront should be 2, because number of vehicles in front is 4, then numOfVehiclesGaps = VehicleInFront - leaderIndex   
	        int leaderIndex;//????? = plugin_.platoonManager.getIndexOf(leader);
	        int numOfVehiclesGaps;//??????? = plugin_.platoonManager.getNumberOfVehicleInFront() - leaderIndex;
	        ROS_DEBUG("The host vehicle have " , numOfVehiclesGaps ," vehicles between itself and its leader (includes the leader)");
	        desiredGap_ = std::max(hostVehicleSpeed * timeHeadway * numOfVehiclesGaps, standStillHeadway * numOfVehiclesGaps);
	        ROS_DEBUG("The desired gap with the leader is " , desiredGap_);
	        ROS_DEBUG("Based on raw radar, the current gap with the front vehicle is " , getDistanceToFrontVehicle());
	        double desiredHostPosition = leaderCurrentPosition - desiredGap_;
	        ROS_DEBUG("The desired host position and the setpoint for pid controller is " , desiredHostPosition);
	        // PD controller is used to adjust the speed to maintain the distance gap between the subject vehicle and leader vehicle
	        // Error input for PD controller is defined as the difference between leaderCurrentPosition and desiredLeaderPosition
	        // A positive error implies that that the two vehicles are too far and a negative error implies that the two vehicles are too close
	        // The summation of the leader vehicle command speed and the output of PD controller will be used as speed commands
	        // The command speed of leader vehicle will act as the baseline for our speed control
	        
	        // distanceGapController_.changeSetpoint(desiredHostPosition);
	        // Signal<Double> signal = new Signal<Double>(hostVehiclePosition, timeStamp);
	        controllerOutput = pid_ctrl_->calculate(desiredHostPosition, hostVehiclePosition);//; = speedController_.apply(signal).get().getData();

		    double adjSpeedCmd = controllerOutput + leader.commandSpeed;
	        ROS_DEBUG("Adjusted Speed Cmd = " , adjSpeedCmd , "; Controller Output = " , controllerOutput
	        	, "; Leader CmdSpeed= " , leader.commandSpeed , "; Adjustment Cap " , adjustmentCap);
	            // After we get a adjSpeedCmd, we apply three filters on it if the filter is enabled
	            // First: we do not allow the difference between command speed of the host vehicle and the leader's commandSpeed higher than adjustmentCap
	        if(enableMaxAdjustmentFilter) {
                if(adjSpeedCmd > leader.commandSpeed + adjustmentCap) {
                    adjSpeedCmd = leader.commandSpeed + adjustmentCap;
                } else if(adjSpeedCmd < leader.commandSpeed - adjustmentCap) {
                    adjSpeedCmd = leader.commandSpeed - adjustmentCap;
                }
                ROS_DEBUG("The adjusted cmd speed after max adjustment cap is " , adjSpeedCmd , " m/s");
            }
            // Second: we do not exceed the local speed limit
            if(enableLocalSpeedLimitFilter) {
                // SpeedLimit limit  = pluginServiceLocator_.getRouteService().getSpeedLimitAtLocation(pluginServiceLocator_.getRouteService().getCurrentDowntrackDistance());???
                double localSpeedLimit = adjSpeedCmd;
                // if(limit != null) {??
                //     localSpeedLimit = limit.getLimit();
                //     ROS_DEBUG("The local speed limit is " , localSpeedLimit , ", cap adjusted speed to speed limit if necessary");
                // } else {
                //     ROS_WARN("Cannot find local speed limit in current location" , getCurrentDowntrackDistance());
                // }
                adjSpeedCmd = std::min(std::max(adjSpeedCmd, 0.0), localSpeedLimit);
                ROS_DEBUG("The speed command after local limit cap is: " , adjSpeedCmd , " m/s");
            }
            // Third: we allow do not a large gap between two consecutive speed commands
            if(enableMaxAccelFilter) {
                // if(!lastCmdSpeed.isPresent()) {
                //     lastCmdSpeed = Optional.of(plugin_.getLastSpeedCmd());
                // }
                double max = lastCmdSpeed + (maxAccel * (CMD_TIMESTEP / 1000.0));
                double min = lastCmdSpeed - (maxAccel * (CMD_TIMESTEP / 1000.0));
                if(adjSpeedCmd > max) {
                    adjSpeedCmd = max; 
                } else if (adjSpeedCmd < min) {
                    adjSpeedCmd = min;
                }
                lastCmdSpeed = adjSpeedCmd;//Optional.of(adjSpeedCmd);
                ROS_DEBUG("The speed command after max accel cap is: " , adjSpeedCmd , " m/s");
            }
            speedCmd_ = adjSpeedCmd;//speedCmd_.set(adjSpeedCmd);
            ROS_DEBUG("A speed command is generated from command generator: " , speedCmd_ , " m/s");

        }

        else {
            // TODO if there is no leader available, we should change back to Leader State and re-join other platoon later
            // speedCmd_.set(plugin_.getManeuverInputs().getCurrentSpeed());
            // distanceGapController_.reset();
        }


    }

    void PlatoonControlWorker::generateSteer(double timeStamp){
    	
    	double steerCmd = 0;
    	steerCmd = pp_->apply();



    }

    // ???????????????
    PlatoonMember PlatoonControlWorker::getLeader(){
    	return platoon_leader;
    }

    double PlatoonControlWorker::getCurrentSpeed(){
    	return currentSpeed;
    }

    double PlatoonControlWorker::getCurrentDowntrackDistance(){
    	return currentDTD;
    }

    double PlatoonControlWorker::getDistanceToFrontVehicle(){
    	return dist_to_front_vehicle;
    }




}
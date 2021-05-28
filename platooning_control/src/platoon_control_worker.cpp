#include "platoon_control_worker.hpp"


namespace platoon_control
{
   
	PlatoonControlWorker::PlatoonControlWorker()
    {
        pid_ctrl_ = PIDController();
        pp_ = PurePursuit();

    }

    void PlatoonControlWorker::updateConfigParams(PlatooningControlPluginConfig new_config)
    {
        ctrl_config = new_config;
        pid_ctrl_.config_ = new_config;
        pp_.config_ = new_config;
    }

	double PlatoonControlWorker::getLastSpeedCommand() const {
        return speedCmd_;
    }

    void PlatoonControlWorker::setInitialPose(const geometry_msgs::PoseStamped msg)
	{
		initial_pose_ = msg.pose;
	}

    void PlatoonControlWorker::setCurrentPose(const geometry_msgs::PoseStamped msg)
	{
		current_pose_ = msg.pose;
        // ROS_DEBUG_STREAM("pose x: " << current_pose_.position.x << "pose y: " << current_pose_.position.y);
	}

    void PlatoonControlWorker::generateSpeed(const cav_msgs::TrajectoryPlanPoint& point)
    {
        
        PlatoonLeaderInfo leader = platoon_leader;
    	if(leader.staticId != "") {
            double controllerOutput = 0.0;


	        double leaderCurrentPosition = leader.vehiclePosition;
	        ROS_DEBUG_STREAM("The current leader position is " << leaderCurrentPosition);
	        double hostVehiclePosition = getCurrentDowntrackDistance(point);
	        double hostVehicleSpeed = currentSpeed;

	        ROS_DEBUG_STREAM("The host vehicle speed is " << hostVehicleSpeed << " and its position is " << hostVehiclePosition);
	        // If the host vehicle is the fifth vehicle and it is following the third vehicle, the leader index here is 2
	        // vehiclesInFront should be 2, because number of vehicles in front is 4, then numOfVehiclesGaps = VehicleInFront - leaderIndex   
	        int leaderIndex = leader.leaderIndex;////TODO: Communicate leader index in the platoon (plugin_.platoonManager.getIndexOf(leader);)
	        int numOfVehiclesGaps = leader.NumberOfVehicleInFront - leaderIndex;////TODO: Communicate vehicles ahead in the platoon (plugin_.platoonManager.getNumberOfVehicleInFront() - leaderIndex;)
	        ROS_DEBUG_STREAM("The host vehicle have " << numOfVehiclesGaps << " vehicles between itself and its leader (includes the leader)");
	        double desiredGap = std::max(hostVehicleSpeed * ctrl_config.timeHeadway * numOfVehiclesGaps, ctrl_config.standStillHeadway * numOfVehiclesGaps);
	        ROS_DEBUG_STREAM("The desired gap with the leader is " << desiredGap);
	        // ROS_DEBUG("Based on raw radar, the current gap with the front vehicle is " , getDistanceToFrontVehicle());
	        double desiredHostPosition = leaderCurrentPosition - desiredGap;
	        ROS_DEBUG_STREAM("The desired host position and the setpoint for pid controller is " << desiredHostPosition);
	        // PD controller is used to adjust the speed to maintain the distance gap between the subject vehicle and leader vehicle
	        // Error input for PD controller is defined as the difference between leaderCurrentPosition and desiredLeaderPosition
	        // A positive error implies that that the two vehicles are too far and a negative error implies that the two vehicles are too close
	        // The summation of the leader vehicle command speed and the output of PD controller will be used as speed commands
	        // The command speed of leader vehicle will act as the baseline for our speed control
	        
	        controllerOutput = pid_ctrl_.calculate(desiredHostPosition, hostVehiclePosition);//; = speedController_.apply(signal).get().getData();

		    double adjSpeedCmd = controllerOutput + leader.commandSpeed;
	        ROS_DEBUG_STREAM("Adjusted Speed Cmd = " << adjSpeedCmd << "; Controller Output = " << controllerOutput
	        	<< "; Leader CmdSpeed= " << leader.commandSpeed << "; Adjustment Cap " << ctrl_config.adjustmentCap);
	            // After we get a adjSpeedCmd, we apply three filters on it if the filter is enabled
	            // First: we do not allow the difference between command speed of the host vehicle and the leader's commandSpeed higher than adjustmentCap
	        if(enableMaxAdjustmentFilter) {
                if(adjSpeedCmd > leader.commandSpeed + ctrl_config.adjustmentCap) {
                    adjSpeedCmd = leader.commandSpeed + ctrl_config.adjustmentCap;
                } else if(adjSpeedCmd < leader.commandSpeed - ctrl_config.adjustmentCap) {
                    adjSpeedCmd = leader.commandSpeed - ctrl_config.adjustmentCap;
                }
                ROS_DEBUG_STREAM("The adjusted cmd speed after max adjustment cap is " << adjSpeedCmd << " m/s");
            }
        
            // Third: we allow do not a large gap between two consecutive speed commands
            if(enableMaxAccelFilter) {
                
                double max = lastCmdSpeed + (ctrl_config.maxAccel * (ctrl_config.CMD_TIMESTEP / 1000.0));
                double min = lastCmdSpeed - (ctrl_config.maxAccel * (ctrl_config.CMD_TIMESTEP / 1000.0));
                if(adjSpeedCmd > max) {
                    adjSpeedCmd = max; 
                } else if (adjSpeedCmd < min) {
                    adjSpeedCmd = min;
                }
                lastCmdSpeed = adjSpeedCmd;
                ROS_DEBUG_STREAM("The speed command after max accel cap is: " << adjSpeedCmd << " m/s");
            }
            speedCmd_ = adjSpeedCmd;
            ROS_DEBUG_STREAM("A speed command is generated from command generator: " << speedCmd_ << " m/s");

        }

        else {
            // TODO if there is no leader available, we should change back to Leader State and re-join other platoon later
            ROS_DEBUG_STREAM("There is no leader available");
            speedCmd_ = currentSpeed;
            pid_ctrl_.reset();
        }

        lastCmdSpeed = speedCmd_;

    }

    void PlatoonControlWorker::generateSteer(const cav_msgs::TrajectoryPlanPoint& point){
        pp_.current_pose_ = current_pose_;
        pp_.velocity_ = currentSpeed;
    	steerCmd_ = pp_.calculateSteer(point);
    }

    // TODO get the actual leader from strategic plugin
    void PlatoonControlWorker::setLeader(const PlatoonLeaderInfo& leader){
    	platoon_leader = leader;
    }

    void  PlatoonControlWorker::setCurrentSpeed(double speed){
    	currentSpeed = speed;
    }

    double PlatoonControlWorker::getCurrentDowntrackDistance(const cav_msgs::TrajectoryPlanPoint& point) {
        
        double x_diff = (point.x - initial_pose_.position.x);
		double y_diff = (point.y - initial_pose_.position.y);
		double dist = std::sqrt(x_diff * x_diff + y_diff * y_diff);
    	return dist;
        ROS_DEBUG_STREAM("down track distance: " << dist);
    }

    // TODO Get information about front vehicle from world model (if needed)
    // double PlatoonControlWorker::getDistanceToFrontVehicle(){
    // 	return dist_to_front_vehicle;
    // }




}
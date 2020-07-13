#pragma once

#include <ros/ros.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/PlanType.h>
#include "pid_controller.hpp"
#include "pure_pursuit.hpp"







namespace platoon_control
{

	struct PlatoonMember{
            // Static ID is permanent ID for each vehicle
            std::string staticId;
            // Current BSM Id for each CAV
            std::string bsmId;
            // Vehicle real time command speed in m/s
            double commandSpeed;
            // Actual vehicle speed in m/s
            double vehicleSpeed;
            // Vehicle current down track distance on the current route in m
            double vehiclePosition;
            // The local time stamp when the host vehicle update any informations of this member
            long   timestamp;

            // PlatoonMember(std::string staticId, std::string bsmId, double commandSpeed, double vehicleSpeed, double vehiclePosition, long timestamp): staticId(staticId),
            // bsmId(bsmId), commandSpeed(commandSpeed), vehicleSpeed(vehicleSpeed), timestamp(timestamp) {}
        };


        // Leader info: platoonmember + leader index + number of vehicles in front


    class PlatoonControlWorker
    {
    public:

        PlatoonControlWorker();

        double getLastSpeedCommand();

        // Update speed commands based on the list of platoon members
        void generateSpeed(double timeStamp);
        void generateSteer(double timeStamp);

        PlatoonMember getLeader();

        double speedCmd;
        double currentSpeed; //???
        double adjustmentCap = 10.0;
        double lastCmdSpeed;

        double speedCmd_ = 0;
        double steerCmd_ = 0;

        PlatoonMember platoon_leader;




    private:

        PIDController *pid_ctrl_;
        PurePursuit *pp_;


    	double maxAccel = 2.5; // m/s/s

    	double desiredTimeGap = 1.0; // s

        double desiredGap_ = 0.0;


        long CMD_TIMESTEP = 100;

        double currentDTD;

    	double getCurrentSpeed();

        double getCurrentDowntrackDistance();

        double getDistanceToFrontVehicle();//????????????/*/*/**/*/*/

        // platooning_desired_time_headway"
        double timeHeadway = 2.0;

        // platooning standstillheadway"
        double standStillHeadway = 12.0;

        double dist_to_front_vehicle;

        bool enableMaxAdjustmentFilter = leaderSpeedCapEnabled;
        bool leaderSpeedCapEnabled = true;

        bool enableLocalSpeedLimitFilter = speedLimitCapEnabled;
        bool speedLimitCapEnabled = true;

        bool enableMaxAccelFilter = maxAccelCapEnabled;
        bool maxAccelCapEnabled = true;

    	
    

    

    };
}
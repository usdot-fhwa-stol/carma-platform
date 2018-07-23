#pragma once

#include "delphi_esr_can_client.h"
#include <cav_driver_utils/can/socketcan_interface/socketcan_interface.h>
#include <driver_application/driver_application.h>

#include <geometry_msgs/TwistStamped.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <vector>

/**
 * @brief Class encapsulates the ROS behaviour for the DelphiESRApplication
 */
class DelphiESRApplication : public cav::DriverApplication
{
    std::vector<std::string> api_;
    std::unique_ptr<DelphiESRCANClient<cav::SocketCANInterface>> client_;

    DelphiESRCANClient<cav::SocketCANInterface>::BurstData sensor_burst;
    volatile bool received_update_;
    std::mutex mutex_;

    delphi::DelphiRX4F1 rx1_;
    delphi::DelphiRX4F0 rx0_;

    ros::Publisher objects_pub_;
    ros::Subscriber velocity_sub_;
    ros::Time last_radar_update_;
    geometry_msgs::TwistStampedConstPtr last_velocity_update_;
    bool recv_velocities_;
    diagnostic_updater::Updater diag_updater_;
    std::string sensor_frame_;

public:

    DelphiESRApplication(int argc, char **argv, const std::string &name = "delphi_esr");
    virtual ~DelphiESRApplication();
    /**
     * @brief returns the api
     * @return
     */
    virtual std::vector<std::string> &get_api();

private:

    /**
     * @brief Publishes object updates received from handler, called after receiving a burst from sensor
     */
    void publish_updates();

    /**
     * @brief handles receipt of velocity messages from velocity topic.
     * @param msg
     */
    void velocity_cb(const geometry_msgs::TwistStampedConstPtr& msg);

    /**
     * @brief produce diagnostic status from burst message
     * @param stat
     */
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    /**
     * @brief handles the onBurstReceived event for the DelphiESRClient
     * @param data
     */
    void burst_cb(const DelphiESRCANClient<cav::SocketCANInterface>::BurstData& data);

    //cav::DriverApplication members
    virtual void initialize();
    virtual void pre_spin();
    virtual void post_spin();
    virtual void shutdown();

};
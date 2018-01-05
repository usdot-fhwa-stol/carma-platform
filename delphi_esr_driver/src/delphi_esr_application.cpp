#include "delphi_esr_application.h"

#include <cav_msgs/ExternalObjectList.h>
#include <cav_srvs/GetDriversWithCapabilities.h>
#include <boost/math/constants/constants.hpp>

namespace delphi
{
//these are helpers for the diagnostic messages
std::ostream& operator<<(std::ostream &o,SpeedDirection n) { switch(n){
        case SpeedDirection::Forward: return o<<"Forward";
        case SpeedDirection::Reverse: return o<<"Reverse";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,SteeringWheelSign n) { switch(n){
        case SteeringWheelSign::CounterClockwise: return o<<"CounterClockwise";
        case SteeringWheelSign::Clockwise: return o<<"Clockwise";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,Validity n) { switch(n){
        case Validity::UnavailableOrInvalid: return o<<"UnavailableOrInvalid";
        case Validity::AvailableAndValid: return o<<"AvailableAndValid";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,SensorModeCommand n) { switch(n){
        case SensorModeCommand::DoNotRadiate: return o<<"DoNotRadiate";
        case SensorModeCommand::Radiate: return o<<"Radiate";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,WindShieldWiperStatus n) { switch(n){
        case WindShieldWiperStatus::Off: return o<<"Off";
        case WindShieldWiperStatus::On: return o<<"On";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,RawDataEnable n) { switch(n){
        case RawDataEnable::Filtered: return o<<"Filtered";
        case RawDataEnable::Raw: return o<<"Raw";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,RadiatingStatus n) { switch(n){
        case RadiatingStatus::NotRadiating: return o<<"NotRadiating";
        case RadiatingStatus::Radiating: return o<<"Radiating";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,ErrorStatus n) { switch(n){
        case ErrorStatus::NotFailed: return o<<"NotFailed";
        case ErrorStatus::Failed: return o<<"Failed";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,BlockedStatus n) { switch(n){
        case BlockedStatus::NotBlocked: return o<<"NotBlocked";
        case BlockedStatus::Blocked: return o<<"Blocked";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,OverHeatStatus n) { switch(n){
        case OverHeatStatus::NotOverTemp: return o<<"NotOverTemp";
        case OverHeatStatus::OverTemp: return o<<"OverTemp";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,GroupingMode n) { switch(n){
        case GroupingMode::None: return o<<"None";
        case GroupingMode::MovingTargetsOnly: return o<<"MovingTargetsOnly";
        case GroupingMode::StationaryTargetsOnly: return o<<"StationaryTargetsOnly";
        case GroupingMode::StationayAndMovingTargets: return o<<"StationayAndMovingTargets";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,MRLRMode n) { switch(n){
        case MRLRMode::Reserved: return o<<"Reserved";
        case MRLRMode::MROnly: return o<<"MROnly";
        case MRLRMode::LROnly: return o<<"LROnly";
        case MRLRMode::MRAndLR: return o<<"MRAndLR";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,MedRangeMode n) { switch(n){
        case MedRangeMode::NoUpdate: return o<<"NoUpdate";
        case MedRangeMode::MRUpdate: return o<<"MRUpdate";
        case MedRangeMode::LRUpdate: return o<<"LRUpdate";
        case MedRangeMode::BothMRandLRUpdate: return o<<"BothMRandLRUpdate";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,TrackStatus n) { switch(n){
        case TrackStatus::NoTarget: return o<<"NoTarget";
        case TrackStatus::NewTarget: return o<<"NewTarget";
        case TrackStatus::NewUpdatedTarget: return o<<"NewUpdatedTarget";
        case TrackStatus::UpdatedTarget: return o<<"UpdatedTarget";
        case TrackStatus::CoastedTarget: return o<<"CoastedTarget";
        case TrackStatus::MergedTarget: return o<<"MergedTarget";
        case TrackStatus::InvalidCOastedTarget: return o<<"InvalidCOastedTarget";
        case TrackStatus::NewCoastedTarget: return o<<"NewCoastedTarget";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,InterfaceVersion n) { switch(n){
        case InterfaceVersion::TDPwith64tracks: return o<<"TDPwith64tracks";
        case InterfaceVersion::ADPDV1with64tracks: return o<<"ADPDV1with64tracks";
        case InterfaceVersion::ADPDV1forDV1MOART: return o<<"ADPDV1forDV1MOART";
        case InterfaceVersion::ADPDV2: return o<<"ADPDV2";
        case InterfaceVersion::ADPDV3: return o<<"ADPDV3";
        case InterfaceVersion::ADPPV: return o<<"ADPPV";
        case InterfaceVersion::ADPPV_2011: return o<<"ADPPV_2011";
        case InterfaceVersion::Reserved7: return o<<"Reserved7";
        case InterfaceVersion::Reserved8: return o<<"Reserved8";
        case InterfaceVersion::Reserved9: return o<<"Reserved9";
        case InterfaceVersion::Reserved10: return o<<"Reserved10";
        case InterfaceVersion::Reserved11: return o<<"Reserved11";
        case InterfaceVersion::Reserved12: return o<<"Reserved12";
        case InterfaceVersion::Reserved13: return o<<"Reserved13";
        case InterfaceVersion::Reserved14: return o<<"Reserved14";
        case InterfaceVersion::Reserved15: return o<<"Reserved15";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,PowerMode n) { switch(n){
        case PowerMode::DSPInit: return o<<"DSPInit";
        case PowerMode::RadiateOff: return o<<"RadiateOff";
        case PowerMode::RadiateOn: return o<<"RadiateOn";
        case PowerMode::DSPShutdown: return o<<"DSPShutdown";
        case PowerMode::DSPOff: return o<<"DSPOff";
        case PowerMode::HostShutdown: return o<<"HostShutdown";
        case PowerMode::Test: return o<<"Test";
        case PowerMode::Invalid: return o<<"Invalid";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,AlignmentStatus n) { switch(n){
        case AlignmentStatus::Off: return o<<"Off";
        case AlignmentStatus::Bust: return o<<"Bust";
        case AlignmentStatus::Success: return o<<"Success";
        case AlignmentStatus::FailNoTarget: return o<<"FailNoTarget";
        case AlignmentStatus::FailDevTooLarge: return o<<"FailDevTooLarge";
        case AlignmentStatus::FailVarTooLarge: return o<<"FailVarTooLarge";
        default: return o<<"(invalid value)"; }}
std::ostream& operator<<(std::ostream &o,TurnSignalStatus n) { switch(n){
        case TurnSignalStatus::None: return o<<"None";
        case TurnSignalStatus::Left: return o<<"Left";
        case TurnSignalStatus::Right: return o<<"Right";
        default: return o<<"(invalid value)"; }}
}

DelphiESRApplication::DelphiESRApplication(int argc, char **argv, const std::string &name) : DriverApplication(argc,argv,name), received_update_(false) {
}

DelphiESRApplication::~DelphiESRApplication() {
    client_.reset();
}

std::vector<std::string> &DelphiESRApplication::get_api() {
    return api_;
}

void DelphiESRApplication::publish_updates() {
    cav_msgs::ExternalObjectList out_list;
    out_list.header.frame_id = sensor_frame_;
    out_list.header.stamp = ros::Time::fromBoost(sensor_burst.stamp);

    for(auto it : sensor_burst.tracks)
    {
        if(it.track_status == delphi::TrackStatus::NoTarget) continue;

        cav_msgs::ExternalObject obj;

        obj.header.frame_id = sensor_frame_;
        obj.header.stamp = out_list.header.stamp;
        obj.presence_vector = cav_msgs::ExternalObject::SIZE_PRESENCE_VECTOR;
        obj.size.y = it.width / 2.0;
        obj.size.x = 0.0;
        obj.size.z = 0.0;

        obj.presence_vector |= cav_msgs::ExternalObject::POSE_PRESENCE_VECTOR;
        obj.pose.pose.position.x = it.range*cos(it.angle*boost::math::constants::pi<double>()/180.0);
        obj.pose.pose.position.y = it.range*sin(it.angle*boost::math::constants::pi<double>()/180.0);
        obj.pose.pose.position.z = 0;

        obj.presence_vector |= cav_msgs::ExternalObject::VELOCITY_PRESENCE_VECTOR;
        obj.velocity.twist.linear.x = it.range_rate*cos(it.angle*boost::math::constants::pi<double>()/180.0)
                                      + it.lat_rate*sin(it.angle*boost::math::constants::pi<double>()/180.0);

        obj.velocity.twist.linear.y = it.range_rate*sin(it.angle*boost::math::constants::pi<double>()/180.0)
                                      - it.lat_rate*cos(it.angle*boost::math::constants::pi<double>()/180.0);

        obj.velocity.twist.linear.z = 0;


        obj.presence_vector |= cav_msgs::ExternalObject::RANGE_RATE_PRESENCE_VECTOR;
        obj.range_rate = it.range_rate;

        obj.presence_vector |= cav_msgs::ExternalObject::AZIMUTH_RATE_PRESENCE_VECTOR;
        obj.azimuth_rate = -it.lat_rate/it.range;

        obj.presence_vector |= cav_msgs::ExternalObject::CONFIDENCE_PRESENCE_VECTOR;
        obj.confidence = it.track_status == delphi::TrackStatus::NewTarget ? 0.5 : 1.0;

        out_list.objects.push_back(obj);
    }

    objects_pub_.publish(out_list);

    diag_updater_.update();
}

void DelphiESRApplication::velocity_cb(const geometry_msgs::TwistStampedConstPtr &msg) {
    rx0_.speed = static_cast<float>(std::fabs(msg->twist.linear.x));
    rx0_.speed_direction = msg->twist.linear.x > 0 ? delphi::SpeedDirection::Forward : delphi::SpeedDirection::Reverse;
    rx0_.yaw_rate = static_cast<float>(msg->twist.angular.z * 180.0 / M_PI);

    rx0_.yaw_rate_validity = delphi::Validity::AvailableAndValid;
    rx1_.vehicle_speed_validity = delphi::Validity::AvailableAndValid;

    last_velocity_update_ = msg;
}

void DelphiESRApplication::initialize() {

    //are we getting velocities
    pnh_->param<bool>("use_velocities", recv_velocities_, false);
    if(recv_velocities_)
    {
        std::string topic = "velocity";
        pnh_->param<std::string>("velocity_topic",topic,topic);
        if(topic.empty())
        {
            ROS_INFO_STREAM("Looking for available velocity driver");
            ros::service::waitForService("get_drivers_with_capabilities");
            ros::ServiceClient client = nh_->serviceClient<cav_srvs::GetDriversWithCapabilities>("get_drivers_with_capabilities");
            cav_srvs::GetDriversWithCapabilities srv;
            srv.request.capabilities.push_back("position/velocity");
            while(srv.response.driver_data.empty())
                client.call(srv);

            topic = srv.response.driver_data.front();
        }

        velocity_sub_ = pnh_->subscribe<geometry_msgs::TwistStamped>(topic,1,&DelphiESRApplication::velocity_cb,this);
    }

    //initialize static variables sent as commands
    {
        pnh_->param<int32_t>("radius_curvature", rx0_.radius_curvature, rx0_.radius_curvature);
        rx1_.radiate_command = delphi::SensorModeCommand::Radiate;

        pnh_->param<float>("lateral_mounting_offset", rx1_.lateral_mounting_offset,rx1_.lateral_mounting_offset);
        pnh_->param<float>("alignment_angle_offset", rx1_.alignment_angle_offset,rx1_.alignment_angle_offset);
        pnh_->param<float>("angle_misalignment", rx1_.angle_misalignment,rx1_.angle_misalignment);
        pnh_->param<bool>("mmr_upside_down", rx1_.mmr_upside_down,rx1_.mmr_upside_down);
        pnh_->param<bool>("blockage_disable", rx1_.blockage_disable,rx1_.blockage_disable);
        pnh_->param<bool>("use_angle_misalignment", rx1_.use_angle_misalignment,rx1_.use_angle_misalignment);
        pnh_->param<bool>("lr_only_trasmit", rx1_.lr_only_trasmit,rx1_.lr_only_trasmit);
        pnh_->param<bool>("mr_only_transmit", rx1_.mr_only_transmit,rx1_.mr_only_transmit);
        pnh_->param<int32_t>("short_track_roc", rx1_.short_track_roc,rx1_.short_track_roc);
        int32_t temp;
        if(pnh_->getParam("maximum_tracks",temp))
            rx1_.maximum_tracks = static_cast<uint8_t>(temp);

        if(pnh_->getParam("high_yaw_angle", temp))
            rx1_.high_yaw_angle = static_cast<int8_t>(temp);

        if(pnh_->getParam("raw_data_enable", temp))
            rx1_.raw_data_enable = static_cast<delphi::RawDataEnable>(temp);

        if(pnh_->getParam("grouping_mode", temp))
            rx1_.grouping_mode = static_cast<delphi::GroupingMode>(temp);
    }

    //setup object topic
    pnh_->param<std::string>("sensor_frame", sensor_frame_,"delphi");
    objects_pub_=pnh_->advertise<cav_msgs::ExternalObjectList>("sensor/objects",10);
    api_.push_back(objects_pub_.getTopic());


    std::string can_device_name;
    pnh_->param<std::string>("device_name", can_device_name, "can0");
    //setup DelphiESRCANClient
    std::shared_ptr<cav::SocketCANInterface> device = std::make_shared<cav::SocketCANInterface>(can_device_name);
    client_.reset(new DelphiESRCANClient<cav::SocketCANInterface>(device));
    client_->init();

    client_->onBurstReceived.connect(std::bind(&DelphiESRApplication::burst_cb, this, std::placeholders::_1));
    client_->onError.connect([this](const boost::system::error_code&ec){ ROS_WARN_STREAM_THROTTLE(1, "Error with delphi client: " << ec.message());});

    //diagnostics updater
    diag_updater_.setHardwareID("Delphi-ESR2.5");
    diag_updater_.add("updater",this, &DelphiESRApplication::produce_diagnostics);

    auto status = getStatus();
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    status.sensor = true;

    setStatus(status);
    last_radar_update_ = ros::Time::now();

    spin_rate = 50;

}

void DelphiESRApplication::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"Delphi Status Messages");

    stat.add("Status1.comm_error",sensor_burst.esrStatus1.comm_error);
    stat.add("Status1.radius_curvature_calc",sensor_burst.esrStatus1.radius_curvature_calc);
    stat.add("Status1.vehicle_speed_calc",sensor_burst.esrStatus1.vehicle_speed_calc);
    stat.add("Status1.yaw_rate_calc",sensor_burst.esrStatus1.yaw_rate_calc);

    stat.add("Status2.temperature",(int)sensor_burst.esrStatus2.temperature);
    stat.add("Status2.maximum_tracks_ack",(int)sensor_burst.esrStatus2.maximum_tracks_ack);
    stat.add("Status2.sw_version",(int)sensor_burst.esrStatus2.sw_version);
    stat.add("Status2.internal_error",sensor_burst.esrStatus2.internal_error);
    stat.add("Status2.steering_angle_ack",(int)sensor_burst.esrStatus2.steering_angle_ack);
    stat.add("Status2.overheat_error",sensor_burst.esrStatus2.overheat_error);
    stat.add("Status2.grouping_mode",sensor_burst.esrStatus2.grouping_mode);
    stat.add("Status2.raw_data_mode",sensor_burst.esrStatus2.raw_data_mode);
    stat.add("Status2.xcvr_operational",sensor_burst.esrStatus2.xcvr_operational);
    stat.add("Status2.veh_spd_comp_factor",sensor_burst.esrStatus2.veh_spd_comp_factor);
    stat.add("Status2.yaw_rate_bias",sensor_burst.esrStatus2.yaw_rate_bias);
    stat.add("Status2.range_perf_error",sensor_burst.esrStatus2.range_perf_error);

    stat.add("Status3.sw_version_pld",(int)sensor_burst.esrStatus3.sw_version_pld);
    stat.add("Status3.serial_num",(int)sensor_burst.esrStatus3.serial_num);
    stat.add("Status3.interface_version",(int)sensor_burst.esrStatus3.interface_version);
    stat.add("Status3.hardware_version",(int)sensor_burst.esrStatus3.hardware_version);
    stat.add("Status3.host_sw_version",(int)sensor_burst.esrStatus3.host_sw_version);

    stat.add("Status4.LR_only_grating_lobe_detected",sensor_burst.esrStatus4.LR_only_grating_lobe_detected);
    stat.add("Status4.sidelobe_blockage",sensor_burst.esrStatus4.sidelobe_blockage);
    stat.add("Status4.partial_blockage",sensor_burst.esrStatus4.partial_blockage);
    stat.add("Status4.auto_align_angle",sensor_burst.esrStatus4.auto_align_angle);
    stat.add("Status4.path_ID_FCW_stat",(int)sensor_burst.esrStatus4.path_ID_FCW_stat);
    stat.add("Status4.path_ID_FCW_move",(int)sensor_burst.esrStatus4.path_ID_FCW_move);
    stat.add("Status4.path_ID_CMBB_stat",(int)sensor_burst.esrStatus4.path_ID_CMBB_stat);
    stat.add("Status4.path_ID_CMBB_move",(int)sensor_burst.esrStatus4.path_ID_CMBB_move);
    stat.add("Status4.path_ID_ACC",(int)sensor_burst.esrStatus4.path_ID_ACC);
    stat.add("Status4.mode",sensor_burst.esrStatus4.mode);
    stat.add("Status4.truck_target_detected",sensor_burst.esrStatus4.truck_target_detected);

    stat.add("Status5.supply_10v_a2d",(int)sensor_burst.esrStatus5.supply_10v_a2d);
    stat.add("Status5.supply_3p3v_a2d",(int)sensor_burst.esrStatus5.supply_3p3v_a2d);
    stat.add("Status5.supply_5vd_a2d",(int)sensor_burst.esrStatus5.supply_5vd_a2d);
    stat.add("Status5.supply_5va_a2d",(int)sensor_burst.esrStatus5.supply_5va_a2d);
    stat.add("Status5.temp2_a2d",(int)sensor_burst.esrStatus5.temp2_a2d);
    stat.add("Status5.temp1_a2d",(int)sensor_burst.esrStatus5.temp1_a2d);
    stat.add("Status5.ignp_a2d",(int)sensor_burst.esrStatus5.ignp_a2d);
    stat.add("Status5.swbatt_a2d",(int)sensor_burst.esrStatus5.swbatt_a2d);

    stat.add("Status6.wave_diff_a2d",(int)sensor_burst.esrStatus6.wave_diff_a2d);
    stat.add("Status6.supply_n5v_a2d",(int)sensor_burst.esrStatus6.supply_n5v_a2d);
    stat.add("Status6.supply_1p8v_a2d",(int)sensor_burst.esrStatus6.supply_1p8v_a2d);
    stat.add("Status6.sw_version_dsp_3rd_byte",(int)sensor_burst.esrStatus6.sw_version_dsp_3rd_byte);
    stat.add("Status6.service_alignment_updates_done",(int)sensor_burst.esrStatus6.service_alignment_updates_done);
    stat.add("Status6.vertical_alignment_updated",sensor_burst.esrStatus6.vertical_alignment_updated);
    stat.add("Status6.found_target",sensor_burst.esrStatus6.found_target);
    stat.add("Status6.recommend_unconverge",sensor_burst.esrStatus6.recommend_unconverge);
    stat.add("Status6.factory_align_status2",sensor_burst.esrStatus6.factory_align_status2);
    stat.add("Status6.factory_align_status1",sensor_burst.esrStatus6.factory_align_status1);
    stat.add("Status6.system_power_mode",sensor_burst.esrStatus6.system_power_mode);
    stat.add("Status6.factory_misalignment",sensor_burst.esrStatus6.factory_misalignment);
    stat.add("Status6.vertical_misalignment",sensor_burst.esrStatus6.vertical_misalignment);

    stat.add("Status7.active_fault0",(int)sensor_burst.esrStatus7.active_fault0);
    stat.add("Status7.active_fault1",(int)sensor_burst.esrStatus7.active_fault1);
    stat.add("Status7.active_fault2",(int)sensor_burst.esrStatus7.active_fault2);
    stat.add("Status7.active_fault3",(int)sensor_burst.esrStatus7.active_fault3);
    stat.add("Status7.active_fault4",(int)sensor_burst.esrStatus7.active_fault4);
    stat.add("Status7.active_fault5",(int)sensor_burst.esrStatus7.active_fault5);
    stat.add("Status7.active_fault6",(int)sensor_burst.esrStatus7.active_fault6);
    stat.add("Status7.active_fault7",(int)sensor_burst.esrStatus7.active_fault7);

    stat.add("Status8.history_fault0",(int)sensor_burst.esrStatus8.history_fault0);
    stat.add("Status8.history_fault1",(int)sensor_burst.esrStatus8.history_fault1);
    stat.add("Status8.history_fault2",(int)sensor_burst.esrStatus8.history_fault2);
    stat.add("Status8.history_fault3",(int)sensor_burst.esrStatus8.history_fault3);
    stat.add("Status8.history_fault4",(int)sensor_burst.esrStatus8.history_fault4);
    stat.add("Status8.history_fault5",(int)sensor_burst.esrStatus8.history_fault5);
    stat.add("Status8.history_fault6",(int)sensor_burst.esrStatus8.history_fault6);
    stat.add("Status8.history_fault7",(int)sensor_burst.esrStatus8.history_fault7);

    stat.add("Status9.path_id_acc_2",(int)sensor_burst.esrStatus9.path_id_acc_2);
    stat.add("Status9.path_id_acc_3",(int)sensor_burst.esrStatus9.path_id_acc_3);
    stat.add("Status9.filtered_xohp_of_acc_tgt",sensor_burst.esrStatus9.filtered_xohp_of_acc_tgt);
    stat.add("Status9.water_spray_id",(int)sensor_burst.esrStatus9.water_spray_id);
    stat.add("Status9.serial_num_3rd_byte",(int)sensor_burst.esrStatus9.serial_num_3rd_byte);
    stat.add("Status9.sideslip_angle",sensor_burst.esrStatus9.sideslip_angle);
    stat.add("Status9.avg_pwr_cwblkg",(int)sensor_burst.esrStatus9.avg_pwr_cwblkg);

}

void DelphiESRApplication::burst_cb(const DelphiESRCANClient<cav::SocketCANInterface>::BurstData &data) {
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if(!lock.owns_lock()) return;

    ROS_DEBUG_STREAM("Received burst from sensor");
    sensor_burst = data;
    received_update_ = true;
    last_radar_update_ = ros::Time::now();

}

void DelphiESRApplication::pre_spin() {
    if(received_update_)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        publish_updates();
        received_update_ = false;
    }

    if(ros::Time::now() - last_radar_update_ > ros::Duration(0.5))
    {
        ROS_WARN_STREAM_THROTTLE(1,"Not receiving updates from sensor");
        auto status = getStatus();
        status.status = cav_msgs::DriverStatus::FAULT;
        status.sensor = true;

        setStatus(status);
    }
    else if (getStatus().status != cav_msgs::DriverStatus::OPERATIONAL)
    {
        auto status = getStatus();
        status.status = cav_msgs::DriverStatus::OPERATIONAL;
        status.sensor = true;

        setStatus(status);
    }
}

void DelphiESRApplication::post_spin() {
    if(last_velocity_update_ && ros::Time::now() - last_velocity_update_->header.stamp > ros::Duration(0.1))
    {
        rx0_.yaw_rate_validity = delphi::Validity::UnavailableOrInvalid;
        rx1_.vehicle_speed_validity = delphi::Validity::UnavailableOrInvalid;
    }    

    client_->sendCommands(rx0_,rx1_);
}

void DelphiESRApplication::shutdown() {
    ROS_WARN_STREAM("Shutting down");
    client_.reset();
}

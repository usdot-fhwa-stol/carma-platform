/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "sensor_fusion.h"
#include "wgs84_utils.h"
#include "timer.h"
#include <cav_msgs/ConnectedVehicleList.h>

#include <cav_srvs/GetDriversWithCapabilities.h>
#include <cav_srvs/Bind.h>

#include <rosgraph_msgs/Clock.h>

namespace tf2
{

template<>
inline void doTransform(const cav_msgs::ExternalObject& in, cav_msgs::ExternalObject& out, const geometry_msgs::TransformStamped& transform)
{
    out = in;

    geometry_msgs::PoseStamped p_i, p_o;

    p_i.header = in.header;
    p_i.pose.position.x = in.pose.pose.position.x;
    p_i.pose.position.y = in.pose.pose.position.y;
    p_i.pose.position.z = in.pose.pose.position.z;

    p_i.pose.orientation.w = in.pose.pose.orientation.w;
    p_i.pose.orientation.x = in.pose.pose.orientation.x;
    p_i.pose.orientation.y = in.pose.pose.orientation.y;
    p_i.pose.orientation.z = in.pose.pose.orientation.z;

    doTransform< geometry_msgs::PoseStamped>(p_i, p_o, transform);

    out.pose.pose = p_o.pose;
    out.header = transform.header;
}

}

namespace torc
{
class SimTimer : public cav::Timer
{
    virtual boost::posix_time::ptime getTime() override {
        return ros::Time::now().toBoost();
    }
};
/**
 * @brief Simple merging of sizes.
 * @param tgt
 * @param src
 */
void mergeSize(geometry_msgs::Vector3& tgt, const geometry_msgs::Vector3& src, size_t merged_so_far)
{
    tgt.x = ((tgt.x*merged_so_far)+src.x)/(merged_so_far+1);
    tgt.y = ((tgt.y*merged_so_far)+src.y)/(merged_so_far+1);
    tgt.z = ((tgt.z*merged_so_far)+src.z)/(merged_so_far+1);
}

cav_msgs::ExternalObject toExternalObject(const TrackedObject& obj)
{
    cav_msgs::ExternalObject out;
    out.presence_vector = cav_msgs::ExternalObject::ID_PRESENCE_VECTOR;
    out.id = static_cast<unsigned short>(obj.id);

    out.presence_vector |= cav_msgs::ExternalObject::CONFIDENCE_PRESENCE_VECTOR;
    out.confidence = obj.confidence;

    out.presence_vector |= cav_msgs::ExternalObject::POSE_PRESENCE_VECTOR;
    out.pose.pose.position.x = obj.position[0];
    out.pose.pose.position.y = obj.position[1];
    out.pose.pose.position.z = obj.position[2];

    out.pose.pose.orientation.x = obj.orientation.x();
    out.pose.pose.orientation.y = obj.orientation.y();
    out.pose.pose.orientation.z = obj.orientation.z();
    out.pose.pose.orientation.w = obj.orientation.w();

    out.presence_vector |= cav_msgs::ExternalObject::VELOCITY_PRESENCE_VECTOR;
    out.velocity.twist.linear.x = obj.linear_velocity[0];
    out.velocity.twist.linear.y = obj.linear_velocity[1];
    out.velocity.twist.linear.z = obj.linear_velocity[2];

    out.velocity.twist.angular.x = obj.angular_velocity[0];
    out.velocity.twist.angular.y = obj.angular_velocity[1];
    out.velocity.twist.angular.z = obj.angular_velocity[2];

    size_t sizes_merged = 0;
    for(auto& it : obj.src_data)
    {
        if(it.second.get() == nullptr) continue;
        cav_msgs::ExternalObject src_obj;
        uint32_t serial_size;
        memcpy(&serial_size,it.second.get(),sizeof(uint32_t));
        ros::serialization::IStream stream(it.second.get()+sizeof(uint32_t), serial_size);
        ros::serialization::Serializer<cav_msgs::ExternalObject>::read(stream, src_obj);

        if((src_obj.presence_vector & cav_msgs::ExternalObject::SIZE_PRESENCE_VECTOR) != 0)
        {
            out.presence_vector |= cav_msgs::ExternalObject::SIZE_PRESENCE_VECTOR;
            mergeSize(out.size,src_obj.size,sizes_merged++);
        }

        if((src_obj.presence_vector & cav_msgs::ExternalObject::OBJECT_TYPE_PRESENCE_VECTOR) != 0)
        {
            out.presence_vector |= cav_msgs::ExternalObject::OBJECT_TYPE_PRESENCE_VECTOR;
            out.object_type = src_obj.object_type;
        }

        if((src_obj.presence_vector & cav_msgs::ExternalObject::RELATIVE_LANE_PRESENCE_VECTOR) != 0)
        {
            out.presence_vector |= cav_msgs::ExternalObject::RELATIVE_LANE_PRESENCE_VECTOR;
            out.relative_lane = src_obj.relative_lane;
        }

        if((src_obj.presence_vector & cav_msgs::ExternalObject::RANGE_RATE_PRESENCE_VECTOR))
        {
            out.presence_vector |= cav_msgs::ExternalObject::RANGE_RATE_PRESENCE_VECTOR;
            out.range_rate = src_obj.range_rate;            
        }

        if((src_obj.presence_vector & cav_msgs::ExternalObject::AZIMUTH_RATE_PRESENCE_VECTOR))
        {
            out.presence_vector |= cav_msgs::ExternalObject::AZIMUTH_RATE_PRESENCE_VECTOR;
            out.azimuth_rate = src_obj.azimuth_rate;            
        }

        if((src_obj.presence_vector & cav_msgs::ExternalObject::BSM_ID_PRESENCE_VECTOR) != 0)
        {
            out.presence_vector |= cav_msgs::ExternalObject::BSM_ID_PRESENCE_VECTOR;
            out.bsm_id = std::move(src_obj.bsm_id);
        }
    }

    return out;
}

TrackedObject toTrackedObject(const cav_msgs::ExternalObject& obj)
{
    TrackedObject ret;

    ret.id              = obj.id;
    ret.position        = Eigen::Vector3d(obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z);
    ret.dimensions      = Eigen::Vector3d(obj.size.x,obj.size.y,obj.size.z);
    ret.orientation.x() = obj.pose.pose.orientation.x;
    ret.orientation.y() = obj.pose.pose.orientation.y;
    ret.orientation.z() = obj.pose.pose.orientation.z;
    ret.orientation.w() = obj.pose.pose.orientation.w;

    ret.linear_velocity     = Eigen::Vector3d(obj.velocity.twist.linear.x,obj.velocity.twist.linear.y,obj.velocity.twist.linear.z);
    ret.angular_velocity    = Eigen::Vector3d(obj.velocity.twist.angular.x,obj.velocity.twist.angular.y,obj.velocity.twist.angular.z);
    ret.confidence          = obj.confidence;

    ret.presence_vector = obj.presence_vector;

    return ret;
}
}

int SensorFusionApplication::run() {
    nh_.reset(new ros::NodeHandle());
    pnh_.reset(new ros::NodeHandle("~"));
    ros::NodeHandle pnh("filtered");
    tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));

    pnh_->param<std::string>("inertial_frame_name",inertial_frame_name_,"odom");
    pnh_->param<std::string>("body_frame_name",body_frame_name_,"base_link");
    pnh_->param<std::string>("ned_frame_name",ned_frame_name_,"ned");
    pnh_->param<bool>("use_interface_mgr",use_interface_mgr_,false);

    bool use_sim_time;
    nh_->param<bool>("/use_sim_time", use_sim_time, false);

		//This use sim time fix was added to support rosbag playback. The issue is that the tracker is time based
		//and if a measurement comes in with an earlier time we assume it is old. So to support looping play back 
		//we need to reset the tracker and also use a special SimTimer to read time from the ROS timer rather than
		//the default timer.
    if(use_sim_time)
    {
        ROS_INFO_STREAM("Using Sim Time");
        tracker_.reset(new torc::ObjectTracker(std::make_shared<torc::SimTimer>()));
        sub_map_["/clock"] = nh_->subscribe<rosgraph_msgs::Clock>("/clock", 10, [this](const rosgraph_msgs::ClockConstPtr& msg)
        {
            static ros::Time time = msg->clock;
            if(msg->clock < time)
            {
                ROS_INFO_STREAM("Detected an early clock, resetting tracker");
                tracker_->reset();
            }

            time = msg->clock;
        });
    }else
    {
        tracker_.reset(new torc::ObjectTracker());
    }


    //setup dyn_recfg_server
    {
        dynamic_reconfigure::Server<sensor_fusion::SensorFusionConfig>::CallbackType f;
        f = std::bind(&SensorFusionApplication::dyn_recfg_cb,this,std::placeholders::_1,std::placeholders::_2);
        dyn_cfg_server_->setCallback(f);
    }

    ros::Subscriber bsm_sub = nh_->subscribe<cav_msgs::BSM>("bsm", 1000, &SensorFusionApplication::bsm_cb, this);

    if(use_interface_mgr_)
    {
        ROS_INFO_STREAM("Waiting for Interface Manager");
        ros::service::waitForService("get_drivers_with_capabilities");
        update_services_timer_ = nh_->createTimer(ros::Duration(5.0),[this](const ros::TimerEvent& ev){ update_subscribed_services(); },false, true);
        ROS_INFO_STREAM("Interface Manager available");
    }
		else
    {
		    //This allows us to manually set the topics to listen, rather than querying the interface manager. Topics
				//can be set through a xaml list in the launch file
				
        //odometry
        {
            ROS_INFO_STREAM("Odometry Topics");
            XmlRpc::XmlRpcValue v;
            pnh_->param("odometry_topics",v,v);
            for(int i = 0; i < v.size(); i++)
            {
                ROS_INFO_STREAM("Subscribing to "<< v[i]);
                sub_map_[v[i]] = nh_->subscribe<nav_msgs::Odometry>(v[i],10,[this](const ros::MessageEvent<nav_msgs::Odometry const>& msg){ odom_cb(msg);});
            }
        }

        //velocity
        {

            ROS_INFO_STREAM("Velocity Topics");
            XmlRpc::XmlRpcValue v;
            pnh_->param("velocity_topics",v,v);
            for(int i = 0; i < v.size(); i++)
            {
                ROS_INFO_STREAM("Subscribing to "<< v[i]);
                sub_map_[v[i]] = nh_->subscribe<geometry_msgs::TwistStamped>(v[i],10,[this](const ros::MessageEvent<geometry_msgs::TwistStamped>& msg){ velocity_cb(msg);});
            }
        }

        //navsatfix
        {
            ROS_INFO_STREAM("NavSatFix Topics");
            XmlRpc::XmlRpcValue v;
            pnh_->param("navsatfix_topics",v,v);
            for(int i = 0; i < v.size(); i++)
            {
                ROS_INFO_STREAM("Subscribing to "<< v[i]);
                sub_map_[v[i]] = nh_->subscribe<sensor_msgs::NavSatFix>(v[i],10,[this](const ros::MessageEvent<sensor_msgs::NavSatFix>& msg){ navsatfix_cb(msg);});
            }
        }

        //heading
        {
            ROS_INFO_STREAM("Heading Topics");
            XmlRpc::XmlRpcValue v;
            pnh_->param("heading_topics",v,v);
            for(int i = 0; i < v.size(); i++)
            {
                ROS_INFO_STREAM("Subscribing to "<< v[i]);
                sub_map_[v[i]] = nh_->subscribe<cav_msgs::HeadingStamped>(v[i],10,[this](const ros::MessageEvent<cav_msgs::HeadingStamped>& msg){ heading_cb(msg);});
            }
        }

        //objects
        {
            ROS_INFO_STREAM("Objects Topics");
            XmlRpc::XmlRpcValue v;
            pnh_->param("objects_topics",v,v);
            for(int i = 0; i < v.size(); i++)
            {
                ROS_INFO_STREAM("Subscribing to "<< v[i]);
                sub_map_[v[i]] =  nh_->subscribe<cav_msgs::ExternalObjectList>(v[i],10,[this,&v,i](const cav_msgs::ExternalObjectListConstPtr& msg){objects_cb_q_.push_back(std::make_pair(v[i], msg));});
            }
        }
    }

    odom_pub_       = pnh.advertise<nav_msgs::Odometry>("odometry",100);
    navsatfix_pub_  = pnh.advertise<sensor_msgs::NavSatFix>("nav_sat_fix",100);
    velocity_pub_   = pnh.advertise<geometry_msgs::TwistStamped>("velocity",100);
    heading_pub_    = pnh.advertise<cav_msgs::HeadingStamped>("heading",100);
    objects_pub_    = pnh.advertise<cav_msgs::ExternalObjectList>("tracked_objects",100);
    vehicles_pub_   = pnh.advertise<cav_msgs::ConnectedVehicleList>("tracked_vehicles",100);

    ros::Rate r(20);
    while(ros::ok())
    {
        ros::spinOnce();

        while(!objects_cb_q_.empty())
        {
            objects_cb(objects_cb_q_.front().second,objects_cb_q_.front().first);
            objects_cb_q_.pop_front();
        }

        publish_updates();
        r.sleep();
    }

    return 0;
}

void SensorFusionApplication::update_subscribed_services() {
    ROS_DEBUG_STREAM("Updating subscribed services");
    //odometry
    std::vector<std::string> ret = get_api("position/odometry");
    for(const std::string& it : ret)
    {
        if(sub_map_.find(it) == sub_map_.end())
            sub_map_[it] = nh_->subscribe<nav_msgs::Odometry>(it,1,[this](const ros::MessageEvent<nav_msgs::Odometry const>& msg){ odom_cb(msg);});
    }

    //nav_sat_fix
    ret = get_api("position/nav_sat_fix");
    for(const std::string& it : ret)
    {
        if(sub_map_.find(it) == sub_map_.end())
            sub_map_[it] = nh_->subscribe<sensor_msgs::NavSatFix>(it,1,[this](const ros::MessageEvent<sensor_msgs::NavSatFix const>&msg){ navsatfix_cb(msg);});
    }

    //heading
    ret = get_api("position/heading");
    for(const std::string& it : ret)
    {
        if(sub_map_.find(it) == sub_map_.end())
            sub_map_[it] = nh_->subscribe<cav_msgs::HeadingStamped>(it,1,[this](const ros::MessageEvent<cav_msgs::HeadingStamped>&  msg){ heading_cb(msg);});
    }

    //velocity
    ret = get_api("position/velocity");
    for(const std::string& it : ret)
    {
        if(sub_map_.find(it) == sub_map_.end())
            sub_map_[it] = nh_->subscribe<geometry_msgs::TwistStamped>(it,1,[this](const ros::MessageEvent<geometry_msgs::TwistStamped>& msg){ velocity_cb(msg);});
    }

    //tracked_objects
    ret = get_api("sensor/objects");
    for(const std::string& it : ret)
    {
        if(sub_map_.find(it) == sub_map_.end())
            sub_map_[it] = nh_->subscribe<cav_msgs::ExternalObjectList>(it,1,[this, it](const cav_msgs::ExternalObjectListConstPtr& msg){ objects_cb_q_.push_back(std::make_pair(it,msg));});
    }

}


std::vector<std::string> SensorFusionApplication::get_api(const std::string &name)
{

    ros::ServiceClient client = nh_->serviceClient<cav_srvs::GetDriversWithCapabilities>("get_drivers_with_capabilities");
    cav_srvs::GetDriversWithCapabilities srv;
    srv.request.capabilities.push_back(name);

    ROS_DEBUG_STREAM("Sending request to get_drivers_with_capabilities: " << srv.request);
    std::vector<std::string> ret;
    if(client.exists() && client.call(srv))
    {

	ROS_DEBUG_STREAM("get_drivers_with_capabilities returned: " << srv.response);
        //The service returns a list of drivers that have the api we provided
        for(std::string fqn : srv.response.driver_data)
        {
            size_t pos = fqn.find(name);
            std::string driverName = fqn.substr(0,pos);

            //Bond with the node if we haven't already
            //todo: make bonding configurable, evaluate if it is even necessary
//            if(bond_map_.find(driverName) == bond_map_.end())
//            {
//                ROS_DEBUG_STREAM("Bonding to node: " << driverName);
//                ros::ServiceClient bond_client = nh_->serviceClient<cav_srvs::Bind>(driverName+"/bind");
//                cav_srvs::Bind req;
//                req.request.id = boost::lexical_cast<std::string>(uuid_);
//
//                if(bond_client.call(req))
//                {
//                    bond_map_[driverName]= std::unique_ptr<bond::Bond>(new bond::Bond(driverName+"/bond",
//                                                                              boost::lexical_cast<std::string>(uuid_),
//                                                                              boost::bind(&SensorFusionApplication::on_broken_cb,
//                                                                                          this,
//                                                                                          driverName),boost::bind(
//                                    &SensorFusionApplication::on_connected_cb,this,driverName)));
//
//                    bond_map_[driverName]->start();
//                    if(!bond_map_[driverName]->waitUntilFormed(ros::Duration(1.0)))
//                    {
//                        ROS_ERROR_STREAM("Failed to form bond");
//                        continue;
//                    }
//
//                }
//            }

            //If we haven't subscribed to the topic formed by the name of the node and the service
            //add this topic to the return list
            if(sub_map_.find(fqn) == sub_map_.end())
                ret.push_back(fqn);
        }

    }
    else
    {
        ROS_WARN_STREAM_THROTTLE(2,"Unable to query service get_drivers_with_capabilites");
    }

    return ret;

}


void SensorFusionApplication::publish_updates() {

    if(!odom_map_.empty())
        odom_pub_.publish(odom_map_.begin()->second);

    if(!navsatfix_map_.empty())
        navsatfix_pub_.publish(navsatfix_map_.begin()->second);

    if(!heading_map_.empty())
        heading_pub_.publish(heading_map_.begin()->second);

    if(!velocity_map_.empty())
        velocity_pub_.publish(velocity_map_.begin()->second);

    cav_msgs::ConnectedVehicleList msg;
    std_msgs::Header header;
    header.frame_id = body_frame_name_;
    header.stamp = ros::Time::now();

    vehicles_pub_.publish(msg);

    if(tracker_->process() > 0 && !tracker_->tracked_sensor->objects.empty())
    {
        cav_msgs::ExternalObjectList list;
        list.header.stamp = ros::Time::fromBoost(tracker_->tracked_sensor->time_stamp);
        list.header.frame_id = inertial_frame_name_;
        for (auto& it : tracker_->tracked_sensor->objects)
        {
            cav_msgs::ExternalObject obj = toExternalObject(it);
            obj.header = list.header;
            list.objects.push_back(obj);
        }

        static uint64_t objects_published = 0;
        objects_published += list.objects.size();
        ROS_INFO_STREAM_THROTTLE(5.0, "Published " << objects_published << " so far");
        ROS_DEBUG_STREAM("Publish objects: " << list.objects.size());
        objects_pub_.publish(list);
    }
    else {
        ROS_DEBUG_STREAM_THROTTLE(1.0,"No tracked_objects. Publishing empty list");
        cav_msgs::ExternalObjectList list;
        list.header.stamp = ros::Time::now();
        list.header.frame_id = inertial_frame_name_;
        objects_pub_.publish(list);
    }
}

void SensorFusionApplication::objects_cb(const cav_msgs::ExternalObjectListConstPtr& msg,const std::string& topic_name) {

    std::hash<std::string> hash_fn;
    size_t hash = hash_fn(topic_name);
    //Get Transform from object measurement to inertial frame
    //All tracking should be done in inertial frame
    geometry_msgs::TransformStamped transformStamped;
    if(tf2_buffer_.canTransform(inertial_frame_name_,msg->header.frame_id,msg->header.stamp))
    {
        transformStamped = tf2_buffer_.lookupTransform(inertial_frame_name_,msg->header.frame_id,msg->header.stamp);
    }
    else if(tf2_buffer_.canTransform(inertial_frame_name_,msg->header.frame_id,ros::Time(0)))
    {
        ROS_DEBUG_STREAM("Using latest transform available");
        transformStamped = tf2_buffer_.lookupTransform(inertial_frame_name_,msg->header.frame_id,ros::Time(0));
    }
    else
    {
        ROS_WARN_STREAM("No transform available from " << inertial_frame_name_ << " to " << msg->header.frame_id);
        return;
    }

    std::vector<torc::TrackedObject> transformed_list;
    for(auto & it : msg->objects)
    {
        //Transform to inertial frame
        cav_msgs::ExternalObject obj;
        tf2::doTransform(it,obj,transformStamped);

        geometry_msgs::TwistStamped twistStamped;
        try
        {
            if(!twist_history_buffer_.getTwist(body_frame_name_,obj.header.stamp,twistStamped))
                continue;
        }catch(cav::ExtrapolationException e)
        {
            ROS_WARN_STREAM(e.what());
            twist_history_buffer_.getLatest(body_frame_name_,twistStamped);
        }

        if(obj.presence_vector & cav_msgs::ExternalObject::VELOCITY_PRESENCE_VECTOR)
        {
            tf2::Vector3 obj_v;
            tf2::convert(obj.velocity.twist.linear,obj_v);

            tf2::Vector3 body_v;
            tf2::convert(twistStamped.twist.linear,body_v);

            obj_v += body_v;
            tf2::Quaternion rotation;
            tf2::convert(transformStamped.transform.rotation,rotation);
            tf2::Vector3 obj_v_rot = tf2::quatRotate(rotation,obj_v);
            tf2::convert(obj_v_rot,obj.velocity.twist.linear);
        }

        transformed_list.push_back(torc::toTrackedObject(obj));
        torc::TrackedObject& back = transformed_list.back();
        uint32_t serial_size = ros::serialization::serializationLength(obj);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size+sizeof(uint32_t)]);
        back.src_data[hash] = buffer;
        memcpy(back.src_data[hash].get(),&serial_size,sizeof(uint32_t));
        ros::serialization::OStream stream(back.src_data[hash].get()+sizeof(uint32_t),serial_size);
        ros::serialization::serialize(stream,obj);
    }


    tracker_->addObjects(transformed_list.begin(),transformed_list.end(),hash,transformStamped.header.stamp.toBoost());
}

void SensorFusionApplication::bsm_cb(const cav_msgs::BSMConstPtr &msg) {
    ROS_DEBUG_STREAM_NAMED("bsm_logger","Received bsm message: " << msg);
    if(heading_map_.empty() || navsatfix_map_.empty())
    {
        ROS_DEBUG_STREAM_NAMED("bsm_logger","Received bsm before heading and navsatfix updated unable to process msg");
        return;
    }
    auto hash = std::hash<std::string>();
    size_t src_id = hash("bsm_objects");
    cav_msgs::ExternalObject obj;
    obj.header.frame_id = inertial_frame_name_;
    obj.header.stamp = msg->header.stamp;
    obj.presence_vector = 0;

    geometry_msgs::TransformStamped odom_tf, ned_odom_tf;
    try {
        odom_tf = tf2_buffer_.lookupTransform(inertial_frame_name_, body_frame_name_, msg->header.stamp);
        ned_odom_tf = tf2_buffer_.lookupTransform(ned_frame_name_, inertial_frame_name_, msg->header.stamp);
    } catch (tf2::TransformException&ex) {
        ROS_WARN_STREAM(ex.what());
        return;
    }
    obj.presence_vector |= cav_msgs::ExternalObject::ID_PRESENCE_VECTOR;
    obj.id = (msg->core_data.id[0] << 24) | (msg->core_data.id[1] << 16) | (msg->core_data.id[2] << 8) | (msg->core_data.id[3]);

    obj.presence_vector |= cav_msgs::ExternalObject::BSM_ID_PRESENCE_VECTOR;
    obj.bsm_id.resize(msg->core_data.id.size());
    std::copy(msg->core_data.id.begin(),msg->core_data.id.end(),obj.bsm_id.begin());

    wgs84_utils::wgs84_coordinate bsm_coord;
    bsm_coord.heading   = msg->core_data.heading;
    bsm_coord.elevation = msg->core_data.elev;
    bsm_coord.lat       = msg->core_data.latitude;
    bsm_coord.lon       = msg->core_data.longitude;

    wgs84_utils::wgs84_coordinate ref_wgs84;
    ref_wgs84.heading   = heading_map_.begin()->second->heading;
    ref_wgs84.elevation = navsatfix_map_.begin()->second->altitude;
    ref_wgs84.lat       = navsatfix_map_.begin()->second->latitude;
    ref_wgs84.lon       = navsatfix_map_.begin()->second->longitude;

    Eigen::Vector3d odom_pose;
    odom_pose[0] = odom_tf.transform.translation.x;
    odom_pose[1] = odom_tf.transform.translation.y;
    odom_pose[2] = odom_tf.transform.translation.z;

    Eigen::Quaterniond odom_rot;
    odom_rot.x() = odom_tf.transform.rotation.x;
    odom_rot.y() = odom_tf.transform.rotation.y;
    odom_rot.z() = odom_tf.transform.rotation.z;
    odom_rot.w() = odom_tf.transform.rotation.w;

    Eigen::Transform<double, 3, Eigen::Affine> ned_odom_tf_eig;
    ned_odom_tf_eig = Eigen::Translation3d(ned_odom_tf.transform.translation.x, ned_odom_tf.transform.translation.y,
                                           ned_odom_tf.transform.translation.z)
                      * Eigen::Quaterniond(ned_odom_tf.transform.rotation.x, ned_odom_tf.transform.rotation.y,
                                           ned_odom_tf.transform.rotation.z, ned_odom_tf.transform.rotation.w);

    Eigen::Quaterniond out_rot;
    Eigen::Vector3d out_pose;

    wgs84_utils::convertToOdom(bsm_coord, ref_wgs84, odom_pose, odom_rot, ned_odom_tf_eig, out_pose, out_rot);

    obj.presence_vector |= cav_msgs::ExternalObject::POSE_PRESENCE_VECTOR;

    obj.pose.pose.position.x = out_pose[0];
    obj.pose.pose.position.y = out_pose[1];
    obj.pose.pose.position.z = out_pose[2];

    obj.pose.pose.orientation.x = out_rot.x();
    obj.pose.pose.orientation.y = out_rot.y();
    obj.pose.pose.orientation.z = out_rot.z();
    obj.pose.pose.orientation.w = out_rot.w();

    if(msg->core_data.speed < cav_msgs::BSMCoreData::SPEED_UNAVAILABLE)
    {
    
      Eigen::Vector3d velocity_vector;
      velocity_vector[0] = msg->core_data.speed;
      velocity_vector[1] = 0.0;
      velocity_vector[2] = 0.0;
      
      velocity_vector = out_rot.inverse().toRotationMatrix()*velocity_vector;
      
      obj.presence_vector |= cav_msgs::ExternalObject::VELOCITY_PRESENCE_VECTOR;
      obj.velocity.twist.linear.x = velocity_vector[0];
      obj.velocity.twist.linear.y = velocity_vector[1];
      obj.velocity.twist.linear.z = velocity_vector[2];
    }

    obj.presence_vector |= cav_msgs::ExternalObject::CONFIDENCE_PRESENCE_VECTOR;
    obj.confidence = 1.0;

    obj.presence_vector |= cav_msgs::ExternalObject::SIZE_PRESENCE_VECTOR;
    obj.size.x = msg->core_data.size.vehicle_length;
    obj.size.y = msg->core_data.size.vehicle_width / 2;
    obj.size.z = 1.5;
    ROS_DEBUG_STREAM_NAMED("bsm_logger","Converted bsm message: " << obj);

    std::vector<torc::TrackedObject> objects;
    objects.push_back(torc::toTrackedObject(obj));
    torc::TrackedObject& back = objects.back();
    uint32_t serial_size = ros::serialization::serializationLength(obj);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size+sizeof(uint32_t)]);
    back.src_data[src_id] = buffer;
    memcpy(back.src_data[src_id].get(),&serial_size,sizeof(uint32_t));
    ros::serialization::OStream stream(back.src_data[src_id].get()+sizeof(uint32_t),serial_size);
    ros::serialization::serialize(stream,obj);
    tracker_->addObjects(objects.begin(),objects.end(),src_id,msg->header.stamp.toBoost());
}

void SensorFusionApplication::velocity_cb(const ros::MessageEvent<geometry_msgs::TwistStamped> &event) {
    std::string name = event.getPublisherName();
    velocity_map_[name] = event.getMessage();
    twist_history_buffer_.addTwist(event.getMessage());
}

void SensorFusionApplication::heading_cb(const ros::MessageEvent<cav_msgs::HeadingStamped> &event) {
    std::string name = event.getPublisherName();
    heading_map_[name] = event.getMessage();
}

void SensorFusionApplication::navsatfix_cb(const ros::MessageEvent<sensor_msgs::NavSatFix> &event) {
    std::string name = event.getPublisherName();
    navsatfix_map_[name] = event.getMessage();
}

void SensorFusionApplication::odom_cb(const ros::MessageEvent<nav_msgs::Odometry> &event) {
    std::string name = event.getPublisherName();
    odom_map_[name] = event.getMessage();
}

void SensorFusionApplication::dyn_recfg_cb(sensor_fusion::SensorFusionConfig &cfg, uint32_t level)
{
    static bool init = false;
    std::string msg = init ? "Reconfigure Request" : "Initial Config";
    ROS_INFO("%s: \n\ttracker_lifetime_decay: %f\n\ttracker_score_threshold: %f\n\ttracker_max_velocity: %f\n\ttracker_minimum_track_score: %f",
             msg.c_str(),
        cfg.tracker_lifetime_decay,
        cfg.tracker_score_threshold,
        cfg.tracker_max_velocity,
        cfg.tracker_minimum_track_score);

    config_ = cfg;

    tracker_->config.life_time_decay = config_.tracker_lifetime_decay;
    tracker_->config.max_velocity = config_.tracker_max_velocity;
    tracker_->config.score_threshold = config_.tracker_score_threshold;
    tracker_->config.minimum_tracker_score = config_.tracker_minimum_track_score;

    init = true;
}


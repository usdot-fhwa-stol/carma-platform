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
#include "transform_maintainer.h"
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

} // End tf2 namespace

namespace torc
{

class SimTimer : public cav::Timer
{
    virtual boost::posix_time::ptime getTime() override
    {
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
    tgt.x = ((tgt.x * merged_so_far) + src.x) / (merged_so_far + 1);
    tgt.y = ((tgt.y * merged_so_far) + src.y) / (merged_so_far + 1);
    tgt.z = ((tgt.z * merged_so_far) + src.z) / (merged_so_far + 1);
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
        if(it.second.get() == nullptr) 
        {
            continue;
        }

        cav_msgs::ExternalObject src_obj;
        uint32_t serial_size;
        memcpy(&serial_size, it.second.get(), sizeof(uint32_t));

        ros::serialization::IStream stream(it.second.get() + sizeof(uint32_t), serial_size);
        ros::serialization::Serializer<cav_msgs::ExternalObject>::read(stream, src_obj);

        if((src_obj.presence_vector & cav_msgs::ExternalObject::SIZE_PRESENCE_VECTOR) != 0)
        {
            out.presence_vector |= cav_msgs::ExternalObject::SIZE_PRESENCE_VECTOR;
            mergeSize(out.size, src_obj.size, sizes_merged++);
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
} // End torc namespace

int SensorFusionApplication::run()
{
    nh_.reset(new ros::NodeHandle());
    pnh_.reset(new ros::NodeHandle("~"));
    ros::NodeHandle pnh("filtered");
    tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));

    pnh_->param<std::string>("inertial_frame_name", inertial_frame_name_, "odom");
    pnh_->param<std::string>("body_frame_name", body_frame_name_, "base_link");
    pnh_->param<std::string>("ned_frame_name", ned_frame_name_, "ned");
    pnh_->param<std::string>("earth_frame_name", earth_frame_name_, "earth");
    pnh_->param<std::string>("global_pos_sensor_frame_name", global_pos_sensor_frame_name_, "pinpoint");
    pnh_->param<std::string>("local_pos_sensor_frame_name", local_pos_sensor_frame_name_, "pinpoint");
    pnh_->param<bool>("use_interface_mgr", use_interface_mgr_, false);

    // Setup transform maintainer
    tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster());
    tf_maintainer_.init(&tf2_buffer_, &(*tf2_broadcaster_),
     &odom_map_, &navsatfix_map_, &heading_map_,
     earth_frame_name_, ned_frame_name_, inertial_frame_name_, 
     body_frame_name_, global_pos_sensor_frame_name_, local_pos_sensor_frame_name_);

    // Use sim time if needed
    bool use_sim_time;
    nh_->param<bool>("/use_sim_time", use_sim_time, false);

	/* This use sim time fix was added to support rosbag playback. The issue is that the tracker is time based
	 * and if a measurement comes in with an earlier time we assume it is old. So to support looping play back 
	 * we need to reset the tracker and also use a special SimTimer to read time from the ROS timer rather than
	 * the default timer. */
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
    }
    else
    {
        tracker_.reset(new torc::ObjectTracker());
    }

    // Setup dyn_recfg_server
    {
        dynamic_reconfigure::Server<sensor_fusion::SensorFusionConfig>::CallbackType f;
        f = std::bind(&SensorFusionApplication::dyn_recfg_cb, this, std::placeholders::_1, std::placeholders::_2);
        dyn_cfg_server_->setCallback(f);
    }

    ros::Subscriber bsm_sub = nh_->subscribe<cav_msgs::BSM>("bsm", 50, &SensorFusionApplication::bsm_map_cb, this);
    
    if(use_interface_mgr_)
    {
        ROS_INFO_STREAM("Waiting for Interface Manager");
        ros::service::waitForService("get_drivers_with_capabilities");
        update_services_timer_ = nh_->createTimer(ros::Duration(5.0), [this](const ros::TimerEvent& ev){ update_subscribed_services(); }, false, true);
        ROS_INFO_STREAM("Interface Manager available");
    }
		else
    {
		    //This allows us to manually set the topics to listen, rather than querying the interface manager. Topics can be set through a xaml list in the launch file
				
        // Odometry
        {
            ROS_INFO_STREAM("Odometry Topics");
            XmlRpc::XmlRpcValue v;
            pnh_->param("odometry_topics", v, v);
            for(int i = 0; i < v.size(); i++)
            {
                ROS_INFO_STREAM("Subscribing to "<< v[i]);
                sub_map_[v[i]] = nh_->subscribe<nav_msgs::Odometry>(v[i], 1, [this](const ros::MessageEvent<nav_msgs::Odometry const>& msg){ odom_cb(msg); });
            }
        }

        // Velocity
        {
            ROS_INFO_STREAM("Velocity Topics");
            XmlRpc::XmlRpcValue v;
            pnh_->param("velocity_topics", v, v);
            for(int i = 0; i < v.size(); i++)
            {
                ROS_INFO_STREAM("Subscribing to "<< v[i]);
                sub_map_[v[i]] = nh_->subscribe<geometry_msgs::TwistStamped>(v[i], 1, [this](const ros::MessageEvent<geometry_msgs::TwistStamped>& msg){ velocity_cb(msg); });
            }
        }

        // NavSatFix
        {
            ROS_INFO_STREAM("NavSatFix Topics");
            XmlRpc::XmlRpcValue v;
            pnh_->param("navsatfix_topics", v, v);
            for(int i = 0; i < v.size(); i++)
            {
                ROS_INFO_STREAM("Subscribing to "<< v[i]);
                sub_map_[v[i]] = nh_->subscribe<sensor_msgs::NavSatFix>(v[i], 1, [this](const ros::MessageEvent<sensor_msgs::NavSatFix>& msg){ navsatfix_cb(msg); });
            }
        }

        // Heading
        {
            ROS_INFO_STREAM("Heading Topics");
            XmlRpc::XmlRpcValue v;
            pnh_->param("heading_topics", v, v);
            for(int i = 0; i < v.size(); i++)
            {
                ROS_INFO_STREAM("Subscribing to "<< v[i]);
                sub_map_[v[i]] = nh_->subscribe<cav_msgs::HeadingStamped>(v[i], 1, [this](const ros::MessageEvent<cav_msgs::HeadingStamped>& msg){ heading_cb(msg); });
            }
        }

        // Objects
        {
            ROS_INFO_STREAM("Objects Topics");
            XmlRpc::XmlRpcValue v;
            pnh_->param("objects_topics", v, v);
            for(int i = 0; i < v.size(); i++)
            {
                ROS_INFO_STREAM("Subscribing to "<< v[i]);
                sub_map_[v[i]] =  nh_->subscribe<cav_msgs::ExternalObjectList>(v[i], 1, [this, &v, i](const cav_msgs::ExternalObjectListConstPtr& msg){ objects_cb_q_.push_back(std::make_pair(v[i], msg)); });
            }
        }
    }

    odom_pub_       = pnh.advertise<nav_msgs::Odometry>("odometry", 100);
    navsatfix_pub_  = pnh.advertise<sensor_msgs::NavSatFix>("nav_sat_fix", 100);
    velocity_pub_   = pnh.advertise<geometry_msgs::TwistStamped>("velocity", 100);
    heading_pub_    = pnh.advertise<cav_msgs::HeadingStamped>("heading", 100);
    objects_pub_    = pnh.advertise<cav_msgs::ExternalObjectList>("tracked_objects", 100);

    ros::Rate r(20);
    while(ros::ok())
    {
        // Process callbacks
        ros::spinOnce(); // spinOnce tries to process all the elements in every subscriber queue at the moment it is called. 

        // Can we get them processed afterword just like objects

        // Updates transforms with new data.
        // If we got a new nav sat fix but not a new heading should we wait for the new heading?
        // TODO only update the transforms if new data is provided
        if (!odom_map_.empty())
            tf_maintainer_.odometry_update_cb(odom_map_.begin()->second); // Always update odometry first
        if (!navsatfix_map_.empty() && !heading_map_.empty())
            tf_maintainer_.nav_sat_fix_update_cb(navsatfix_map_.begin()->second, heading_map_.begin()->second);

        // After updating transforms we should process bsm objects
        ROS_INFO_STREAM("BSM Id map of size: " << bsm_id_map_.size());
        for (std::pair<std::string, cav_msgs::BSMConstPtr> el : bsm_id_map_)
        {
            process_bsm(el.second);
        }

        // Clear the bsm map. It will be rebuilt on the next call to spinOnce.
        bsm_id_map_.clear();


        // After updating transforms we should process the sensor objects
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

void SensorFusionApplication::update_subscribed_services() 
{
    ROS_INFO_STREAM("Updating subscribed services");
    
    // Odometry
    std::vector<std::string> ret = get_api("position/odometry");
    for(const std::string& it : ret)
    {
        if(sub_map_.find(it) == sub_map_.end()) 
        {
            sub_map_[it] = nh_->subscribe<nav_msgs::Odometry>(it, 1, [this](const ros::MessageEvent<nav_msgs::Odometry const>& msg){ odom_cb(msg); });
        }
    }

    // Nav_Sat_Fix
    ret = get_api("position/nav_sat_fix");
    for(const std::string& it : ret)
    {
        if(sub_map_.find(it) == sub_map_.end()) 
        {
            sub_map_[it] = nh_->subscribe<sensor_msgs::NavSatFix>(it, 1, [this](const ros::MessageEvent<sensor_msgs::NavSatFix const>&msg){ navsatfix_cb(msg); });
        }
    }

    // Heading
    ret = get_api("position/heading");
    for(const std::string& it : ret)
    {
        if(sub_map_.find(it) == sub_map_.end()) 
        {
            sub_map_[it] = nh_->subscribe<cav_msgs::HeadingStamped>(it, 1, [this](const ros::MessageEvent<cav_msgs::HeadingStamped>&  msg){ heading_cb(msg); });
        }
    }

    // Velocity
    ret = get_api("position/velocity");
    for(const std::string& it : ret)
    {
        if(sub_map_.find(it) == sub_map_.end())
        {
            sub_map_[it] = nh_->subscribe<geometry_msgs::TwistStamped>(it, 1, [this](const ros::MessageEvent<geometry_msgs::TwistStamped>& msg){ velocity_cb(msg); });
        }
    }

    // Tracked_objects
    ret = get_api("sensor/objects");
    for(const std::string& it : ret)
    {
        if(sub_map_.find(it) == sub_map_.end()) 
        {
            sub_map_[it] = nh_->subscribe<cav_msgs::ExternalObjectList>(it, 1, [this, it](const cav_msgs::ExternalObjectListConstPtr& msg){ objects_cb_q_.push_back(std::make_pair(it, msg)); });
        }
    }
}

std::vector<std::string> SensorFusionApplication::get_api(const std::string &name)
{

    ros::ServiceClient client = nh_->serviceClient<cav_srvs::GetDriversWithCapabilities>("get_drivers_with_capabilities");
    cav_srvs::GetDriversWithCapabilities srv;
    srv.request.capabilities.push_back(name);

    ROS_INFO_STREAM("Sending request to get_drivers_with_capabilities: " << srv.request);
    std::vector<std::string> ret;

    if(client.exists() && client.call(srv))
    {
        ROS_INFO_STREAM("get_drivers_with_capabilities returned: " << srv.response);
        
        // The service returns a list of drivers that have the api we provided
        for(std::string fqn : srv.response.driver_data)
        {
            size_t pos = fqn.find(name);
            std::string driverName = fqn.substr(0, pos);

            // If we haven't subscribed to the topic formed by the name of the node and the service add this topic to the return list
            if(sub_map_.find(fqn) == sub_map_.end()) 
            {
                ret.push_back(fqn);
            }
        }
    }
    else
    {
        ROS_WARN_STREAM_THROTTLE(2, "Unable to query service get_drivers_with_capabilites");
    }

    return ret;
}

void SensorFusionApplication::publish_updates() 
{
    if(!odom_map_.empty())
    {
        odom_pub_.publish(odom_map_.begin()->second);
    }

    if(!navsatfix_map_.empty())
    {
        navsatfix_pub_.publish(navsatfix_map_.begin()->second);
    }

    if(!heading_map_.empty())
    {
        heading_pub_.publish(heading_map_.begin()->second);
    }

    if(!velocity_map_.empty())
    {
        velocity_pub_.publish(velocity_map_.begin()->second);
    }

    tracker_->process();

    cav_msgs::ExternalObjectList list;
    list.header.stamp = ros::Time::fromBoost(tracker_->tracked_sensor_->time_stamp);
    list.header.frame_id = inertial_frame_name_;

    // Check if Tracked Objects list is empty
    if(!tracker_->tracked_sensor_->objects.empty()) 
    {
        // Loop through all tracked objects and publish
        for (auto& it : tracker_->tracked_sensor_->objects)
        {
            cav_msgs::ExternalObject obj = toExternalObject(it);
            obj.header = list.header;
            list.objects.push_back(obj);
        }

        static uint64_t objects_published = 0;
        objects_published += list.objects.size();
        ROS_DEBUG_STREAM("(Object Tracker) Total: " << objects_published << ", This loop: " << list.objects.size());
        objects_pub_.publish(list);
    }
    else 
    {
        objects_pub_.publish(list); // Publishing an empty list when there are no tracked objects
        ROS_DEBUG_STREAM("(Object Tracker) No tracked_objects");
    }
}

void SensorFusionApplication::objects_cb(const cav_msgs::ExternalObjectListConstPtr& msg, const std::string& topic_name) 
{
    std::hash<std::string> hash_fn;
    size_t hash = hash_fn(topic_name);

    // Variables and calls to track transform errors
    std::string transform_error_1 = "";
    bool first_transform_check = tf2_buffer_.canTransform(inertial_frame_name_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.0), &transform_error_1);
    std::string transform_error_2 = "";
    bool second_transform_check = tf2_buffer_.canTransform(inertial_frame_name_, msg->header.frame_id, ros::Time(0), ros::Duration(0.0), &transform_error_2);
    // Get Transform from object measurement to inertial frame. All tracking should be done in inertial frame
    geometry_msgs::TransformStamped transformStamped, sensor_in_veh_tf;

    // Get Transform from sensor frame to vehicle frame. This should be a static transform so if this lookup fails log an error
    try {
        sensor_in_veh_tf = tf2_buffer_.lookupTransform(body_frame_name_, msg->header.frame_id, msg->header.stamp);

    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return;
    }

    if(first_transform_check)
    {
        transformStamped = tf2_buffer_.lookupTransform(inertial_frame_name_, msg->header.frame_id, msg->header.stamp);
    }
    else if(second_transform_check)
    {
        //ROS_INFO_STREAM("Using latest transform available");
        transformStamped = tf2_buffer_.lookupTransform(inertial_frame_name_, msg->header.frame_id, ros::Time(0));
    }
    else
    {
        //ROS_WARN_STREAM("No transform available from " << inertial_frame_name_ << " to " << msg->header.frame_id);
        return;
    }

    //ROS_INFO_STREAM("Transform 1: " << first_transform_check << " : " << transform_error_1);
    //ROS_INFO_STREAM("Transform 2: " << second_transform_check << " : " << transform_error_2);

    std::vector<torc::TrackedObject> transformed_list;
    for(auto & it : msg->objects)
    {
        // Transform to inertial frame
        cav_msgs::ExternalObject obj;
        tf2::doTransform(it, obj, transformStamped);

        geometry_msgs::TwistStamped twistStamped;
        try
        {
            if(!twist_history_buffer_.getTwist(body_frame_name_,obj.header.stamp,twistStamped))
            {
                continue;
            }
        }
        catch(cav::ExtrapolationException e)
        {
            //ROS_WARN_STREAM(e.what());
            twist_history_buffer_.getLatest(body_frame_name_, twistStamped);
        }

        if(obj.presence_vector & cav_msgs::ExternalObject::VELOCITY_PRESENCE_VECTOR)
        {
            tf2::Vector3 obj_v_unrotated;
            tf2::convert(obj.velocity.twist.linear, obj_v_unrotated);

            tf2::Quaternion sensor_in_veh_rot;
            tf2::convert(sensor_in_veh_tf.transform.rotation, sensor_in_veh_rot);

            tf2::Vector3 obj_v = tf2::quatRotate(sensor_in_veh_rot, obj_v_unrotated);

            tf2::Vector3 body_v;
            tf2::convert(twistStamped.twist.linear, body_v);

            tf2::Quaternion rotation;
            tf2::convert(transformStamped.transform.rotation, rotation);

            obj_v += body_v;

            tf2::Vector3 obj_v_rot = tf2::quatRotate(rotation, obj_v);
            tf2::convert(obj_v_rot, obj.velocity.twist.linear);
        }

        transformed_list.push_back(torc::toTrackedObject(obj)); // Convert Object to Tracked Obj and Put in Tansformed List

        torc::TrackedObject& back = transformed_list.back();    

        uint32_t serial_size = ros::serialization::serializationLength(obj);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size + sizeof(uint32_t)]);
        back.src_data[hash] = buffer;

        memcpy(back.src_data[hash].get(), &serial_size, sizeof(uint32_t));
        ros::serialization::OStream stream(back.src_data[hash].get() + sizeof(uint32_t), serial_size);

        ros::serialization::serialize(stream, obj);
    }

    // Add all Message Objects to Object Tracker as measures objects
    tracker_->addObjects(transformed_list.begin(), transformed_list.end(), hash, transformStamped.header.stamp.toBoost());
}

inline std::string bsmIdFromBuffer(std::vector<uint8_t> id_buffer) {

    if (id_buffer.size() != 4) {
        // TODO throw exception
        return "";
    }

    std::stringstream stream;
    for (int i = 0; i < id_buffer.size(); i++) {
        stream << std::setfill ('0') << std::setw(2) << std::hex << id_buffer[i];
    }

    return stream.str();

}

void SensorFusionApplication::bsm_map_cb(const cav_msgs::BSMConstPtr &msg) {

    ROS_INFO_NAMED("bsm_logger","PROCESSING BSM");
    std::string bsm_id = bsmIdFromBuffer(msg->core_data.id);
    ROS_INFO_STREAM_NAMED("bsm_logger","BSM_ID: " << bsm_id);
    bsm_id_map_[bsm_id] = msg;
}

// TODO investigate BSM fusion further. There may be extra bugs introduced by merge
void SensorFusionApplication::process_bsm(const cav_msgs::BSMConstPtr &msg) {
    ROS_DEBUG_STREAM_NAMED("bsm_logger","Received bsm message: " << msg);
    if(heading_map_.empty() || navsatfix_map_.empty())
    {
        ROS_DEBUG_STREAM_NAMED("bsm_logger", "Received bsm before heading and navsatfix updated unable to process msg");
        return;
    }
    auto hash = std::hash<std::string>();
    size_t src_id = hash("bsm_objects");

    tf2::Stamped<tf2::Transform> ecef_in_ned_tf, odom_in_map_tf;
    try {

        odom_in_map_tf = tf_maintainer_.get_transform(ned_frame_name_, inertial_frame_name_, msg->header.stamp, true);
        ecef_in_ned_tf = tf_maintainer_.get_transform(ned_frame_name_, "earth", msg->header.stamp, true);

    } catch (tf2::TransformException&ex) {
        ROS_WARN_STREAM_NAMED("bsm_logger", "Sensor fusion transform lookup exception: " << ex.what());
        return;
    }

    cav_msgs::ExternalObject obj;
    obj.header.frame_id = inertial_frame_name_;
    obj.header.stamp = odom_in_map_tf.stamp_; // Use the timestamp of the transform for this bsm to ensure it can be converted
    obj.presence_vector = 0;


    obj.presence_vector |= cav_msgs::ExternalObject::ID_PRESENCE_VECTOR;
    obj.id = (msg->core_data.id[0] << 24) | (msg->core_data.id[1] << 16) | (msg->core_data.id[2] << 8) | (msg->core_data.id[3]);

    obj.presence_vector |= cav_msgs::ExternalObject::BSM_ID_PRESENCE_VECTOR;
    obj.bsm_id.resize(msg->core_data.id.size());
    std::copy(msg->core_data.id.begin(), msg->core_data.id.end(), obj.bsm_id.begin());

    wgs84_utils::wgs84_coordinate bsm_coord_rad;
    bsm_coord_rad.heading   = msg->core_data.heading * wgs84_utils::DEG2RAD;
    bsm_coord_rad.elevation = msg->core_data.elev;
    bsm_coord_rad.lat       = msg->core_data.latitude * wgs84_utils::DEG2RAD;
    bsm_coord_rad.lon       = msg->core_data.longitude * wgs84_utils::DEG2RAD;

   
    // All tf2 multiplication works as expected with matrix on right applied to matrix on left to do multiplication
    ROS_DEBUG_STREAM_NAMED("bsm_logger","bsm_coord_rad (lat,lon,elev,heading): (" << bsm_coord_rad.lat << ", " << bsm_coord_rad.lon << ", " << bsm_coord_rad.elevation << ", " << bsm_coord_rad.heading << ")");
    // Get bsm position in nef frame
    tf2::Vector3 bsm_in_map_trans = wgs84_utils::geodesic_2_cartesian(bsm_coord_rad, ecef_in_ned_tf);
    // Apply heading as orientation
    const tf2::Vector3 z_axis(0,0,1);
    tf2::Quaternion rot_in_ned(z_axis, bsm_coord_rad.heading);
    // BSM transform in map
    tf2::Transform bsm_in_map_tf(rot_in_ned, bsm_in_map_trans);
    

    tf2::Transform bsm_in_odom = odom_in_map_tf.inverse() * bsm_in_map_tf;
    tf2::Vector3 bsm_trans = bsm_in_odom.getOrigin();
    tf2::Quaternion bsm_rot = bsm_in_odom.getRotation();
    
    
    Eigen::Vector3d out_pose(bsm_trans.getX(), bsm_trans.getY(), bsm_trans.getZ());
    
    Eigen::Quaterniond out_rot(bsm_rot.getW(), bsm_rot.getX(), bsm_rot.getY(), bsm_rot.getZ()); // Quaturniond is a quaturnion with double values

    obj.presence_vector |= cav_msgs::ExternalObject::POSE_PRESENCE_VECTOR;

    obj.pose.pose.position.x = out_pose[0];
    obj.pose.pose.position.y = out_pose[1];
    obj.pose.pose.position.z = out_pose[2];

    obj.pose.pose.orientation.x = out_rot.x();
    obj.pose.pose.orientation.y = out_rot.y();
    obj.pose.pose.orientation.z = out_rot.z();
    obj.pose.pose.orientation.w = out_rot.w();

    if(msg->core_data.presence_vector & cav_msgs::BSMCoreData::SPEED_AVAILABLE)
    {
      Eigen::Vector3d velocity_vector;
      velocity_vector[0] = msg->core_data.speed;
      velocity_vector[1] = 0.0;
      velocity_vector[2] = 0.0;
      
      velocity_vector = out_rot.toRotationMatrix() * velocity_vector;
      
      obj.presence_vector |= cav_msgs::ExternalObject::VELOCITY_PRESENCE_VECTOR;
      obj.velocity.twist.linear.x = velocity_vector[0];
      obj.velocity.twist.linear.y = velocity_vector[1];
      obj.velocity.twist.linear.z = velocity_vector[2];
    }

    obj.presence_vector |= cav_msgs::ExternalObject::CONFIDENCE_PRESENCE_VECTOR;
    obj.confidence = 1.0;

    obj.presence_vector |= cav_msgs::ExternalObject::SIZE_PRESENCE_VECTOR;
    // TODO double check these size values
    obj.size.x = msg->core_data.size.vehicle_length / 2;
    obj.size.y = msg->core_data.size.vehicle_width / 2;
    obj.size.z = 1.5; // TODO probably worth picking a picking a big z value
    ROS_DEBUG_STREAM_NAMED("bsm_logger", "Converted bsm message: " << obj);

    //TODO uncomment this is where fusion occurs
    std::vector<torc::TrackedObject> objects;
    objects.push_back(torc::toTrackedObject(obj));
    torc::TrackedObject& back = objects.back();
    uint32_t serial_size = ros::serialization::serializationLength(obj);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size + sizeof(uint32_t)]);
    back.src_data[src_id] = buffer;
    memcpy(back.src_data[src_id].get(),&serial_size,sizeof(uint32_t));
    ros::serialization::OStream stream(back.src_data[src_id].get() + sizeof(uint32_t), serial_size);
    ros::serialization::serialize(stream, obj);
    tracker_->addObjects(objects.begin(), objects.end(),src_id, obj.header.stamp.toBoost());
    //bsm_obj_ = obj; //TODO used to bypass fusion
}

void SensorFusionApplication::velocity_cb(const ros::MessageEvent<geometry_msgs::TwistStamped> &event) 
{
    std::string name = event.getPublisherName();
    velocity_map_[name] = event.getMessage();
    twist_history_buffer_.addTwist(event.getMessage());
}

void SensorFusionApplication::heading_cb(const ros::MessageEvent<cav_msgs::HeadingStamped> &event) 
{    
    std::string name = event.getPublisherName();
    heading_map_[name] = event.getMessage();
}

void SensorFusionApplication::navsatfix_cb(const ros::MessageEvent<sensor_msgs::NavSatFix> &event) 
{    
    std::string name = event.getPublisherName();
    navsatfix_map_[name] = event.getMessage();
}

void SensorFusionApplication::odom_cb(const ros::MessageEvent<nav_msgs::Odometry> &event) 
{    
    std::string name = event.getPublisherName();
    odom_map_[name] = event.getMessage();
}

void SensorFusionApplication::dyn_recfg_cb(sensor_fusion::SensorFusionConfig &cfg, uint32_t level)
{
    static bool init = false;
    std::string msg = init ? "Reconfigure Request" : "Initial Config";
    ROS_INFO("%s: \t\ntracker_life_time_decay: %f\t\ntracker_life_time_divider: %f\t\ntracker_life_time_threshold: %f\t\ntracker_out_of_range_dist: %f\t\ntracker_unassociated_group_dist: %f\t\ntracker_score_threshold: %f",
        msg.c_str(),
        cfg.tracker_life_time_decay,
        cfg.tracker_life_time_divider,
        cfg.tracker_life_time_threshold,
        cfg.tracker_out_of_range_dist,
        cfg.tracker_unassociated_group_dist,
        cfg.tracker_score_threshold);

    config_ = cfg;

   tracker_->config.tracker_life_time_decay = config_.tracker_life_time_decay;
   tracker_->config.tracker_life_time_divider = config_.tracker_life_time_divider;
   tracker_->config.tracker_life_time_threshold = config_.tracker_life_time_threshold;
   tracker_->config.tracker_out_of_range_dist = config_.tracker_out_of_range_dist;
   tracker_->config.tracker_unassociated_group_dist = config_.tracker_unassociated_group_dist;
   tracker_->config.tracker_score_threshold = config_.tracker_score_threshold;

    init = true;
}
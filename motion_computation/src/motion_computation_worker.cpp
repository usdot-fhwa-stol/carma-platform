/*
 * Copyright (C) 2019-2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
#include "motion_computation/motion_computation_worker.hpp"

namespace motion_computation{

    MotionComputationWorker::MotionComputationWorker(const PublishObjectCallback& obj_pub, rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger) 
        : obj_pub_(obj_pub), logger_(logger) {};

    void MotionComputationWorker::predictionLogic(carma_perception_msgs::msg::ExternalObjectList::UniquePtr obj_list)
    {
        carma_perception_msgs::msg::ExternalObjectList output_list;
        carma_perception_msgs::msg::ExternalObjectList sensor_list;

        for (auto obj : obj_list->objects)
        {
            // Header contains the frame rest of the fields will use
            // obj.header = obj_list.objects[i].header;

            // Object id. Matching ids on a topic should refer to the same object within some time period, expanded
            // obj.id = obj_list.objects[i].id;

            // Update the object type and generate predictions using CV or CTRV vehicle models.
            // If the object is a bicycle or motor vehicle use CTRV otherwise use CV.

            bool use_ctrv_model;

            if (  obj.object_type == obj.UNKNOWN)
            {
                use_ctrv_model = true;
            }
            else if (obj.object_type == obj.MOTORCYCLE)
            {
                use_ctrv_model = true;
            }
            else if (obj.object_type == obj.SMALL_VEHICLE)
            {
                use_ctrv_model = true;
            }
            else if (obj.object_type == obj.LARGE_VEHICLE)
            {
                use_ctrv_model = true;
            }
            else if ( obj.object_type == obj.PEDESTRIAN)
            {
                use_ctrv_model = false;
            }
            else
            {
                obj.object_type = obj.UNKNOWN;
                use_ctrv_model = false;
            }//end if-else

            
            if (use_ctrv_model == true)
            {
                obj.predictions =
                    motion_predict::ctrv::predictPeriod(obj, prediction_time_step_, prediction_period_,
                                                        prediction_process_noise_max_, prediction_confidence_drop_rate_);
            }
            else
            {
                obj.predictions = motion_predict::cv::predictPeriod(
                    obj, prediction_time_step_, prediction_period_, cv_x_accel_noise_, cv_y_accel_noise_,
                    prediction_process_noise_max_, prediction_confidence_drop_rate_);
            }
            sensor_list.objects.emplace_back(obj);
        }//end for-loop


        // Synchronization priority
        // Sensor
        // BSM
        // PSM
        // MobilityPath
        // TODO add descriptive comments here
        const carma_perception_msgs::msg::ExternalObjectList& synchronization_base_objects;
        if (enable_sensor_processing) {

            synchronization_base_objects = sensor_list;

        } else if (enable_bsm_processing) {

            synchronization_base_objects = bsm_list;

        } else if (enable_psm_processing) {

            synchronization_base_objects = psm_list;

        } else if (enable_mobility_path_processing) {

            synchronization_base_objects = mobility_path_list_;

        } else {
            
            RCLCPP_WARN_STREAM(logger_->get_logger(), "Not configured to publish any data publishing empty object list. Operating like this is NOT advised.");
            // synchronization_base_objects.header.stamp = now();// TODO get access to clock
            obj_pub_(synchronization_base_objects);
            // TODO clear queues?
            return;
        }

        carma_perception_msgs::msg::ExternalObjectList current_output; // TODO we need to set the header for this
        current_output.objects.reserve(synchronization_base_objects.objects.size());

        if (enable_sensor_processing) {

            current_output = synchronizeAndAppend(sensor_list, current_output);

        }
        
        if (enable_bsm_processing) {

            current_output = synchronizeAndAppend(bsm_list, current_output);

        } 
        
        if (enable_psm_processing) {

            current_output = synchronizeAndAppend(psm_list, current_output);

        } 
        
        if (enable_mobility_path_processing) {
            
            current_output = synchronizeAndAppend(mobility_path_list_, current_output);

        }

        obj_pub_(output_list);

        // Clear mobility msg path queue since it is published
        mobility_path_list_.objects = {};
    }

    void MotionComputationWorker::georeferenceCallback(const std_msgs::msg::String::UniquePtr msg) 
    {
        // Build projector from proj string
        map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str()); 

        
        std::string axis = wgs84_utils::proj_tools::getAxisFromProjString(msg->data);  // Extract axis for orientation calc

        ROS_INFO_STREAM("Extracted Axis: " << axis);

        ned_in_map_rotation_ = wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis(axis);  // Extract map rotation from axis

        ROS_DEBUG_STREAM("Extracted NED in Map Rotation (x,y,z,w) : ( "
                        << ned_in_map_rotation_.get().x() << ", " << ned_in_map_rotation_.get().y() << ", "
                        << ned_in_map_rotation_.get().z() << ", " << ned_in_map_rotation_.get().w());
    }
    
    void MotionComputationWorker::setPredictionTimeStep(double time_step)
    {
        prediction_time_step_ = time_step;
    }

    void MotionComputationWorker::setMobilityPathPredictionTimeStep(double time_step)
    {
        mobility_path_prediction_time_step_ = time_step;
    }

    void MotionComputationWorker::setPredictionPeriod(double period)
    {
        prediction_period_ = period;
    }

    void MotionComputationWorker::setXAccelerationNoise(double noise)
    {
        cv_x_accel_noise_ = noise;
    }

    void MotionComputationWorker::setYAccelerationNoise(double noise)
    {
        cv_y_accel_noise_ = noise;
    }

    void MotionComputationWorker::setProcessNoiseMax(double noise_max)
    {
        prediction_process_noise_max_ = noise_max;
    }

    void MotionComputationWorker::setConfidenceDropRate(double drop_rate)
    {
        prediction_confidence_drop_rate_ = drop_rate;
    }

    void MotionComputationWorker::setExternalObjectPredictionMode(int external_object_prediction_mode)
    {
        external_object_prediction_mode_ = static_cast<MotionComputationMode>(external_object_prediction_mode);
    }

    void MotionComputationWorker::mobilityPathCallback(const carma_v2x_msgs::msg::MobilityPath::UniquePtr msg)
    {
        if (!map_projector_) {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Map projection not available yet so ignoring MobilityPath messages");
            return;
        }

        if (!enable_mobility_path_processing) {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "enable_mobility_path_processing is false so ignoring MobilityPath messages");
            return;
        }

        mobility_path_list_.objects.push_back(mobilityPathToExternalObject(msg));
    }

    void MotionComputationWorker::psmCallback(const carma_v2x_msgs::msg::PSM::UniquePtr msg)
    {
        if (!map_projector_) {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Map projection not available yet so ignoring PSM messages");
            return;
        }

        if (!enable_psm_processing) {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "enable_psm_processing is false so ignoring PSM messages");
            return;
        }

        psm_list_.objects.push_back(mobilityPathToExternalObject(msg));
    }

    void MotionComputationWorker::bsmCallback(const carma_v2x_msgs::msg::BSM::UniquePtr msg)
    {
        if (!map_projector_) {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Map projection not available yet so ignoring PSM messages");
            return;
        }

        if (!enable_bsm_processing) {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "enable_bsm_processing is false so ignoring BSM messages");
            return;
        }

        bsm_list_.objects.push_back(mobilityPathToExternalObject(msg));
    }

    carma_perception_msgs::msg::ExternalObject MotionComputationWorker::mobilityPathToExternalObject(const carma_v2x_msgs::msg::MobilityPath::UniquePtr& msg) const
    {
        carma_perception_msgs::msg::ExternalObject output;

        output.size.x = 2.5; // TODO identify better approach for object side in mobility path
        output.size.y = 2.25;
        output.size.z = 2.0;

        // get reference origin in ECEF (convert from cm to m)
        double ecef_x = (double)msg->trajectory.location.ecef_x/100.0;
        double ecef_y = (double)msg->trajectory.location.ecef_y/100.0;
        double ecef_z = (double)msg->trajectory.location.ecef_z/100.0;

        // Convert general information
        output.presence_vector |= carma_perception_msgs::msg::ExternalObject::ID_PRESENCE_VECTOR; 
        output.presence_vector |= carma_perception_msgs::msg::ExternalObject::POSE_PRESENCE_VECTOR;
        output.presence_vector |= carma_perception_msgs::msg::ExternalObject::VELOCITY_PRESENCE_VECTOR;
        output.presence_vector |= carma_perception_msgs::msg::ExternalObject::OBJECT_TYPE_PRESENCE_VECTOR;
        output.presence_vector |= carma_perception_msgs::msg::ExternalObject::BSM_ID_PRESENCE_VECTOR;
        output.presence_vector |= carma_perception_msgs::msg::ExternalObject::DYNAMIC_OBJ_PRESENCE;
        output.presence_vector |= carma_perception_msgs::msg::ExternalObject::PREDICTION_PRESENCE_VECTOR;
        output.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;
        std::hash<std::string> hasher;
        auto hashed = hasher(msg->m_header.sender_id); //TODO hasher returns size_t, message accept uint32_t which we might lose info
        output.id = (uint32_t)hashed;
        
        // convert hex std::string to uint8_t array
        for (size_t i = 0; i < msg->m_header.sender_bsm_id.size(); i+=2) 
        {
            int num = 0;
            sscanf(msg->m_header.sender_bsm_id.substr(i, i + 2).c_str(), "%x", &num);
            output.bsm_id.push_back((uint8_t)num);
        }

        // first point's timestamp
        output.header.stamp = rclcpp::Time((uint64_t)msg->m_header.timestamp *1e6); // ms to nanoseconds

        // If it is a static object, we finished processing
        if (msg->trajectory.offsets.size() < 2)
        {
            output.dynamic_obj = false;
            return output;
        }
        output.dynamic_obj = true;

        // get planned trajectory points
        carma_perception_msgs::msg::PredictedState prev_state;
        tf2::Vector3 prev_pt_ecef {ecef_x, ecef_y, ecef_z};

        auto prev_pt_map = transform_to_map_frame(prev_pt_ecef);
        double prev_yaw = 0.0;

        double message_offset_x = 0.0; // units cm
        double message_offset_y = 0.0;
        double message_offset_z = 0.0;

        for (size_t i = 0; i < msg->trajectory.offsets.size(); i ++)
        {
            auto curr_pt_msg = msg->trajectory.offsets[i];

            message_offset_x = (double)curr_pt_msg.offset_x + message_offset_x;
            message_offset_y = (double)curr_pt_msg.offset_y + message_offset_y;
            message_offset_z = (double)curr_pt_msg.offset_z + message_offset_z;

            tf2::Vector3 curr_pt_ecef {ecef_x + message_offset_x /100.0, ecef_y + message_offset_y /100.0, ecef_z + message_offset_z /100.0}; // ecef_x is in m while message_offset_x is in cm. Want m as final result
            auto curr_pt_map = transform_to_map_frame(curr_pt_ecef);

            carma_perception_msgs::msg::PredictedState curr_state;
            
            if (i == 0) // First point's state should be stored outside "predictions"
            {
                auto res = composePredictedState(curr_pt_map, prev_pt_map, output.header.stamp, prev_yaw); // Position returned is that of prev_pt_map NOT curr_pt_map 
                curr_state = std::get<0>(res);
                prev_yaw = std::get<1>(res);
                // Compute output pose
                output.pose.pose = curr_state.predicted_position; // Orientation computed from first point in offsets with location
                output.velocity.twist = curr_state.predicted_velocity; // Velocity derived from first point
            }
            else    
            {
                rclcpp::Time updated_time_step = rclcpp::Time(prev_state.header.stamp) + rclcpp::Duration(mobility_path_prediction_time_step_ * 1e9);
                auto res = composePredictedState(curr_pt_map, prev_pt_map, updated_time_step, prev_yaw);
                curr_state = std::get<0>(res);
                prev_yaw = std::get<1>(res);
                output.predictions.push_back(curr_state);
            }

            if (i == msg->trajectory.offsets.size() - 1) // if last point, copy the prev_state velocity & orientation to the last point too
            {
                curr_state.predicted_position.position.x = curr_pt_map.x();
                curr_state.predicted_position.position.y = curr_pt_map.y();
                curr_state.predicted_position.position.z = curr_pt_map.z();
                output.predictions.push_back(curr_state);
            }
            
            prev_state = curr_state;
            prev_pt_map = curr_pt_map;
        }

        calculateAngVelocityOfPredictedStates(output);        

        return output;
    }

    carma_perception_msgs::msg::ExternalObject MotionComputationWorker::psmToExternalObject(const carma_v2x_msgs::msg::PSM::UniquePtr& msg) const
    {
        carma_perception_msgs::msg::ExternalObject output;
        
        output.dynamic_obj = true; // If a PSM is sent then the object is dynamic since its a living thing

        // Generate a unique object id from the psm id
        output.id = 0
        for (int i = msg->id.id.size() - 1; i >= 0; i--) { // using signed iterator to handle empty case
            output.id |= msg->id.id[i] << (8*i);
        }

        // Additionally, store the id in the bsm_id field
        output.bsm_id = msg->id.id;

        // Compute the pose
        output.pose = pose_from_gnss(
            map_projector_, 
            ned_in_map_rotation_,
            { msg->position.latitude, msg->position.longitude, msg->position.elevation }, 
            msg->heading.heading
        );

        // Compute the position covariance
        // TODO

        msg->sec_mark.millisecond

        // Compute the timestamp
        // TODO

        if (msg->sec_mark.millisecond)
        
        // HERE TRYING TO CONVERT TIMESTAMP CORRECTLY
        auto current_time = get_clock()->now();
        auto inception_boost( boost::posix_time::time_from_string("1970-01-01 00:00:00.000") ); // inception of epoch
        auto duration_since_inception( lanelet::time::durationFromSec(current_time.seconds()) );
        auto curr_time_boost = inception_boost + duration_since_inception;
        boost::posix_time::ptime start_of_day(curr_time_boost.date().day());
        boost::posix_time::ptime

        
        boost::posix_time::ptime s_in_minute = lanelet::time::timeFromSec(msg->sec_mark.millisecond * 1000.0);
        auto received_state = static_cast<lanelet::CarmaTrafficSignalState>(current_movement_state.movement_event_list[0].event_state.movement_phase_state);

        if (curr_intersection.moy_exists) //account for minute of the year
        {
          auto inception_boost(boost::posix_time::time_from_string("1970-01-01 00:00:00.000")); // inception of epoch
          auto duration_since_inception(lanelet::time::durationFromSec(ros::Time::now().toSec()));
          auto curr_time_boost = inception_boost + duration_since_inception;
          ROS_DEBUG_STREAM("Calculated current time: " << boost::posix_time::to_simple_string(curr_time_boost));

          int curr_year = curr_time_boost.date().year();
          auto curr_year_start_boost(boost::posix_time::time_from_string(std::to_string(curr_year)+ "-01-01 00:00:00.000"));

          ROS_DEBUG_STREAM("MOY extracted: " << (int)curr_intersection.moy);
          auto curr_minute_stamp_boost = curr_year_start_boost + boost::posix_time::minutes((int)curr_intersection.moy);

          int hours_of_day = curr_minute_stamp_boost.time_of_day().hours();
          int curr_month = curr_minute_stamp_boost.date().month();
          int curr_day = curr_minute_stamp_boost.date().day();

          auto curr_day_boost(boost::posix_time::time_from_string(std::to_string(curr_year) + "/" + std::to_string(curr_month) + "/" + std::to_string(curr_day) +" 00:00:00.000")); // GMT is the standard
          auto curr_hour_boost = curr_day_boost + boost::posix_time::hours(hours_of_day);

          min_end_time += lanelet::time::durationFromSec(lanelet::time::toSec(curr_hour_boost));
          ROS_DEBUG_STREAM("New min_end_time: " << std::to_string(lanelet::time::toSec(min_end_time)));
        }

        // Set the type
        // TODO

        // Set the velocity
        // TODO

        // Compute the velocity covariance TODO

        // Set the size
        // TODO

        // Set the confidence
        // TODO

        // Compute predictions 
        // TODO



        

        return output;
    }

    carma_perception_msgs::msg::ExternalObject MotionComputationWorker::bsmToExternalObject(const carma_v2x_msgs::msg::BSM::UniquePtr& msg) const
    {
        carma_perception_msgs::msg::ExternalObject output;

        // Generate a unique object id from the bsm id
        output.id = 0
        for (int i = msg->core_data.id.size() - 1; i >= 0; i--) { // using signed iterator to handle empty case
            output.id |= msg->core_data.id[i] << (8*i);
        }

        output.bsm_id = msg->core_data.id;

        return output;
    }
    

    double getYawFromQuaternionMsg(const geometry_msgs::msg::Quaternion& quaternion)
    {
        tf2::Quaternion orientation;
        orientation.setX(quaternion.x);
        orientation.setY(quaternion.y);
        orientation.setZ(quaternion.z);
        orientation.setW(quaternion.w);

        double roll;
        double pitch;
        double yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        return yaw;
    }

    void MotionComputationWorker::calculateAngVelocityOfPredictedStates(carma_perception_msgs::msg::ExternalObject& object) const
    {
        if (!object.dynamic_obj)
        {
            return;
        }

        // Object's current angular velocity
        object.velocity.twist.angular.z = (getYawFromQuaternionMsg(object.pose.pose.orientation) - 
                                            getYawFromQuaternionMsg(object.predictions[0].predicted_position.orientation)) / mobility_path_prediction_time_step_;
        
        // Predictions' angular velocities
        auto prev_orient = object.pose.pose.orientation;
        for (auto& pred: object.predictions)
        {
            pred.predicted_velocity.angular.z = (getYawFromQuaternionMsg(prev_orient) - 
                                            getYawFromQuaternionMsg(pred.predicted_position.orientation)) / mobility_path_prediction_time_step_;
        }
    }

    std::pair<carma_perception_msgs::msg::PredictedState, double> MotionComputationWorker::composePredictedState(const tf2::Vector3& curr_pt, const tf2::Vector3& prev_pt, const rclcpp::Time& prev_time_stamp, double prev_yaw) const
    {
        carma_perception_msgs::msg::PredictedState output_state;
        // Set Position
        output_state.predicted_position.position.x = prev_pt.x();
        output_state.predicted_position.position.y = prev_pt.y();
        output_state.predicted_position.position.z = prev_pt.z();

        // Set Orientation
        Eigen::Vector2d vehicle_vector = {curr_pt.x() - prev_pt.x() ,curr_pt.y() - prev_pt.y()};
        Eigen::Vector2d x_axis = {1, 0};
        double yaw = 0.0;
        if (vehicle_vector.norm() < 0.000001) { // If there is zero magnitude use previous yaw to avoid devide by 0
            yaw = prev_yaw;
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Two identical points sent for predicting heading. Forced to use previous yaw or 0 if first point");
        } else {
            yaw = std::acos(vehicle_vector.dot(x_axis)/(vehicle_vector.norm() * x_axis.norm()));
        }
    
        tf2::Quaternion vehicle_orientation;
        vehicle_orientation.setRPY(0, 0, yaw);
        output_state.predicted_position.orientation.x = vehicle_orientation.getX();
        output_state.predicted_position.orientation.y = vehicle_orientation.getY();
        output_state.predicted_position.orientation.z = vehicle_orientation.getZ();
        output_state.predicted_position.orientation.w = vehicle_orientation.getW();

        // Set velocity
        output_state.predicted_velocity.linear.x = vehicle_vector.norm() / mobility_path_prediction_time_step_;

        // Set timestamp
        output_state.header.stamp = builtin_interfaces::msg::Time(prev_time_stamp);

        return std::make_pair(output_state, yaw);
    }

    carma_perception_msgs::msg::ExternalObjectList MotionComputationWorker::synchronizeAndAppend(const carma_perception_msgs::msg::ExternalObjectList& sensor_list, carma_perception_msgs::msg::ExternalObjectList mobility_path_list) const
    {
        carma_perception_msgs::msg::ExternalObjectList output_list;
        // Compare time_stamps of first elements of each list as they are guaranteed to be the earliest of the respective lists
        
        for (auto &path: mobility_path_list.objects)
        {
            // interpolate and match timesteps
            path = matchAndInterpolateTimeStamp(path, rclcpp::Time(sensor_list.header.stamp));
        }
        
        output_list.objects.insert(output_list.objects.begin(),sensor_list.objects.begin(),sensor_list.objects.end());
        output_list.objects.insert(output_list.objects.end(),mobility_path_list.objects.begin(),mobility_path_list.objects.end());
        return output_list;
    }

    carma_perception_msgs::msg::ExternalObject MotionComputationWorker::matchAndInterpolateTimeStamp(carma_perception_msgs::msg::ExternalObject path, const rclcpp::Time& time_to_match) const
    {
        carma_perception_msgs::msg::ExternalObject output = path;
        // empty predictions
        output.predictions = {};

        // add the first point to start of the predictions to easily loop over
        carma_perception_msgs::msg::PredictedState prev_state;
        prev_state.header.stamp = output.header.stamp;
        prev_state.predicted_position.orientation = output.pose.pose.orientation ;         
        prev_state.predicted_velocity = output.velocity.twist ;    
        prev_state.predicted_position.position.x = output.pose.pose.position.x ;        
        prev_state.predicted_position.position.y = output.pose.pose.position.y ;        
        prev_state.predicted_position.position.z = output.pose.pose.position.z ;  
        path.predictions.insert(path.predictions.begin(), prev_state);

        rclcpp::Time curr_time_to_match = time_to_match;
        // because of this logic, we would not encounter mobility path
        // that starts later than the time we are trying to match (which is starting time of sensed objects)
        bool is_first_point = true;
        carma_perception_msgs::msg::PredictedState new_state;
        for (auto const& curr_state : path.predictions)
        { 
            if (curr_time_to_match > curr_state.header.stamp )
            {
                prev_state = curr_state;
                continue;
            }

            // reaching here means curr_state starts later than the time we are trying to match
            rclcpp::Duration delta_t = rclcpp::Time(curr_state.header.stamp) - curr_time_to_match;
            double ratio = delta_t.seconds() / mobility_path_prediction_time_step_;
            double delta_x = curr_state.predicted_position.position.x - prev_state.predicted_position.position.x;
            double delta_y = curr_state.predicted_position.position.y - prev_state.predicted_position.position.y;
            double delta_z = curr_state.predicted_position.position.z - prev_state.predicted_position.position.z;

            // copy old unchanged parts
            new_state.header.stamp = curr_time_to_match;
            new_state.predicted_velocity = prev_state.predicted_velocity;
            new_state.predicted_position.orientation = prev_state.predicted_position.orientation;

            // interpolate position
            // we are "stepping back in time" to match the position
            new_state.predicted_position.position.x = curr_state.predicted_position.position.x - delta_x * ratio; 
            new_state.predicted_position.position.y = curr_state.predicted_position.position.y - delta_y * ratio; 
            new_state.predicted_position.position.z = curr_state.predicted_position.position.z - delta_z * ratio; 

            if (is_first_point) // we store in the body if it is the first point, not predictions
            {
                output.header.stamp = curr_time_to_match;
                output.pose.pose.orientation = new_state.predicted_position.orientation;
                output.velocity.twist = new_state.predicted_velocity;
                output.pose.pose.position.x = new_state.predicted_position.position.x;
                output.pose.pose.position.y = new_state.predicted_position.position.y;
                output.pose.pose.position.z = new_state.predicted_position.position.z;
                is_first_point = false;
            }
            else
            {
                output.predictions.push_back(new_state);
            }

            prev_state = curr_state;
            curr_time_to_match += rclcpp::Duration(mobility_path_prediction_time_step_ * 1e9);
        }

    return output;
    }

    tf2::Vector3 MotionComputationWorker::transform_to_map_frame(const tf2::Vector3& ecef_point) const
    {
        if (!map_projector_) {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }
            
        lanelet::BasicPoint3d map_point = map_projector_->projectECEF( { ecef_point.x(),  ecef_point.y(), ecef_point.z() } , -1); // Input should already be converted to m
        
        return tf2::Vector3(map_point.x(), map_point.y(), map_point.z());
    }


    tf2::Vector3 MotionComputationWorker::gnss_to_map(double lat, double lon, double ele) const
    {
        if (!map_projector_) {
            throw std::invalid_argument("No map projector available for gnss conversion");
        }

        // GPSPoint gps_point;
        // gps_point.lat = lat; // TODO if possible delete

            
        lanelet::BasicPoint3d map_point = map_projector_->forward( { lat,  lon, ele } ); // Input should already be converted to m
        
        return tf2::Vector3(map_point.x(), map_point.y(), map_point.z());
    }

    // NOTE heading will need to be set after calling this
    geometry_msgs::PoseWithCovariance MotionComputationWorker::pose_from_gnss(
        const lanelet::projection::LocalFrameProjector& projector, 
        const tf2::Quaternion& ned_in_map_rotation,
        const GPSPoint& gps_point, 
        const double& heading)
    {
        //// Convert the position information into the map frame using the proj library
        lanelet::BasicPoint3d map_point = projector.forward(gps_point);

        ROS_DEBUG_STREAM("map_point: " << map_point.x() << ", " << map_point.y() << ", " << map_point.z());

        if (fabs(map_point.x()) > 10000.0 || fabs(map_point.y()) > 10000.0)
        {  // Above 10km from map origin earth curvature will start to have a negative impact on system performance

            ROS_WARN_STREAM("Distance from map origin is larger than supported by system assumptions. Strongly advise "
                            "alternative map origin be used. ");
        }

        //// Convert the orientation information into the map frame
        // This logic assumes that the orientation difference between an NED frame located at the map origin and an NED frame
        // located at the GNSS point are sufficiently small that they can be ignored. Therefore it is assumed the heading
        // report of the GNSS system regardless of its postion in the map without change in its orientation will give the
        // same result (as far as we are concered).

        tf2::Quaternion R_m_n(ned_in_map_rotation);  // Rotation of NED frame in map frame
        tf2::Quaternion R_n_h;                       // Rotation of sensor heading report in NED frame
        R_n_h.setRPY(0, 0, heading * wgs84_utils::DEG2RAD);

        tf2::Quaternion R_m_s =
            R_m_n * R_n_h;  // Rotation of sensor in map frame under assumption that distance from map origin is
                            // sufficiently small so as to ignore local changes in NED orientation

            ROS_DEBUG_STREAM("R_m_n (x,y,z,w) : ( "
                        << R_m_n.x() << ", " << R_m_n.y() << ", "
                        << R_m_n.z() << ", " << R_m_n.w());
            
            ROS_DEBUG_STREAM("R_n_h (x,y,z,w) : ( "
                        << R_n_h.x() << ", " << R_n_h.y() << ", "
                        << R_n_h.z() << ", " << R_n_h.w());
            
            ROS_DEBUG_STREAM("R_h_s (x,y,z,w) : ( "
                        << R_h_s.x() << ", " << R_h_s.y() << ", "
                        << R_h_s.z() << ", " << R_h_s.w());

            ROS_DEBUG_STREAM("R_m_s (x,y,z,w) : ( "
                        << R_m_s.x() << ", " << R_m_s.y() << ", "
                        << R_m_s.z() << ", " << R_m_s.w());

        tf2::Transform T_m_s(R_m_s,
                            tf2::Vector3(map_point.x(), map_point.y(), map_point.z()));  // Reported position and orientation
                                                                                            // of sensor frame in map frame

        // TODO handle covariance. This should be doable with the recent changes

        // Populate message
        geometry_msgs::PoseWithCovariance pose;
        pose.pose.position.x = T_m_s.getOrigin().getX();
        pose.pose.position.y = T_m_s.getOrigin().getY();
        pose.pose.position.z = T_m_s.getOrigin().getZ();

        pose.pose.orientation.x = T_m_s.getRotation().getX();
        pose.pose.orientation.y = T_m_s.getRotation().getY();
        pose.pose.orientation.z = T_m_s.getRotation().getZ();
        pose.pose.orientation.w = T_m_s.getRotation().getW();

        return pose;
    }

} // namespace motion_computation
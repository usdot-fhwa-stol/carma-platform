#include "message_to_external_object_convertor.h"
#include <carma_v2x_msgs/psm.hpp>
#include <carma_perception_msgs/external_object.hpp>

namespace object
{

    class PsmToExternalObject : public MessageToExternalObjectConvertor<carma_v2x_msgs::msg::PSM>
    {

        std::string frame_id_;
        PsmToExternalObject(std::string frame_id) {
            frame_id_ = frame_id;
        }
        
        void convert(const carma_v2x_msgs::msg::PSM &in_msg, carma_perception_msgs::msg::ExternalObject &out_msg)
        {            
            out_msg.dynamic_obj = true; // If a PSM is sent then the object is dynamic since its a living thing

            // Generate a unique object id from the psm id
            out_msg.id = 0
            for (int i = in_msg.id.id.size() - 1; i >= 0; i--) { // using signed iterator to handle empty case
                out_msg.id |= in_msg.id.id[i] << (8*i);
            }

            // Additionally, store the id in the bsm_id field
            out_msg.bsm_id = in_msg.id.id;

            // Compute the pose
            out_msg.pose = pose_from_gnss(
                map_projector_, 
                ned_in_map_rotation_,
                { in_msg.position.latitude, in_msg.position.longitude, in_msg.position.elevation }, 
                in_msg.heading.heading
            );

            // Compute the position covariance
            // TODO

            in_msg.sec_mark.millisecond

            // Compute the timestamp

            out_msg.header.stamp = builtin_interfaces::msg::Time( get_psm_timestamp(in_msg) );
            out_msg.header.frame_id = frame_id_;

            // Set the type
            if (in_msg.basic_type.type == carma_v2x_msgs::PersonalDeviceUserType::A_PEDESTRIAN
                || in_msg.basic_type.type == carma_v2x_msgs::PersonalDeviceUserType::A_PUBLIC_SAFETY_WORKER
                || in_msg.basic_type.type == carma_v2x_msgs::PersonalDeviceUserType::AN_ANIMAL) // Treat animals like people since we have no internal class for that
            {
                out_msg.object_type = carma_perception_msgs::msg::ExternalObject::PEDESTRIAN;

                // Default pedestrian size
                // Assume a 
                // ExternalObject dimensions are half actual size
                // Here we assume 1.0, 1.0, 2.0
                out_msg.size.x = 0.5;
                out_msg.size.y = 0.5;
                out_msg.size.z = 1.0;

            } else if (in_msg.basic_type.type == carma_v2x_msgs::PersonalDeviceUserType::A_PEDALCYCLIST) {
                
                out_msg.object_type = carma_perception_msgs::msg::ExternalObject::MOTORCYCLE; // Currently external object cannot represent bicycles, but motor cycle seems like the next best choice
            
                // Default bicycle size
                out_msg.size.x = 1.0;
                out_msg.size.y = 0.5;
                out_msg.size.z = 1.0;

            } else {
                
                out_msg.object_type = carma_perception_msgs::msg::ExternalObject::UNKNOWN;
            
                // Default pedestrian size
                out_msg.size.x = 0.5;
                out_msg.size.y = 0.5;
                out_msg.size.z = 1.0;

            }

            // Set the velocity
            out_msg.velocity.twist.linear.x = in_msg.velocity.velocity;

            // Compute the velocity covariance TODO

            // Set the confidence
            // TODO

            // Compute predictions 
            // TODO
            // For prediction, if the prediction is available we will sample it
            // If not then assume linear motion



            
        }

        rclcpp::Time get_psm_timestamp(const carma_v2x_msgs::msg::PSM &in_msg) {
            
            boost::posix_time::ptime utc_time_of_current_psm; 

            // Get the utc epoch start time
            static const boost::posix_time::ptime inception_boost( boost::posix_time::time_from_string("1970-01-01 00:00:00.000") ); 

            // Determine if the utc time of the path history can be used instead of the sec_mark
            // The sec_mark is susceptible to large error on minute transitions due to missing "minute of the year" field
            // If the second mark in the path history is identical and the full utc time is provided with ms resolution 
            // then it can be assumed the initial_position is the same as the PSM data and the utc_time can be used instead of sec_mark
            if ((in_msg.presence_vector & carma_v2x_msgs::msg::PSM::HAS_PATH_HISTORY) 
                && (in_msg.path_history.presence_vector & carma_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION)
                && (in_msg.path_history.initial_position.presence_vector & carma_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME)
                && (in_msg.path_history.initial_position.utc_time.presence_vector 
                    & carma_v2x_msgs::msg::FullPositionVector::YEAR
                    & carma_v2x_msgs::msg::FullPositionVector::MONTH
                    & carma_v2x_msgs::msg::FullPositionVector::DAY
                    & carma_v2x_msgs::msg::FullPositionVector::HOUR
                    & carma_v2x_msgs::msg::FullPositionVector::MINUTE 
                    & carma_v2x_msgs::msg::FullPositionVector::SECOND)
                && in_msg.sec_mark.millisecond == in_msg.path_history.initial_position.utc_time.second)
            {
                RCLCPP_DEBUG_STREAM(get_logger(), "Using UTC time of path history to determine PSM timestamp. Assumed valid since UTC is fully specified and sec_mark == utc_time.seconds in this message.");

                boost::posix_time::time_duration time_of_day = hours(in_msg.path_history.initial_position.utc_time.hour) 
                    + minutes(in_msg.path_history.initial_position.utc_time.minute) 
                    + milliseconds(in_msg.path_history.initial_position.utc_time.second);
                
                boost::gregorian::date utc_day(in_msg.path_history.initial_position.utc_time.year, in_msg.path_history.initial_position.utc_time.month, in_msg.path_history.initial_position.utc_time.day);

                utc_time_of_current_psm = boost::posix_time::ptime(utc_day) + time_of_day;
                
            } else { // If the utc time of the path history cannot be used to account for minute change over, then we have to default to the sec mark
                
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), get_clock(), rclcpp::Duration(5, 0), 
                    "PSM PathHistory utc timstamp does not match sec_mark. Unable to determine the minute of the year used for PSM data. Assuming local clock is exactly synched. This is NOT ADVISED.");

                // Get the current ROS time
                auto current_time = get_clock()->now();
                
                // Convert the ros time to a boost duration
                boost::posix_time::time_duration duration_since_inception( lanelet::time::durationFromSec(current_time.seconds()) );
                
                // Get the current ROS time in UTC
                auto curr_time_boost = inception_boost + duration_since_inception;

                // Get duration of current day
                auto duration_in_day_till_current_time = curr_time_boost.time_of_day();

                // Extract hours and minutes
                long hour_count_in_day = duration_in_day_till_current_time.hours()
                long minute_count_in_hour = duration_in_day_till_current_time.minutes()

                // Get the duration of the minute in the day
                auto start_of_minute_in_day = hours(hour_count_in_day) + minutes(minute_count_in_hour)
                
                // Get the start of the day in ROS time
                boost::posix_time::ptime start_of_day(curr_time_boost.date());

                // Ge the start of the current minute in ROS time
                boost::posix_time::ptime utc_start_of_current_minute = start_of_day + start_of_minute_in_day;

                // Compute the UTC PSM stamp from the sec_mark using ROS time as the clock
                boost::posix_time::time_duration s_in_cur_minute = milliseconds(in_msg.sec_mark.millisecond);

                utc_time_of_current_psm = utc_start_of_current_minute + s_in_cur_minute;

            }

            boost::posix_time::time_duration nsec_since_epoch = utc_time_of_current_psm - inception_boost;

            if (nsec_since_epoch.is_special()) {
                RCLCPP_ERROR_STREAM(get_logger(), "Computed psm nsec_since_epoch is special (computation failed). Value effectively undefined.")
            }

            return rclcpp::Time(nsec_since_epoch.total_nanoseconds());
        }
    };

}
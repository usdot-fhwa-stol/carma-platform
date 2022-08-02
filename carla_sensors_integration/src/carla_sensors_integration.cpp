#include "carla_sensors_integration.h"

namespace carla_sensors
{
    CarlaSensors::CarlaSensors()
    {

    }

    void CarlaSensors::initialize()
    {
        nh_ = nh_.reset(new ros::CARMANodeHandle());
        
        points_raw_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_raw", 1);
        image_raw_pub_= nh_.advertise<sensor_msgs::Image>("image_raw", 1);
        image_color_pub_ = nh_.advertise<sensor_msgs::Image>("image_color",1);
        image_rect_pub_ = nh_.advertise<sensor_msgs::Image>("image_rect", 1);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

        gnss_fixed_fused_pub_ = nh_.advertise<gps_common::GPSFix>("gnss_fix_fused");


        //Subscribers
        point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("points_cloud", 10, CarlaSensors::point_cloud_cb);
        image_raw_sub_ = nh_.subscribe<sensor_msgs::Image>("/carla/{}/camera/image", 10, CarlaSensors::image_raw_cb);
        image_color_sub_ = nh_.subscribe<sensor_msgs::Image>("/carla/{}/camera/image_color", 10, CarlaSensors::image_color_cb);
        image_rect_sub_ = nh_.subscribe<sensor_msgs::Image>("/carla/{}/camera/image_rect", 10, CarlaSensors::image_rect_cb);
        gnss_fixed_fused_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("/carla/{}/gnss/gnss_fix_fused", 10, CarlaSensors::gnss_fixed_fused_cb);


    }

    void CarlaSensors::spin()
    {

    }

    void CarlaSensors::point_cloud_cb(sensor_msgs::PointCloud2 point_cloud)
    {
        if (point_cloud.data.size() == 0)
        {
            ROS_ERROR_STREAM("Empty point cloud data");
            exit(0);
        }
        point_cloud_msg.data = point_cloud.data;
        point_cloud_msg.fields = point_cloud.fields;
        point_cloud_msg.height = point_cloud.height;
        point_cloud_msg.is_bigendian = point_cloud.is_bigendian;
        point_cloud_msg.is_dense = point_cloud.is_dense;
        point_cloud_msg.point_step = point_cloud.point_step;
        point_cloud_msg.row_step = point_cloud.row_step;
        point_cloud_msg.width = point_cloud.width;

        points_raw_pub_.publish(point_cloud_msg);

    }

    void CarlaSensors::image_raw_cb(sensor_msgs::Image image_raw)
    {

        if (image_raw.data.size() == 0)
        {
            ROS_ERROR_STREAM("Invalid image");
        }

        image_raw_msg.data = image_raw.data;
        image_raw_msg.encoding = image_raw.encoding;
        image_raw_msg.height = image_raw.height;
        image_raw_msg.is_bigendian = image_raw.is_bigendian;
        image_raw_msg.step = image_raw.step;
        image_raw_msg.width = image_raw.width;

        image_raw_pub_.publish(image_raw_msg);

    }

    void CarlaSensors::image_color_cb(sensor_msgs::Image image_color)
    {
         if (image_raw.data.size() == 0)
        {
            ROS_ERROR_STREAM("Invalid image");
        }

        image_color_msg.data = image_color.data;
        image_color_msg.encoding = image_color.encoding;
        image_color_msg.height = image_color.height;
        image_color_msg.is_bigendian = image_color.is_bigendian;
        image_color_msg.step = image_color.step;
        image_color_msg.width = image_color.width;

        image_color_pub_.publish(image_rect_msg);

    }

    void CarlaSensors::image_rect_cb(sensor_msgs::Image image_rect)
    {
         if (image_raw.data.size() == 0)
        {
            ROS_ERROR_STREAM("Invalid image");
        }

        image_rect_msg.data = image_rect.data;
        image_rect_msg.encoding = image_rect.encoding;
        image_rect_msg.height = image_rect.height;
        image_rect_msg.is_bigendian = image_rect.is_bigendian;
        image_rect_msg.step = image_rect.step;
        image_rect_msg.width = image_rect.width;

        image_rect_pub_.publish(image_rect_msg);

    }

    void CarlaSensors::gnss_fixed_fused_cb(sensor_msgs::NavSatFix gnss_fixed)
    {
        gnss_fixed_msg.altitude = gnss_fixed.altitude;
        gnss_fixed_msg.latitude = gnss_fixed.latitude;
        gnss_fixed_msg.longitude = gnss_fixed.longitude;
        gnss_fixed_msg.position_covariance = gnss_fixed.position_covariance;
        gnss_fixed_msg.position_covariance_type = gnss_fixed.position_covariance_type;
        gnss_fixed_msg.status.status = gnss_fixed.status.status;

        gnss_fixed_fused_pub_.publish(gnss_fixed_msg);

    
    }

    sensor_msgs::PointCloud2 CarlaSensors::get_lidar_msg()
    {
        return point_cloud_msg;
    }
    sensor_msgs::Image CarlaSensors::get_image_raw_msg()
    {
        return image_raw_msg;
    }
    sensor_msgs::Image CarlaSensors::get_image_color_msg()
    {
        return image_color_msg;
    }
    sensor_msgs::Image CarlaSensors::get_image_rect_msg()
    {
        return image_rect_msg;
    }
    gps_common::GPSFix CarlaSensors::get_gnss_fixed_msg()
    {
        return gnss_fixed_msg;
    }

}
#ifndef COLOR_LASER_H
#define COLOR_LASER_H
 
#include <string>
#include <vector>
#include <boost/thread.hpp>
 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
 
namespace perception_common{
 
class ColorLaser
{
    public:
 
         ColorLaser(
             ros::NodeHandle& nh
         );
 
         ~ColorLaser();

         // Callbacks for subscriptions to ROS topics
 
         void colorImageCallback(const sensor_msgs::Image::ConstPtr& message);
         void laserPointCloudCallback(sensor_msgs::PointCloud2::Ptr message);
         void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& message);
 
     private:

         // Callback used to setup subscribers once there is a subscription
         // to the lidar_points2_color topic
 
         void startStreaming();
         void stopStreaming();
 
         // Messages for local storage of sensor data
 
         sensor_msgs::Image       color_image_;
         sensor_msgs::CameraInfo  camera_info_;
 
         sensor_msgs::PointCloud2 color_laser_pointcloud_;
 
         //
         // Publisher for the colorized laser point cloud
 
         ros::Publisher color_laser_publisher_; 
         //
         // Subscribers for image, point cloud, and camera info topics
 
         ros::Subscriber color_image_sub_;
         ros::Subscriber laser_pointcloud_sub_;
         ros::Subscriber camera_info_sub_;
 
         //
         // Node handle used for publishing/subscribing to topics
 
         ros::NodeHandle node_handle_;
 
         //
         // Mutex to assure callbacks don't interfere with one another
 
         boost::mutex data_lock_;
 
        //
       // The number of channels in our color image
 
         uint8_t image_channels_;
 };
 
 }// namespace
 
#endif
 


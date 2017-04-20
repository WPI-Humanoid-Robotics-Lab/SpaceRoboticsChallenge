#include <iostream>

#include <ros/ros.h>

#include <math.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>

#include <visualization_msgs/MarkerArray.h>

class StaircaseFilter{
private:

    ros::Subscriber pcl_sub;
    ros::Publisher pcl_filtered_pub,vis_pub;
    std::vector<float> zLower, zUpper;
    int NumSteps;

    void startFilter(const sensor_msgs::PointCloud2ConstPtr& input);
    void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float zLowerVal, float zUpperVal);
    void panelRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

public:
    StaircaseFilter(ros::NodeHandle nh)
    {
        pcl_sub =  nh.subscribe("/assembled_cloud2", 10, &StaircaseFilter::startFilter, this);
        pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/val_filter/staircasePointCloud", 1);
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "/val_filter/footstep_markers", 1 );
        NumSteps=7;
        zLower.push_back(0.09);
        zLower.push_back(0.3);
        zLower.push_back(0.5);
        zLower.push_back(0.7);
        zLower.push_back(0.9);
        zLower.push_back(1.10);
        zLower.push_back(1.28);

        zUpper.push_back(0.2);
        zUpper.push_back(0.45);
        zUpper.push_back(0.65);
        zUpper.push_back(0.85);
        zUpper.push_back(0.95);
        zUpper.push_back(1.15);
        zUpper.push_back(1.6);
    }
    ~StaircaseFilter()
    {
        pcl_sub.shutdown();
    }

};


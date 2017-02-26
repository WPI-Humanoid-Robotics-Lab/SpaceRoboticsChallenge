#include "perception_common/object_detection/object_detectors.h"

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

//Global variables. In actual implementation, these can be class variables
pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());
perception_utils::model_based_object_detector detector;

//Listening to the assembled cloud
void laserCallBack(const sensor_msgs::PointCloud2::Ptr msg) {
    if (model->empty()) {
        return;
    }

    // This is a blocking call. In actual implementation, it should either start in a
    // new thread or should only populate a local variable for pointcloud.
    detector.match_model(model, msg, detector.detection_algorithm::HOUGH);
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_object_detection");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/assembled_cloud2",10, laserCallBack);
    std::string model_filename_ = "/home/ninja/Downloads/PCL_test/satellite_dish.pcd";
    if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
        return (-1);
    }
    ros::spin();
}

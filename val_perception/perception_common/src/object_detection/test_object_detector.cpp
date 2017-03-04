#include "perception_common/object_detection/object_detectors.h"

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

//Global variables. In actual implementation, these can be class variables
pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());
perception_utils::model_based_object_detector detector;
ros::Publisher marker_pub;
void visualize_point(geometry_msgs::Pose goal);

//Listening to the assembled cloud
void laserCallBack(const sensor_msgs::PointCloud2::Ptr msg) {
    if (model->empty()) {
        return;
    }

    // This is a blocking call. In actual implementation, it should either start in a
    // new thread or should only populate a local variable for pointcloud.
    visualize_point(detector.match_model(model, msg, detector.detection_algorithm::ICP));
    return;
}

void visualize_point(geometry_msgs::Pose goal){

    std::cout<< "goal origin :\n"<< goal.position << std::endl;
    std::cout<< "goal orientation :\n"<< goal.orientation << std::endl;


    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "test_object_detector";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose = goal;

    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker_pub.publish(marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_object_detection");
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber sub = n.subscribe("/assembled_cloud2",10, laserCallBack);
    std::string model_filename_ = ros::package::getPath("val_task1") + "/models/satellite_dish_dense.pcd";
    if(argc == 2) {
        model_filename_ = argv[1];
    }
    if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
        return (-1);
    }
    ros::spin();
}

//
// Created by will on 5/24/17.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "perception_common/model_based_detector.h"

int main(int argc, char** argv) {
    ROS_DEBUG("[MBD] Init starting");

    ros::init(argc, argv, "rover_detection", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;

    ros::Publisher rover_location_pub = nh.advertise<geometry_msgs::PoseStamped>("rover_pose", 10);

    model_based_detector detector(nh, "/field/assembled_cloud2", "package://val_task2/models/mars_explorer.pcd");
    detector.setResolution(0.05);
    // Crop out the floor
    detector.setCrop({1000, 1000, 1000, 1}, {0, -1000, 0.05, 1});

    auto rate = ros::Rate(2);

    ROS_DEBUG("[MBD] Init complete");

    while (ros::ok()) {
        geometry_msgs::PoseStamped pose;
        if (detector.getDetection(pose)) {
            rover_location_pub.publish(pose);
            detector.clearDetection();
        }

        ROS_DEBUG_THROTTLE(10, "[MBD] Detection loop running...");

        ros::spinOnce();
        rate.sleep();
    }
}
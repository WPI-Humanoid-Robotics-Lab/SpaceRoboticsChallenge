//
// Created by will on 5/24/17.
//

#include <ros/ros.h>

#include "perception_common/model_based_detector.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rover_detection");

    ros::NodeHandle nh;

    model_based_detector detector("package://val_task2/models/mars_explorer.pcd");

    detector.setResolution(0.01);

    ros::spin();
}
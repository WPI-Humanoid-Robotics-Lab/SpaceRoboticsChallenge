//# pragma once

#ifndef _VAL_RECHABILITY_H_
#define _VAL_RECHABILITY_H_

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_robot.h>
#include <ros/package.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sstream>

#define SSTR(x) dynamic_cast< std::ostringstream & >((std::ostringstream() << std::dec << x)).str()

#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define GREEN   "\033[32m"      /* Green */
#define RESET   "\033[0m"

#define STATUS_BAR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define STATUS_BAR_WIDTH 60


class rechabilityMap {

private:
    ros::NodeHandle nh_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_ptr_;

    //planning_scene::PlanningScene planning_scene_;
    //planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ptr_;
    //planning_scene::PlanningScenePtr planning_scene_ptr_;

    //planning_scene::PlanningScene planning_scene_;

    // collision variables for self collision
    collision_detection::CollisionRequest collision_request_;
    collision_detection::CollisionResult collision_result_;

    //planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene_;

    // publishers
    ros::Publisher vis_pub_array_;
    ros::Publisher vis_pub_obs_ ;

    void print_status(long current_count, double total);
    void publishMarker(visualization_msgs::MarkerArray markerArray, visualization_msgs::Marker marker);
    FILE* initYaml(const std::string group_name, std::string filename);
    void closeYaml(FILE* yaml);
    visualization_msgs::Marker initObsMarker(void);

public:
    rechabilityMap(ros::NodeHandle nh);
    ~rechabilityMap();

    void fkMapGenerator(const std::string group_name);

};

#endif

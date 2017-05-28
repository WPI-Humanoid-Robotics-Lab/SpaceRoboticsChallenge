#ifndef CABLETASK_H
#define CABLETASK_H
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/val_gripper_control.h>
#include <val_controllers/robot_state.h>
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_controllers/val_wholebody_manipulation.h"

#define Y_OFFSET 0.05

enum class BUTTON_LOCATION{
    LEFT=0,
    RIGHT
};

class CableTask{
public:
    CableTask(ros::NodeHandle n, BUTTON_LOCATION button_side= BUTTON_LOCATION::LEFT);
    ~CableTask();
   bool grasp_cable(const geometry_msgs::Pose &goal, float executionTime=2.0f);

   geometry_msgs::QuaternionStamped leftHandOrientation() const;
   void setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation);

   geometry_msgs::QuaternionStamped rightHandOrientation() const;
   void setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation);

private:
   ros::NodeHandle nh_;
   tf::TransformListener listener_;
   armTrajectory armTraj_;
   gripperControl gripper_;
   RobotStateInformer *current_state_;
   cartesianPlanner* right_arm_planner_;
   cartesianPlanner* left_arm_planner_;
   chestTrajectory* chest_controller_;
   wholebodyManipulation* wholebody_controller_;
   geometry_msgs::QuaternionStamped leftHandOrientationAngled_ ;
   geometry_msgs::QuaternionStamped rightHandOrientationAngled_;
   geometry_msgs::QuaternionStamped leftHandOrientationPerpen_ ;
   geometry_msgs::QuaternionStamped rightHandOrientationPerpen_;
   BUTTON_LOCATION button_side_;
   /*Top Grip*/
//   const std::vector<float> leftShoulderSeed_ = {-0.23, -0.07, 0.75, -1.53, 1.21, -0.40, 0.0};
   /*Side Grip*/
//   const std::vector<float> leftShoulderSeed_ = {-0.04, -0.24, 0.49, -1.30, 0.71, 0.61, -0.24};
//   const std::vector<float> rightShoulderSeed_ = {-0.81, -0.15, 1.65, 1.42, 1.28, 0.0, -0.23};

   const std::vector<float> leftShoulderSeedInitial_ = {-0.81,0.0,0.65,-1.51,1.26,0.0,0.0};
   const std::vector<float> rightShoulderSeedInitial_ = {-0.23,0.0,0.65,1.49,1.29,0.0,0.0};
   const std::vector<float> rightAfterGraspShoulderSeed_ = {-0.57, 1.09, 0.65, 1.1, 1.18, -0.19, 0.31};
};

#endif // CABLETASK_H





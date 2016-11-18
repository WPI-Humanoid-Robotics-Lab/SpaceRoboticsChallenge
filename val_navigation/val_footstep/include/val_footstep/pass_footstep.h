#ifndef PASS_FOOTSTEP_HPP
#define PASS_FOOTSTEP_HPP

#include "ros/ros.h"
#include"geometry_msgs/PoseStamped.h"
#include"geometry_msgs/Pose2D.h"
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <humanoid_nav_msgs/StepTarget.h>
#include <val_footstep/StepTargetArray.h>
#include"geometry_msgs/Vector3.h"
#include"geometry_msgs/Quaternion.h"
#include "ihmc_msgs/FootstepDataListRosMessage.h"
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "ihmc_msgs/FootstepStatusRosMessage.h"
#include <tf2_ros/transform_listener.h>
#include"tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include "ros/time.h"
#include "tf/tf.h"


class ValkyrieWalker
{

public:
    ValkyrieWalker(ros::NodeHandle nh);
    ~ValkyrieWalker();

    /// \todo implement this function
    bool WalkToGoal(const geometry_msgs::Pose2D &goal);

    /// \todo implement this function
    bool WalkNStepsForward(int n, float step_size);

    /// \todo implement this function
    bool WalkNStepsBackward(int n, float step_size);

    /// return if robot is walking. Optional
    bool isWalking();

    void walk();

private:
    ros::NodeHandle n;
    ros::ServiceClient footstep_client ;
    ros::Publisher footsteps_to_val ;
    ros::Subscriber footstep_status ;
    int step_counter;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tf_listener;

    std_msgs::String right_foot_frame,left_foot_frame;

    void footstepStatusCB(const ihmc_msgs::FootstepStatusRosMessage & msg);
    void getCurrentStep(int side , ihmc_msgs::FootstepDataRosMessage& foot);
    ihmc_msgs::FootstepDataRosMessage getOffsetStep(int side, double x);

    /// \todo wrong implementation. get rid of this
    void waitForSteps( int n);

    void getFootstep(double goal_x,double goal_y,double goal_th,ihmc_msgs::FootstepDataListRosMessage &list);

};


#endif

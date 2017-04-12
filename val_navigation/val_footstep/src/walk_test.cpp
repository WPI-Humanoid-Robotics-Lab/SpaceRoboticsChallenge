#include <iostream>
#include<val_footstep/ValkyrieWalker.h>
#include<ihmc_msgs/FootstepDataListRosMessage.h>
#include<ihmc_msgs/EndEffectorLoadBearingRosMessage.h>
#include"geometry_msgs/Pose2D.h"
ros::Publisher load_pub;
void load_bearing(int armSide, int load)
{
    ihmc_msgs::EndEffectorLoadBearingRosMessage msg;
    msg.unique_id=1;
    msg.robot_side=armSide;
    msg.end_effector=0;
    msg.request=load;
    load_pub.publish(msg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_test");
    ros::NodeHandle nh;
    ValkyrieWalker walk(nh, 1.0,1.0,0,0.1);
    load_pub= nh.advertise<ihmc_msgs::EndEffectorLoadBearingRosMessage>("/ihmc_ros/valkyrie/control/end_effector_load_bearing",1,true);
    ROS_INFO("About to walk");/*
    geometry_msgs::Pose2D goal;
    goal.x = 6.5;
    goal.y = -1.0;
    goal.theta =0;
    walk.walkToGoal(goal);*/
    //    walk.turn(RIGHT);
    std::vector<float> x,y;
    x={0.35};
    y={0.0};

    // 0- load
    // 1- unload
    //load_bearing(LEFT,1);

    // load_bearing(LEFT,0);
    walk.walkPreComputedSteps(x,y,RIGHT);
    // load_bearing(RIGHT,0);
    // load_bearing(LEFT,1);
    //walk.walkPreComputedSteps(x,y,LEFT);


    //walk.walkNSteps(1,0.3);
    ROS_INFO("Command Executed");

    //walk.WalkNStepsBackward(5,0.4);

    return 0;
}


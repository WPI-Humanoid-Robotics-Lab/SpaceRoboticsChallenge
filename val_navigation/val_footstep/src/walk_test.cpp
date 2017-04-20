#include <iostream>
#include<val_footstep/ValkyrieWalker.h>
#include"geometry_msgs/Pose2D.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_test");
    ros::NodeHandle nh;
    ValkyrieWalker walk(nh, 1.0,1.0,0,0.18);
    ROS_INFO("About to walk");/*
    geometry_msgs::Pose2D goal;
    goal.x = 6.5;
    goal.y = -1.0;
    goal.theta =0;
    walk.walkToGoal(goal);*/
    //    walk.turn(RIGHT);
    std::vector<float> x,y;
    x={0.25};
    y={0.0};
    //load_bearing(RIGHT,1);
    //    walk.NudgeFoot(RIGHT,-1,0.1);

    // 0- load
    // 1- unload
    walk.load_eff(LEFT,EE_LOADING::UNLOAD);
    walk.load_eff(RIGHT,EE_LOADING::LOAD);
    //     load_bearing(RIGHT,0);
//        walk.walkPreComputedSteps(x,y,RIGHT);
    // load_bearing(RIGHT,0);
    // load_bearing(LEFT,1);
//        walk.walkPreComputedSteps(x,y,LEFT);


    //walk.walkNSteps(1,0.3);
    ROS_INFO("Command Executed");

    //walk.WalkNStepsBackward(5,0.4);

    return 0;
}


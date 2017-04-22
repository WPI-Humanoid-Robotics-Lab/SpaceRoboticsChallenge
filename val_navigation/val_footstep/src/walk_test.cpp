#include <iostream>
#include<val_footstep/ValkyrieWalker.h>
#include"geometry_msgs/Pose2D.h"

using namespace std;
vector<vector<float>> footstepList_L,footstepList_R;
vector<float> t;
ros::Subscriber fL,fR;
bool flag1=true;
bool flag2=true;

void getList_L(geometry_msgs::Vector3 step)
{
    t.clear();
    t.push_back(step.x);
    t.push_back(step.y);
    t.push_back(step.z);
    footstepList_L.push_back(t);
    ROS_INFO("size L",footstepList_L.size());
    if(footstepList_L.size()==7) flag1=false;
}

void getList_R(geometry_msgs::Vector3 step)
{
    t.clear();
    t.push_back(step.x);
    t.push_back(step.y);
    t.push_back(step.z);
    footstepList_R.push_back(t);
    ROS_INFO("size R",footstepList_R.size());
    if(footstepList_R.size()==7) flag2=false;
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_test");
    ros::NodeHandle nh;
    ValkyrieWalker walk(nh, 1.0,1.0,0,0.18);
    ROS_INFO("About to walk");

    //    while(flag1 || flag2)
    //    {
    //        fL =  nh.subscribe("/val_filter/footsteps/Left", 10,&getList_L);
    //        ros::spinOnce();
    //        fR =  nh.subscribe("/val_filter/footsteps/Right", 10,&getList_R);
    //        ros::spinOnce();
    //    }

    //    ROS_INFO("OUTSIDE");
    std::vector<float> x,y;
    x={0.30};
    y={0.0};

    //    //    0- load
    //    //    1- unload
    //    walk.load_eff(LEFT,EE_LOADING::UNLOAD);
    //    walk.load_eff(RIGHT,EE_LOADING::LOAD);
        walk.walkPreComputedSteps(x,y,RIGHT);
//    walk.raiseLeg(LEFT,0.25);
//    walk.goTo(LEFT,0.32,0.17,0.2);
    //    walk.walkPreComputedSteps(x,y,LEFT);


    ROS_INFO("Command Executed");


    return 0;
}


#include <iostream>
#include<val_footstep/ValkyrieWalker.h>
#include<ihmc_msgs/FootstepDataListRosMessage.h>
#include"geometry_msgs/Pose2D.h"
int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_test");
    ros::NodeHandle nh;
    ValkyrieWalker walk(nh, 1.0,1.0,0);
    /*
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.transfer_time = 1;
    list.swing_time = 1;
    list.execution_mode =0;
    list.unique_id = -1;

    walk.WalkGivenSteps(list);

*/
    //

    geometry_msgs::Pose2D goal;
    bool flag =0;
    while(ros::ok)
    {
        if (flag == 0) {
            goal.x = 2.0;
            goal.y = 0.6;
            goal.theta = 1.57;
            walk.walkToGoal(goal);
            ROS_INFO("Command Executed");
            flag =1;
        }
        ros::spinOnce();
    }
    //walk.WalkNStepsForward(4,0.6);
    //walk.WalkNStepsBackward(5,0.4);

    return 0;
}

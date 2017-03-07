#include <ros/ros.h>
#include <val_footstep/ValkyrieWalker.h>

enum sm {
    PREPARE_START = 0,
    WALK_TO_DISH,
    SET_DISH_PITCH,
    SET_DISH_ROLL,
    WALK_TO_FINISHBOX
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "val_task1_node");
    ros::NodeHandle nh;
    float transferTime=0.4, swingTime=0.6;
    std::vector<float> x_offset;
    std::vector<float> y_offset;
    ValkyrieWalker walk(nh, transferTime, swingTime);

    sm state = PREPARE_START;
    switch (state)
    {
    case PREPARE_START:
    {
        ROS_INFO("prepare to start");
        state = WALK_TO_DISH;
    }
    case WALK_TO_DISH:
    {
        ROS_INFO("walk to dish");
        ros::Duration(0.5).sleep();
        walk.setWalkParms(transferTime, swingTime, 0);
        geometry_msgs::Pose2D goal;
        goal.x = 2.0;
        goal.y = 0.6;
        goal.theta = 1.57;
        walk.walkToGoal(goal);
        ROS_INFO("Command Executed");
        state = SET_DISH_PITCH;
    }
    case SET_DISH_PITCH:
    {

        ROS_INFO("Set dish pitch state");
        state = SET_DISH_ROLL;
    }
    case SET_DISH_ROLL:
    {


        ROS_INFO("Set dish pitch roll");
        state = WALK_TO_FINISHBOX;
    }

    case WALK_TO_FINISHBOX:
    {
        // Use IHMC robotpose topic to get your current location in world frame and robotstatus topic to see your current motion status
        // Use perception / predefined values /user defined values to calculate goal position.
        // Use 2D foot step planner to generate footsteps.
        // Walk to the goal position : Use ihmc foottrajectoryros messages or whole body ros message

        //TODO: Write CODE here
        ROS_INFO("walk to finish");
        break;
    }

    default:
        break;
    }
    ros::spinOnce();
    return 0;
}

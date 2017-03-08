#include<ros/ros.h>
#include<ihmc_msgs/WholeBodyTrajectoryRosMessage.h>
#include<val_common/val_common_defines.h>
#include"geometry_msgs/Pose2D.h"
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include "ihmc_msgs/FootstepDataListRosMessage.h"
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "ihmc_msgs/FootstepStatusRosMessage.h"
#include <tf2_ros/transform_listener.h>
#include"tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include "ros/time.h"
#include "tf/tf.h"

ihmc_msgs::ArmTrajectoryRosMessage appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage msg, float time, float* pos)
{

    ihmc_msgs::TrajectoryPoint1DRosMessage p;
    ihmc_msgs::OneDoFJointTrajectoryRosMessage t;
    t.trajectory_points.clear();

    for (int i=0;i<7;i++)
    {
        p.time = time;
        p.position = pos[i];
        p.velocity = 0;
        t.trajectory_points.push_back(p);
        msg.joint_trajectory_messages[i].trajectory_points.push_back(p);
    }

    return msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_test");
    ros::NodeHandle nh;
    std::string input;
    ros::Publisher pub= nh.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/whole_body_trajectory",1);
    ihmc_msgs::WholeBodyTrajectoryRosMessage wholebodymsg;
    while(ros::ok())
    {
        std::cout<<"Send another message?";
        std::cin>>input;
        if(input=="y")
        {
            wholebodymsg.unique_id=ros::Time::now().toSec();
            wholebodymsg.left_arm_trajectory_message.unique_id=wholebodymsg.unique_id+100;
            wholebodymsg.right_arm_trajectory_message.unique_id=wholebodymsg.unique_id+200;
            wholebodymsg.left_hand_trajectory_message.unique_id=wholebodymsg.unique_id+300;
            wholebodymsg.right_hand_trajectory_message.unique_id=wholebodymsg.unique_id+400;
            wholebodymsg.pelvis_trajectory_message.unique_id=wholebodymsg.unique_id+500;
            wholebodymsg.chest_trajectory_message.unique_id=wholebodymsg.unique_id+600;
            wholebodymsg.left_foot_trajectory_message.unique_id=wholebodymsg.unique_id+700;
            wholebodymsg.right_foot_trajectory_message.unique_id=wholebodymsg.unique_id+800;

            // Assigning sides
            wholebodymsg.left_arm_trajectory_message.robot_side=0;
            wholebodymsg.right_arm_trajectory_message.robot_side=1;

            wholebodymsg.left_foot_trajectory_message.robot_side=0;
            wholebodymsg.right_foot_trajectory_message.robot_side=1;

            wholebodymsg.left_hand_trajectory_message.robot_side=0;
            wholebodymsg.right_hand_trajectory_message.robot_side=1;

            // Right foot msg
            wholebodymsg.right_foot_trajectory_message.execution_mode=0;

            ihmc_msgs::SE3TrajectoryPointRosMessage trajPointR;
            wholebodymsg.right_foot_trajectory_message.taskspace_trajectory_points.clear();

            trajPointR.unique_id = 1; //(int)ros::Time::now();
            trajPointR.time = 2; // reach instantenously
            trajPointR.position.x = 0;//0.3;
            trajPointR.position.y = 0;//-0.1; //26366231343;
            trajPointR.position.z = 0;//0.04;
            trajPointR.orientation.x = 0; //-1.16587645216e-06;
            trajPointR.orientation.y = 0; //3.03336607205e-06;
            trajPointR.orientation.z = 0; //0.000763307697428;
            trajPointR.orientation.w = 1; //0.999999708675;
            trajPointR.time = 1.0;
            wholebodymsg.right_foot_trajectory_message.taskspace_trajectory_points.push_back(trajPointR);

            trajPointR.unique_id = 2; //(int)ros::Time::now();
            trajPointR.position.x = 0;//0.4;
            trajPointR.position.y = 0;//-0.0626366231343;
            trajPointR.position.z = 0;//0.0;
            trajPointR.orientation.x = 0;//-1.16587645216e-06;
            trajPointR.orientation.y = 0;//3.03336607205e-06;
            trajPointR.orientation.z = 0;//0.000763307697428;
            trajPointR.orientation.w = 0.999999708675;
            trajPointR.time = 2.0;
            wholebodymsg.right_foot_trajectory_message.taskspace_trajectory_points.push_back(trajPointR);

            trajPointR.unique_id = 3; //(int)ros::Time::now();
            trajPointR.position.x = 0;//0.4;
            trajPointR.position.y = 0;//-0.0626366231343;
            trajPointR.position.z = 0.0;
            trajPointR.orientation.x = 0;//-1.16587645216e-06;
            trajPointR.orientation.y = 0;//3.03336607205e-06;
            trajPointR.orientation.z = 0;//0.000763307697428;
            trajPointR.orientation.w = 0.999999708675;
            trajPointR.time = 3.0;
            wholebodymsg.right_foot_trajectory_message.taskspace_trajectory_points.push_back(trajPointR);

            // Left foot msg
            wholebodymsg.left_foot_trajectory_message.execution_mode=0;

            ihmc_msgs::SE3TrajectoryPointRosMessage trajPointL;
            wholebodymsg.left_foot_trajectory_message.taskspace_trajectory_points.clear();

            trajPointL.unique_id = 4; //(int)ros::Time::now();
            trajPointL.time = 2; // reach instantenously
            trajPointL.position.x = 0;
            trajPointL.position.y = 0; //26366231343;
            trajPointL.position.z = 0.;
            trajPointL.orientation.x = 0; //-1.16587645216e-06;
            trajPointL.orientation.y = 0; //3.03336607205e-06;
            trajPointL.orientation.z = 0; //0.000763307697428;
            trajPointL.orientation.w = 1; //0.999999708675;
            trajPointL.time = 0.0;
            wholebodymsg.left_foot_trajectory_message.taskspace_trajectory_points.push_back(trajPointL);

            // left and right arms

            float BUTTON_PRESS_PREPARE_R [] = {0.6, 0.25, 0.2, 1.1, 0.0, 0.0, 0.0};
            float BUTTON_PRESS_PREPARE_L [] = {-0.6, -0.25, 0.2, -1.1, 0.0, 0.0, 0.0};

            wholebodymsg.right_arm_trajectory_message.joint_trajectory_messages.resize(7);
            wholebodymsg.right_arm_trajectory_message = appendTrajectoryPoint(wholebodymsg.right_arm_trajectory_message, 2, BUTTON_PRESS_PREPARE_R);
            wholebodymsg.right_arm_trajectory_message.execution_mode=0;

            wholebodymsg.left_arm_trajectory_message.joint_trajectory_messages.resize(7);
            wholebodymsg.left_arm_trajectory_message = appendTrajectoryPoint(wholebodymsg.left_arm_trajectory_message, 2, BUTTON_PRESS_PREPARE_L);
            wholebodymsg.left_arm_trajectory_message.execution_mode=0;

            //pelvis
            wholebodymsg.pelvis_trajectory_message.execution_mode=0;

            ihmc_msgs::SE3TrajectoryPointRosMessage trajPointPelvis;
            wholebodymsg.pelvis_trajectory_message.taskspace_trajectory_points.clear();

            trajPointPelvis.unique_id = 4; //(int)ros::Time::now();
            trajPointPelvis.time = 2; // reach instantenously
            trajPointPelvis.position.x = 2.186;
            trajPointPelvis.position.y = 1.037; //26366231343;
            trajPointPelvis.position.z = 0.91;
            trajPointPelvis.orientation.x = 0.25; //-1.16587645216e-06;
            trajPointPelvis.orientation.y = 3.5; //3.03336607205e-06;
            trajPointPelvis.orientation.z = -1.31; //0.000763307697428;
            trajPointPelvis.orientation.w = 1; //0.999999708675;
            trajPointPelvis.time = 0.0;
            wholebodymsg.pelvis_trajectory_message.taskspace_trajectory_points.push_back(trajPointPelvis);

            // chest
            wholebodymsg.chest_trajectory_message.execution_mode=0;

            wholebodymsg.chest_trajectory_message.taskspace_trajectory_points.clear();

            ihmc_msgs::SO3TrajectoryPointRosMessage trajChest;
            trajChest.unique_id=9;
            trajChest.time=0;
            trajChest.orientation.x=0;
            trajChest.orientation.y=0;
            trajChest.orientation.z=0;
            trajChest.orientation.w=1;
            trajChest.angular_velocity.x=0;
            trajChest.angular_velocity.y=0;
            trajChest.angular_velocity.z=0;

            wholebodymsg.chest_trajectory_message.taskspace_trajectory_points.push_back(trajChest);

            // left hand

            wholebodymsg.left_hand_trajectory_message.execution_mode=0;

            ihmc_msgs::SE3TrajectoryPointRosMessage trajPointLHand;
            wholebodymsg.left_hand_trajectory_message.taskspace_trajectory_points.clear();

            trajPointLHand.unique_id = 10; //(int)ros::Time::now();
            trajPointLHand.time = 2; // reach instantenously
            trajPointLHand.position.x = 0.0;
            trajPointLHand.position.y = 0.6; //26366231343;
            trajPointLHand.position.z = 0.5;
            trajPointLHand.orientation.x = 0; //-1.16587645216e-06;
            trajPointLHand.orientation.y = 0; //3.03336607205e-06;
            trajPointLHand.orientation.z = 0; //0.000763307697428;
            trajPointLHand.orientation.w = 1; //0.999999708675;
            trajPointLHand.time = 0.0;
            wholebodymsg.left_hand_trajectory_message.taskspace_trajectory_points.push_back(trajPointLHand);

            //right hand

            wholebodymsg.right_hand_trajectory_message.execution_mode=0;

            ihmc_msgs::SE3TrajectoryPointRosMessage trajPointRHand;
            wholebodymsg.right_hand_trajectory_message.taskspace_trajectory_points.clear();

            trajPointRHand.unique_id = 10; //(int)ros::Time::now();
            trajPointRHand.time = 2; // reach instantenously
            trajPointRHand.position.x = 0.0;
            trajPointRHand.position.y = -0.6; //26366231343;
            trajPointRHand.position.z = 0.5;
            trajPointRHand.orientation.x = 0; //-1.16587645216e-06;
            trajPointRHand.orientation.y = 0; //3.03336607205e-06;
            trajPointRHand.orientation.z = 0; //0.000763307697428;
            trajPointRHand.orientation.w = 1; //0.999999708675;
            trajPointRHand.time = 0.0;
            wholebodymsg.right_hand_trajectory_message.taskspace_trajectory_points.push_back(trajPointRHand);

            pub.publish(wholebodymsg);
            ROS_INFO("Message published");
        }

        else
        {
            return 0;
        }
    }
    return 0;
}


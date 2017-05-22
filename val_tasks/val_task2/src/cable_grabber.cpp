#include "val_task2/cable_grabber.h"


cableGrabber::cableGrabber(ros::NodeHandle n):nh_(n), armTraj_(nh_), gripper_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    leftHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    /* Top Grip */

    leftHandOrientation_.quaternion.x = 0.604;
    leftHandOrientation_.quaternion.y = 0.434;
    leftHandOrientation_.quaternion.z = -0.583;
    leftHandOrientation_.quaternion.w = 0.326;

    //    /* Side Grip */
    //    leftHandOrientation_.quaternion.x = 0.155;
    //    leftHandOrientation_.quaternion.y = -0.061;
    //    leftHandOrientation_.quaternion.z = -0.696;
    //    leftHandOrientation_.quaternion.w = 0.699;

    //    /* Side Grip */
    //    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    rightHandOrientation_.quaternion.x = -0.094;
    //    rightHandOrientation_.quaternion.y = -0.027;
    //    rightHandOrientation_.quaternion.z = 0.973;
    //    rightHandOrientation_.quaternion.w = -0.209;

    /* Top Grip Flat Hand */
    //    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    rightHandOrientation_.quaternion.x = -0.576;
    //    rightHandOrientation_.quaternion.y = 0.397;
    //    rightHandOrientation_.quaternion.z = 0.632;
    //    rightHandOrientation_.quaternion.w = 0.332;

    /* Top Grip Slightly Bent Hand */
    //    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    rightHandOrientation_.quaternion.x = 0.640;
    //    rightHandOrientation_.quaternion.y = -0.380;
    //    rightHandOrientation_.quaternion.z = -0.614;
    //    rightHandOrientation_.quaternion.w = -0.261;

    /* Top Grip Flat Hand */
    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientation_.quaternion.x = -0.549;
    rightHandOrientation_.quaternion.y = 0.591;
    rightHandOrientation_.quaternion.z = 0.560;
    rightHandOrientation_.quaternion.w = 0.188;

    // cartesian planners for the arm
    left_arm_planner_ = new cartesianPlanner("leftPalm", VAL_COMMON_NAMES::WORLD_TF);
    right_arm_planner_ = new cartesianPlanner("rightMiddleFingerGroup", VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_ = new wholebodyManipulation(nh_);
    chest_controller_ = new chestTrajectory(nh_);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("modified_cable_position",1);
}

cableGrabber::~cableGrabber()
{
    delete left_arm_planner_;
    delete right_arm_planner_;
    delete wholebody_controller_;
}

void cableGrabber::grasp_cable(const armSide side, const geometry_msgs::Point &goal, float executionTime)
{
    // setting initial values
    geometry_msgs::QuaternionStamped* finalOrientationStamped;
    const std::vector<float>* seed;
    const std::vector<float>* seedAfter;
    std::string palmFrame;
    float palmToFingerOffset;
    if(side == armSide::LEFT){
        seed = &leftShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::L_END_EFFECTOR_FRAME;
        palmToFingerOffset = 0.07;//-0.15;
        finalOrientationStamped = &leftHandOrientation_;
    }
    else {
        seed = &rightShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME;
        palmToFingerOffset = -0.07;//0.15;
        finalOrientationStamped = &rightHandOrientation_;
        seedAfter=&rightAfterGraspShoulderSeed_;
    }

    // getting the orientation
    geometry_msgs::QuaternionStamped temp  = *finalOrientationStamped;
    current_state_->transformQuaternion(temp, temp);

    ROS_INFO("opening grippers");
    std::vector<double> gripper1,gripper2,gripper3;
    gripper1={1.2, 0.4, 0.3, 0.0 ,0.0 };
    gripper2={1.2, 0.6, 0.7, 0.0 ,0.0 };
    gripper3={1.2, 0.6, 0.7, 0.9 ,1.0 };
    gripper_.controlGripper(RIGHT,gripper1);
    ros::Duration(executionTime).sleep();

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
    std::vector< std::vector<float> > armData;
    armData.push_back(*seed);

    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*2).sleep();


    //move arm to given point with known orientation and higher z
    geometry_msgs::Point finalGoal, intermGoal;

    current_state_->transformPoint(goal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    intermGoal.z += 0.1;

    //transform that point back to world frame
    current_state_->transformPoint(intermGoal, intermGoal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

    ROS_INFO("Moving at an intermidate point before goal");

    //    geometry_msgs::Pose inter;

    //    inter.position=intermGoal;
    //    inter.orientation= temp.quaternion;


    //    armTraj_.moveArmInTaskSpace(side, inter, executionTime*2);
    //    ros::Duration(executionTime*2).sleep();

    //move arm to final position with known orientation

    current_state_->transformPoint(goal,finalGoal, VAL_COMMON_NAMES::WORLD_TF, palmFrame);
    current_state_->transformPoint(intermGoal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, palmFrame);

    intermGoal.y += palmToFingerOffset;
    intermGoal.z -= 0.0;//0.02; //finger to center of palm in Z-axis of hand frame
    finalGoal.y  += palmToFingerOffset; // this is to compensate for the distance between palm frame and center of palm
    finalGoal.z  -= 0.0;//0.02; //finger to center of palm in Z-axis of hand frame


    //transform that point back to world frame
    current_state_->transformPoint(finalGoal, finalGoal, palmFrame, VAL_COMMON_NAMES::WORLD_TF);
    current_state_->transformPoint(intermGoal, intermGoal, palmFrame, VAL_COMMON_NAMES::WORLD_TF);

    ROS_INFO("Moving towards goal");
    ROS_INFO_STREAM("Final goal"<<finalGoal);
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose final;

    final.position=intermGoal;
    final.orientation= temp.quaternion;

    waypoints.push_back(final);

    final.position=finalGoal;
    final.position.z+=0.01; // offset between cable and table
    final.orientation= temp.quaternion;

    waypoints.push_back(final);

    moveit_msgs::RobotTrajectory traj;
    if(side == armSide::LEFT)
    {
        left_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
    }
    else
    {
        right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
    }

    ROS_INFO("Calculated Traj");
    wholebody_controller_->compileMsg(side, traj.joint_trajectory);

    ros::Duration(executionTime).sleep();

    ROS_INFO("Grip Sequence 1");
    gripper_.controlGripper(RIGHT, gripper1);
    ros::Duration(0.1).sleep();
    ROS_INFO("Grip Sequence 2");
    gripper_.controlGripper(RIGHT, gripper2);
    ros::Duration(0.1).sleep();
    ROS_INFO("Grip Sequence 3");
    gripper_.controlGripper(RIGHT, gripper3);
    ros::Duration(0.1).sleep();

    ROS_INFO("Moving chest to zero position");
    chest_controller_->controlChest(0,0,0);

    armData.clear();
    armData.push_back(*seedAfter);
    ROS_INFO("Moving arms to intermediate position");
    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*2).sleep();

}

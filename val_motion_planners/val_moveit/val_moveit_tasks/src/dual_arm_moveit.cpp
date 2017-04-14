#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <val_control/val_arm_navigation.h>
#include <ros/ros.h>


int main(int argc, char** argv){

  ros::init(argc, argv, "dual_arm_moveit");
  ros::NodeHandle node_handle;

  /**************************************
     * ALWAYS START SPINNER IF USING MOVEIT
     *************************************/
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroup groupRight("rightShoulderPalm");
  moveit::planning_interface::MoveGroup groupLeft("leftShoulderPalm");

    ROS_INFO("set to state");
  // set the start state to the current state of the robot
  groupRight.setStartStateToCurrentState();
  groupLeft.setStartStateToCurrentState();

  // get the cuurent joints and their positions
  std::vector<std::string> jNamesRight,jNamesLeft;
  jNamesRight = groupRight.getActiveJoints();
  jNamesLeft = groupLeft.getActiveJoints();
  std::vector<double> jValuesRight, jValuesLeft;
  jValuesRight = groupRight.getCurrentJointValues();
  jValuesLeft = groupLeft.getCurrentJointValues();
  std::vector<std::string>::iterator it;
  std::vector<double>::iterator itd;
  ROS_INFO("current state");
  for (it=jNamesRight.begin(), itd=jValuesRight.begin(); it<jNamesRight.end(); it++, itd++){
    std::cout << *it << ": " << *itd << std::endl;
  }


  for (it=jNamesLeft.begin(), itd=jValuesLeft.begin(); it<jNamesLeft.end(); it++, itd++){
    std::cout << *it << ": " << *itd << std::endl;
  }

  ROS_INFO("Reference frame for Right: %s", groupRight.getPlanningFrame().c_str());
  ROS_INFO("Reference frame for right end effector: %s", groupRight.getEndEffectorLink().c_str());

  ROS_INFO("Reference frame for left: %s", groupLeft.getPlanningFrame().c_str());
  ROS_INFO("Reference frame for left end effector: %s", groupLeft.getEndEffectorLink().c_str());



  /**********************************************************************
     * These Parameters will alter the behaviour significantly
     *
     * RTT gives promising results
     * Planning attempts should be one
     * Planning time depends on the gola location (7 should be enough)
     * Goal tolerance is required as we dont have a defined orientation
  **********************************************************************/

  groupRight.setPlannerId("RRTkConfigDefault");
  groupRight.setNumPlanningAttempts(1);
  groupRight.setPlanningTime(7);
  groupRight.setGoalTolerance(0.1);

  groupLeft.setPlannerId("RRTkConfigDefault");
  groupLeft.setNumPlanningAttempts(1);
  groupLeft.setPlanningTime(7);
  groupLeft.setGoalTolerance(0.1);


  // set the target location
  geometry_msgs::Pose tPoseRight;
  geometry_msgs::Pose tPoseLeft;
  moveit::planning_interface::MoveGroup::Plan myPlanRight, myPlanLeft;

  armTrajectory armTraj(node_handle);
  // set first pose point
  // The orientation constrains below are for the first task.
  tPoseRight.orientation.w = 0.578;
  tPoseRight.orientation.x = -0.255;
  tPoseRight.orientation.y = -0.263;
  tPoseRight.orientation.z = 0.729;
  tPoseRight.position.x = 0.395;
  tPoseRight.position.y = -0.291;
  tPoseRight.position.z = 0.941;
  groupRight.setPoseTarget(tPoseRight);


  tPoseLeft.position.x = 0.428;
  tPoseLeft.position.y = 0.228;
  tPoseLeft.position.z = 1.063;
  tPoseLeft.orientation.x = 0.033;
  tPoseLeft.orientation.y = -0.112;
  tPoseLeft.orientation.z = -0.774;
  tPoseLeft.orientation.w = 0.622;
  groupLeft.setPoseTarget(tPoseLeft);


  if (groupRight.plan((myPlanRight)))
  {
        ROS_INFO("sucessfully planned the trajectory for right arm");
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator itp;

        std::cout << "joint names" << std::endl;
    for (it = myPlanRight.trajectory_.joint_trajectory.joint_names.begin(); it < myPlanRight.trajectory_.joint_trajectory.joint_names.end(); it++)
      std::cout << *it << std::endl;
    std::cout << "joint trajectories" << std::endl;
    for (itp = myPlanRight.trajectory_.joint_trajectory.points.begin(); itp < myPlanRight.trajectory_.joint_trajectory.points.end(); itp++)
      std::cout << *itp << std::endl;

    /* Sleep to give Rviz time to visualize the plan. */
    sleep(2.0);
    ROS_INFO("executing on robot\n");

    armTraj.moveArmTrajectory(RIGHT, myPlanRight.trajectory_.joint_trajectory);
  }
  else
  {
    ROS_INFO("planning failed for Right Arm");
  }

  if (groupLeft.plan((myPlanLeft)))
  {
        ROS_INFO("sucessfully planned the trajectory for left arm");
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator itp;

        std::cout << "joint names" << std::endl;
    for (it = myPlanLeft.trajectory_.joint_trajectory.joint_names.begin(); it < myPlanLeft.trajectory_.joint_trajectory.joint_names.end(); it++)
      std::cout << *it << std::endl;
    std::cout << "joint trajectories" << std::endl;
    for (itp = myPlanLeft.trajectory_.joint_trajectory.points.begin(); itp < myPlanLeft.trajectory_.joint_trajectory.points.end(); itp++)
      std::cout << *itp << std::endl;

    /* Sleep to give Rviz time to visualize the plan. */
    sleep(2.0);
    ROS_INFO("executing on robot\n");

    armTraj.moveArmTrajectory(LEFT, myPlanLeft.trajectory_.joint_trajectory);
  }
  else
  {
    ROS_INFO("planning failed for left arm");
  }



  ros::spin();

}

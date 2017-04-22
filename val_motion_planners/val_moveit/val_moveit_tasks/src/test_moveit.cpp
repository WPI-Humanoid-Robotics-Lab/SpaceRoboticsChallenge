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

  ros::init(argc, argv, "test_moveit");
  ros::NodeHandle node_handle;

  /**************************************
     * ALWAYS START SPINNER IF USING MOVEIT
     *************************************/
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroup group("rightShoulderPalm");

 /**********************************************************
  * This adds the detected wheel as an objectof the robot
  * We need to import the mesh here below
  * We need to specify the mesh positions below
  * *******************************************************/

//    Adding object to the robot to avoid collision
//  moveit_msgs::CollisionObject collision_object;
//  collision_object.header.frame_id = group.getPlanningFrame();
//  collision_object.object.id = "wheels";

//  shape_msgs::Mesh wheelMesh;
//  wheelMesh.triangles =

//  geometry_msgs::Pose wheelMeshPose;
//  wheelMeshPose.orientation.w =
//  wheelMeshPose.position.x =
//  wheelMeshPose.position.y =
//  wheelMeshPose.position.z =

//  collision_object.meshes.push_back(wheelMesh);
//  collision_object.mesh_poses.push_back(wheelMeshPose);
//  collision_object.operation = collision_object.ADD;

//  std::vector<moveit_msgs::CollisionObject> collision_objects;
//  collision_objects.push_back(collision_object);

//  ROS_INFO("Attach the object to the robot");
//  group.attachObject(collision_object.id);

//  /* Sleep to give Rviz time to show the object attached (different color). */
//  sleep(4.0);




    ROS_INFO("set to state");
  // set the start state to the current state of the robot
  group.setStartStateToCurrentState();

  // get the cuurent joints and their positions
  std::vector<std::string> jNames;
  jNames = group.getActiveJoints();
  std::vector<double> jValues;
  jValues = group.getCurrentJointValues();
  std::vector<std::string>::iterator it;
  std::vector<double>::iterator itd;
  ROS_INFO("current state");
  for (it=jNames.begin(), itd=jValues.begin(); it<jNames.end(); it++, itd++){
    std::cout << *it << ": " << *itd << std::endl;
  }

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());




  /**********************************************************************
     * These Parameters will alter the behaviour significantly
     *
     * RTT gives promising results
     * Planning attempts should be one
     * Planning time depends on the gola location (7 should be enough)
     * Goal tolerance is required as we dont have a defined orientation
     **********************************************************************/
  group.setPlannerId("RRTkConfigDefault");
  group.setNumPlanningAttempts(1);
  group.setPlanningTime(7);
  group.setGoalTolerance(0.1);

  // set the target location
  geometry_msgs::Pose t_pose_1;
  moveit::planning_interface::MoveGroup::Plan my_plan;

  armTrajectory armTraj(node_handle);
  // set first pose point
  // The orientation constrains below are for the first task.
  t_pose_1.orientation.w = 0.943;
  t_pose_1.orientation.x = -0.233;
  t_pose_1.orientation.y = 0.232;
  t_pose_1.orientation.z = -0.045;


  t_pose_1.position.x = 0.152;
  t_pose_1.position.y = -0.754;
  t_pose_1.position.z = 1.353;
  group.setPoseTarget(t_pose_1);
  group.setGoalTolerance(0.1);

  if (group.plan((my_plan)))
  {
        ROS_INFO("sucessfully planned the trajectory");
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator itp;

        std::cout << "joint names" << std::endl;
    for (it = my_plan.trajectory_.joint_trajectory.joint_names.begin(); it < my_plan.trajectory_.joint_trajectory.joint_names.end(); it++)
      std::cout << *it << std::endl;
    std::cout << "joint trajectories" << std::endl;
    for (itp = my_plan.trajectory_.joint_trajectory.points.begin(); itp < my_plan.trajectory_.joint_trajectory.points.end(); itp++)
      std::cout << *itp << std::endl;

    /* Sleep to give Rviz time to visualize the plan. */
    sleep(2.0);
    ROS_INFO("executing on robot\n");

    armTraj.moveArmTrajectory(RIGHT, my_plan.trajectory_.joint_trajectory);
  }
  else
  {
    ROS_INFO("planning failed");
  }

  // sleep for 20 sec before executing next point
  sleep(20.0);
  ROS_INFO("planning for second point");

  // set second target location
  t_pose_1.orientation.w = 1.0;
  t_pose_1.position.x = 0.28;
  t_pose_1.position.y = -1.0;
  t_pose_1.position.z = 1.3;
  group.setPoseTarget(t_pose_1);

  group.setPlanningTime(10);
  if (group.plan((my_plan)))
  {

    /* Sleep to give Rviz time to visualize the plan. */
    sleep(2.0);
    ROS_INFO("executing on robot\n");
    armTrajectory armTraj(node_handle);
    armTraj.moveArmTrajectory(RIGHT, my_plan.trajectory_.joint_trajectory);
  }
  else
  {
    ROS_INFO("planning failed");
  }


  ros::spin();

}

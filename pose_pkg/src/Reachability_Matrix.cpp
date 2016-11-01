#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "Reachability_Matrix");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Program Starts");
  /*
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  ROS_INFO("Call to Model Frame");
  /*
  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "right_arm" of the PR2
  // robot.
*/
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("left_palm");
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();


  //moveit::core::JointBoundsVector obj=joint_model_group->getActiveJointModelsBounds();


  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the left palm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  //  ROS_INFO("Joint Limit %s:", joint_model_group->getActiveJointModelsBounds());
  }

  /*
  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  // Set one joint in the right arm outside its joint limit

  joint_values[0] = 1.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  // Check whether any joint is outside its joint limits
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // Enforce the joint limits for this state and check again
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
  */

  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "r_wrist_roll_link" which is the most distal link in the
  // "right_arm" of the robot.

//ROS_INFO("vALUES DECLARATUIN STARTS");
/*
  float ll_1=-1.329;
  float ul_1=1.181;
  float ll_2=-0.13;
  float ul_2=0.666;
  float ll_3=-0.23;
  float ul_3=0.255;
  float ll_4=-2.85;
  float ul_4=2.0;
  float ll_5=-1.519;
  float ul_5=1.266;
  float ll_6=-3.1;
  float ul_6=2.18;
  float ll_7=-2.174;
  float ul_7=0.12;
  float ll_8=-2.019;
  float ul_8=3.14;
  float ll_9=-0.62;
  float ul_9=0.625;
  float ll_10=-0.36;
  float ul_10=0.49;
*/


  float ll_1=0;
  float ul_1=1;
  float ll_2=0;
  float ul_2=1;
  float ll_3=0;
  float ul_3=1;
  float ll_4=0;
  float ul_4=1;
  float ll_5=0;
  float ul_5=1;
  float ll_6=0;
  float ul_6=1;
  float ll_7=0;
  float ul_7=1;
  float ll_8=0;
  float ul_8=1;
  float ll_9=0;
  float ul_9=1;
  float ll_10=0;
  float ul_10=1;






  ros::NodeHandle n1;
  ros::Publisher vis_pub = n1.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  visualization_msgs::Marker marker;
  marker.header.frame_id = "pelvis";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
 marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
//ROS_INFO("Entering loop");

  //ROS_INFO("LOOP STARTS");
  for(std::size_t j1 = ll_1; j1 < ul_1; j1=j1+0.001)
  {
      for(std::size_t j2 = ll_2; j2 < ul_2; j2=j2+0.001)
      {
          for(std::size_t j3 = ll_3; j3 < ul_3; j3=j3+0.001)
          {
              for(std::size_t j4 = ll_4; j4 < ul_4; j4=j4+0.001)
              {
                  for(std::size_t j5 = ll_5; j5 < ul_5; j5=j5+0.001)
                  {
                      for(std::size_t j6 = ll_6; j6 < ul_6; j6=j6+0.001)
                      {
                          for(std::size_t j7 = ll_7; j7 < ul_7; j7=j7+0.001)
                          {
                              for(std::size_t j8 = ll_8; j8 < ul_8; j8=j8+0.001)
                              {
                                  for(std::size_t j9 = ll_9; j9 < ul_9; j9=j9+0.001)
                                  {
                                      for(std::size_t j10 = ll_10; j10 < ul_10; j10=j10+0.001)
                                      {
                                       //   ROS_INFO("iNNER Loop reached");

                                          joint_values[0] = j1;
                                          joint_values[1] = j2;
                                          joint_values[2] = j3;
                                          joint_values[3] = j4;
                                          joint_values[4] = j5;
                                          joint_values[5] = j6;
                                          joint_values[6] = j7;
                                          joint_values[7] = j8;
                                          joint_values[8] = j9;
                                          joint_values[9] = j10;
                                          //ROS_INFO("sET JOint angles");

                                          kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
                                          const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("leftPalm");

                                          // Print end-effector pose. Remember that this is in the model frame
                                          ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
                                          marker.pose.position.x = end_effector_state.translation()[0];
                                          marker.pose.position.y = end_effector_state.translation()[1];
                                          marker.pose.position.z = end_effector_state.translation()[2];

                                          //ROS_INFO("Translation in x-axis %f", marker.pose.position.x);

                                          vis_pub.publish( marker );



                                      }
                                  }
                              }
                          }
                      }
                  }
              }

          }

      }
  }


/*
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  //kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("leftPalm");

  // Print end-effector pose. Remember that this is in the model frame
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
  ros::shutdown();
  */

  return 0;
}

#include <iostream>

#include "klampt_gz_player/klampt_gz_player.h"

using namespace gazebo;

double asDouble(XmlRpc::XmlRpcValue rpc_value)
{
  if (rpc_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    return static_cast<double>(rpc_value);
  }
  else if (rpc_value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    return static_cast<int>(rpc_value);
  }

  return 0;
}

KlamptGzPlayer::KlamptGzPlayer() {}

void KlamptGzPlayer::run(int _argc, char **_argv)
{
  ros::init(_argc, _argv, "klampt_gz_player");

  // Load gazebo
  client::setup(_argc, _argv);

  // Create our node for communication
  transport::NodePtr node(new transport::Node());
  node->Init();

  // also grab a ros node handle
  ros::NodeHandle nh;

  // Publish to a Gazebo topic
  this->pub = node->Advertise<msgs::JointCmd>("~/valkyrie/joint_cmd");

  // Wait for a subscriber to connect
  ROS_INFO("Waiting for gazebo connection... (unpause it!)");
  this->pub->WaitForConnection();

  this->jointSub = nh.subscribe("/set_joint_state", 10, &KlamptGzPlayer::jointStateCallback, this);

  std::string param_name = "/valkyrie/pid";
  if (nh.hasParam(param_name)) {
    XmlRpc::XmlRpcValue pid_map;
    nh.getParam(param_name, pid_map);

    XmlRpc::XmlRpcValue::ValueStruct::iterator iter;
    ROS_INFO("fetching PID values from parameter server");
    for(iter = pid_map.begin(); iter != pid_map.end(); ++iter)
    {
      std::string joint_name = iter->first;
      XmlRpc::XmlRpcValue pid_value = iter->second;
      // somehow get PID values and set them in the message
      double target = asDouble(pid_value["target"]);
      double p = asDouble(pid_value["p"]);
      double i = asDouble(pid_value["i"]);
      double d = asDouble(pid_value["d"]);

      // publish the initial message setting PID for each joint
      msgs::JointCmd msg;
      std::string full_joint_name = "valkyrie::" + joint_name;
      ROS_INFO("full joint name: %s", full_joint_name.c_str());
      msg.set_name(full_joint_name);
      msgs::PID* pid = msg.mutable_position();
      pid->set_target(target);
      pid->set_p_gain(p);
      pid->set_i_gain(i);
      pid->set_d_gain(d);
      pub->Publish(msg);
    }

    ROS_INFO("Ready.");
    ros::spin();
  }
  else {
    ROS_ERROR("No parameter for PIDs");
  }



  // Make sure to shut everything down.
  client::shutdown();
}


void KlamptGzPlayer::jointStateCallback(const sensor_msgs::JointState& msg)
{
  ROS_INFO_ONCE("recieving joint state.");

  int num_joints = msg.name.size();
  for (int i = 0; i < num_joints; i++)
  {
    msgs::JointCmd command;
    std::string full_joint_name = "valkyrie::" + msg.name[i];
    command.set_name(full_joint_name);
    msgs::PID* pid = command.mutable_position();
    pid->set_target(msg.position[i]);
    ROS_DEBUG("name: %s, target: %f", full_joint_name.c_str(), msg.position[i]);
    pub->Publish(command);
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  KlamptGzPlayer kgp;
  kgp.run(_argc, _argv);
  return 0;
}

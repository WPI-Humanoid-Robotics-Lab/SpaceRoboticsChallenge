The goal is to take the output of a Klampt trajectory and "play" it back in gazebo.
To do this, we publish Joint messages on ~/gazebo/default/joints for the given model name and joint configuration
iterating over the file and publishing joint messages to the PIDs should work

# Nodes

##klampt_gz_player

reads pids from parameter server, publishes those initial pid values and targets. Then, continuously listens for new joint messages and sends them on to gazebo.

 - ROS subscribed topics:
  /set_joint_state [sensor_msgs/JointState]


## klampt_gz_loader

reads pids from parameter server, then publishes those initial pid valuds and targets, then exits.

  - Gz published topics:
   /gazebo/default/valkyrie/joint_cmd [gazebo.msgs.JointCmd]


## player

reads a csv file of the kind produced by klampt, and publishes them as joint messages

 - ROS published topics:
  /set_joint_state [sensor_msgs/JointState]




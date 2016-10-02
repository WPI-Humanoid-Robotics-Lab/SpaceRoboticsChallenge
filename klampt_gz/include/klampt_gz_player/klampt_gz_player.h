#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace gazebo;

class KlamptGzPlayer {

  public:
    KlamptGzPlayer();
    void run(int _argc, char **_argv);

  private:
    typedef std::map<std::string, std::map<std::string, double> > pid_param_t;
    ros::Subscriber jointSub;
    transport::PublisherPtr pub;

    void jointStateCallback(const sensor_msgs::JointState& msg);
};

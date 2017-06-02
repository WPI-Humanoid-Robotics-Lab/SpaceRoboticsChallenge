#include "val_task1/handle_detector.h"
#include <val_task1/pcl_handle_detector.h>


int main(int argc, char** argv)
{
    ros::init (argc,argv,"findHandleDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundButton = false;
    geometry_msgs::Pose panel_loc;
    panel_loc.position.x =  3.06;
    panel_loc.position.y = 0.255;
    panel_loc.position.z = 0;
    panel_loc.orientation.x = 0;
    panel_loc.orientation.y = 0;
    panel_loc.orientation.z = 0.381370615313;
    panel_loc.orientation.w = 0.924422227002;

//    tf::Quaternion q = tf::createQuaternionFromYaw(M_PI_4);
//    tf::quaternionTFToMsg(q, panel_loc.orientation);
    std::vector<geometry_msgs::Point> handle_locations;
    std::vector<geometry_msgs::Point> handleLocs;
    HandleDetector h1(nh);
    pcl_handle_detector  ph1(nh,panel_loc);

    while (numIterations <200)
    {
        foundButton = h1.findHandles(handleLocs);
        ph1.getDetections(handle_locations);
        ROS_INFO(foundButton ? "***** handles detected" : "xxxxx handles not detected");
        numIterations++;
    }

}


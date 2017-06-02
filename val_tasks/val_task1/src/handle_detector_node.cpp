#include "val_task1/handle_detector.h"
#include <val_task1/pcl_handle_detector.h>


int main(int argc, char** argv)
{
    ros::init (argc,argv,"findHandleDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundButton = false;
    geometry_msgs::Pose panel_loc;
    panel_loc.position.x =  2.80390357971;
    panel_loc.position.y = 0.379479408264;
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
    foundButton = h1.findHandles(handleLocs);
    ros::Rate loop(1);
    while (numIterations <200)
    {

        ros::spinOnce();

        ph1.getDetections(handle_locations);
        if (!handle_locations.size()) continue;
        ROS_INFO_STREAM(" Size: " << handle_locations.size() <<std::endl);
        geometry_msgs::Point temp;
        ROS_INFO_STREAM("Handle Locs before "<< handleLocs[0] << "\t" << handleLocs[1] << "\t" << handleLocs[2]<<"\t" << handleLocs[3]<< std::endl);
        double norm1 = std::sqrt(std::pow(handleLocs[1].x - handle_locations[1].x , 2) + std::pow(handleLocs[1].y - handle_locations[1].y , 2) + std::pow(handleLocs[1].z - handle_locations[1].z , 2));
        double norm2 = std::sqrt(std::pow(handleLocs[3].x - handle_locations[0].x , 2) + std::pow(handleLocs[3].y - handle_locations[0].y , 2) + std::pow(handleLocs[3].z - handle_locations[0].z , 2));
        ROS_INFO_STREAM("normLeft "<< norm1 << std::endl << " normRight" <<norm2<<std::endl);
        if(norm1 > 0.05 )
        {
            temp = handleLocs[1];
            handleLocs[1] = handleLocs[0];
            handleLocs[0] = temp;
        }
        if(norm2 > 0.05 )
        {
            temp = handleLocs[3];
            handleLocs[3] = handleLocs[2];
            handleLocs[2] = temp;
        }
        ROS_INFO_STREAM("Handle Locs after "<< handleLocs[0] << "\t" << handleLocs[1] << "\t" << handleLocs[2]<<"\t" << handleLocs[3]<< std::endl);
        ROS_INFO(foundButton ? "***** handles detected" : "xxxxx handles not detected");
        numIterations++;

        loop.sleep();
    }

}


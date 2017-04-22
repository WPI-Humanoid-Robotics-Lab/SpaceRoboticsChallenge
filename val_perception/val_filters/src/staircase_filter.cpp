#include <val_filters/staircase_filter.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
using namespace std;

void StaircaseFilter::startFilter(const sensor_msgs::PointCloud2ConstPtr& input){

    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 output;
    ros::Time startTime = ros::Time::now();
    Eigen::Vector4f xyz_centroid;

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    vector<float> fL,fR;
    vector<vector<float> > fLset,fRset;
    geometry_msgs::Vector3 gfL,gfR;
    float yoffset=0.125;
    int counter=0;
    for (int i = 0; i < NumSteps; ++i) {
        pcl::fromROSMsg(*input, *cloud);
        passThroughFilter(cloud,zLower[i],zUpper[i]);
        panelRemoval(cloud);
        if(!cloud->empty())
        {
            counter++;
            pcl::compute3DCentroid (*cloud,xyz_centroid);

            //Left foot
            marker.id=i;
            marker.header.frame_id = "/world";
            marker.header.stamp = ros::Time();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.27;
            marker.scale.y = 0.18;
            marker.scale.z = 0.01;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            fL.push_back(xyz_centroid[0]);
            fL.push_back(xyz_centroid[1]-yoffset);
            fL.push_back(xyz_centroid[2]);

            marker.pose.position.x =fL[0];
            marker.pose.position.y = fL[1];
            marker.pose.position.z = fL[2];
            fLset.push_back(fL);
            // stairstepL_pub.publish(fL);
            markerArray.markers.push_back(marker);

            // Right Foot
            marker.id = i+100;
            marker.header.frame_id = "/world";
            marker.header.stamp = ros::Time();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.27;
            marker.scale.y = 0.18;
            marker.scale.z = 0.01;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            fR.push_back(xyz_centroid[0]);
            fR.push_back(xyz_centroid[1]+yoffset);
            fR.push_back(xyz_centroid[2]);
            marker.pose.position.x =fR[0];
            marker.pose.position.y = fR[1];
            marker.pose.position.z = fR[2];
            fRset.push_back(fR);
            //stairstepR_pub.publish(fR);
            markerArray.markers.push_back(marker);
            fL.clear();
            fR.clear();
            *finalCloud+=*cloud;
        }
    }


    pcl::toROSMsg(*finalCloud, output);
    output.header.frame_id = "world";
    vis_pub.publish(markerArray);
    pcl_filtered_pub.publish(output);
    if(counter==NumSteps)
    {
        for (int i = 0; i < NumSteps; ++i) {
            gfL.x=fLset[i][0];
            gfL.y=fLset[i][1];
            gfL.z=fLset[i][2];

            gfR.x=fRset[i][0];
            gfR.y=fRset[i][1];
            gfR.z=fRset[i][2];
            ROS_INFO("x left at %d is %f",i+1,gfL.x);
            ROS_INFO("y left at %d is %f",i+1,gfL.y);
            ROS_INFO("z left at %d is %f",i+1,gfL.z);


            ROS_INFO("x right at %d is %f",i+1,gfR.x);
            ROS_INFO("y right at %d is %f",i+1,gfR.y);
            ROS_INFO("z right at %d is %f",i+1,gfR.z);

            stairstepL_pub.publish(gfL);
            stairstepR_pub.publish(gfR);
        }
        ros::Time endTime = ros::Time::now();
        std::cout<<"Point Cloud and Foot Steps calculated in : "<<endTime-startTime<<" secs \n";
        //exit(0);
        counter=0;
    }
    else
    {
        fLset.clear();
        fRset.clear();
        counter=0;
    }
}

void StaircaseFilter::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float zLowerVal, float zUpperVal){

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(1,2.8);
    pass_x.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-1.5,1.5);
    pass_y.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(zLowerVal,zUpperVal);
    pass_z.filter(*cloud);

}

void StaircaseFilter::panelRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.008);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    // check if plane is horizontal
    if(coefficients->values[2]>0.9 && coefficients->values[2]<1.1)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud);
    }
    else
    {
        cloud->clear();
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "staircase_filter");
    ROS_INFO("Starting staircase filter node");
    ros::NodeHandle n;
    StaircaseFilter s(n);
    ros::spin();
    return 0;
}

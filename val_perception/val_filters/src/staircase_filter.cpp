#include <val_filters/staircase_filter.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>


void StaircaseFilter::startFilter(const sensor_msgs::PointCloud2ConstPtr& input){

    ROS_INFO("ENTER");
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 output;
    ros::Time startTime = ros::Time::now();
    Eigen::Vector4f xyz_centroid;

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    float yoffset=0.125;

    for (int i = 0; i < NumSteps; ++i) {
        pcl::fromROSMsg(*input, *cloud);
        passThroughFilter(cloud,zLower[i],zUpper[i]);
        panelRemoval(cloud);
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

        marker.pose.position.x = xyz_centroid[0];
        marker.pose.position.y = xyz_centroid[1]-yoffset;
        marker.pose.position.z = xyz_centroid[2];
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

        marker.pose.position.x = xyz_centroid[0];
        marker.pose.position.y = xyz_centroid[1]+yoffset;
        marker.pose.position.z = xyz_centroid[2];
        markerArray.markers.push_back(marker);
        *finalCloud+=*cloud;
    }

    ros::Time endTime = ros::Time::now();
    std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;
    pcl::toROSMsg(*finalCloud, output);
    output.header.frame_id = "world";
    vis_pub.publish(markerArray);
    pcl_filtered_pub.publish(output);
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

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud);

}


int main(int argc, char** argv){
    ros::init(argc, argv, "staircase_filter");
    ROS_INFO("Starting staircase filter node");
    ros::NodeHandle n;
    StaircaseFilter s(n);
    ros::spin();
    return 0;
}

#include "val_task2/solar_array_detector.h"

ArrayDetector::ArrayDetector(ros::NodeHandle& nh) : nh_(nh), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    pcl_sub_ = nh_.subscribe("/field/assembled_cloud2", 10, &ArrayDetector::cloudCB, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/val_solar_plane/cloud2", 1, true);
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
}

ArrayDetector::~ArrayDetector()
{
    pcl_sub_.shutdown();
    pcl_pub_.shutdown();
}

void ArrayDetector::cloudCB(const sensor_msgs::PointCloud2::Ptr input)
{
    if (input->data.empty())
        return;
    //add mutex
    mtx_.lock();
    pcl::fromROSMsg(*input, *cloud_);
    mtx_.unlock();
}

bool ArrayDetector::getArrayPosition(const geometry_msgs::Pose2D& rover_pose)
{
    bool array_detected = false;

    rover_pose_ = rover_pose;

    if (cloud_->empty())
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //copy cloud into local variable
    mtx_.lock();
    input_cloud = cloud_;
    mtx_.unlock();

    //Segment the plane and pass through filter to narrow the filter region to the stairs
    if (input_cloud->empty())
        return false;

    ROS_INFO("Removing Rover from Point Cloud");

    roverRemove(input_cloud, out_cloud1);

    if (out_cloud1->empty())
        return false;
    ROS_INFO("Computing Normals");
    boxFilter(out_cloud1 , out_cloud2);

    if (out_cloud2->empty())
        return false;
    ROS_INFO("Computing Normals");
    normalSegmentation(out_cloud2, output_cloud);

    if (!output_cloud->empty())
        array_detected = true;

    //publish the output cloud for visualization
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*out_cloud2, output);
    output.header.frame_id = "world";
    pcl_pub_.publish(output);

    return array_detected;
}

void ArrayDetector::boxFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    geometry_msgs::Pose pelvisPose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint[0] = 0.0;
    minPoint[1] = -5.0;
    minPoint[2] = -0.7;
    minPoint.normalize();

    maxPoint[0] = 5.0;//(pelvisPose.position.x - rover_pose_.x) * 5;
    maxPoint[1] = 0.0;//(pelvisPose.position.y - rover_pose_.y) * 5;
    maxPoint[2] = 0.0;
    maxPoint.normalize();

    Eigen::Vector3f boxTranslatation;
    boxTranslatation[0] = pelvisPose.position.x;
    boxTranslatation[1] = pelvisPose.position.y;
    boxTranslatation[2] = pelvisPose.position.z;

    Eigen::Vector3f boxRotation;
    boxRotation[0] = 0;  // rotation around x-axis
    boxRotation[1] = 0;  // rotation around y-axis
    boxRotation[2] = tf::getYaw(pelvisPose.orientation);

    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setInputCloud(input);
    box_filter.setMin(minPoint);
    box_filter.setMax(maxPoint);
    box_filter.setTranslation(boxTranslatation);
    box_filter.setRotation(boxRotation);
    box_filter.setNegative(false);
    box_filter.filter(*output);
}


void ArrayDetector::normalSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Downsample the cloud for faster processing
    float leafsize  = 0.0003;
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (leafsize, leafsize, leafsize);
    grid.setInputCloud (input);
    grid.filter (*temp_cloud);

    ROS_INFO_STREAM("input" << temp_cloud->points.size() << std::endl);

    ROS_INFO("Computing Normals to each point");
    // Normals estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (temp_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);

    ROS_INFO("Removing NaN Normals");
    std::vector<int> index;
    pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals, index);

    auto inliers = boost::make_shared<pcl::PointIndices>();
    inliers->indices = index;
    inliers->header = input->header;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (temp_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*temp_cloud);

    if(temp_cloud->empty()){
        ROS_ERROR("There are no normals in the point cloud!");
        return;
    }

    for(size_t i = 0; i < cloud_normals->points.size(); i++){
        if(fabs(cloud_normals->points[i].normal_z) < 0.01)
            output->points.push_back(temp_cloud->points[i]);
    }

    if(output->empty()){
        ROS_ERROR("There are no points above the threshold...");
        return;
    }

    ROS_INFO_STREAM("output" << output->points.size() << std::endl);
}


void ArrayDetector::roverRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    /*tf::Quaternion quat(rover_loc_.orientation.x,rover_loc_.orientation.y,rover_loc_.orientation.z,rover_loc_.orientation.w);
    tf::Matrix3x3 rotation(quat);
    tfScalar roll,pitch,yaw;
    rotation.getRPY(roll,pitch,yaw);
    */
    double theta;
    theta = rover_pose_.theta;
/*
    if (isroverRight_)
    {
        theta = rover_loc_.theta-1.5708; // rover right of walkway
    }
    else
    {
        theta = rover_loc_.theta+1.5708;   //left of walkway
    }
*/
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint[0]=0;
    minPoint[1]=-2;
    minPoint[2]=0;

    maxPoint[0]=9;
    maxPoint[1]=+2;
    maxPoint[2]=3;
    Eigen::Vector3f boxTranslatation;
         boxTranslatation[0]=rover_pose_.x;
         boxTranslatation[1]=rover_pose_.y;
         boxTranslatation[2]=0.1;  // to remove the points belonging to the walkway
    Eigen::Vector3f boxRotation;
         boxRotation[0]=0;  // rotation around x-axis
         boxRotation[1]=0;  // rotation around y-axis
         boxRotation[2]=theta;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.

    pcl::CropBox<pcl::PointXYZ> box_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rover_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    box_filter.setInputCloud(input);
    box_filter.setMin(minPoint);
    box_filter.setMax(maxPoint);
    box_filter.setTranslation(boxTranslatation);
    box_filter.setRotation(boxRotation);
    box_filter.setNegative(false);
    box_filter.filter(*rover_cloud);
    if (rover_cloud->empty())
        return;

    auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
    coefficients->header.frame_id = "world";
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(rover_cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*rover_cloud);

    // Widen cloud in +/- y to make sure val doesn't walk into the side of it
    Eigen::Affine3f rover_shift_l, rover_shift_r;
    const auto rover_align = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());

    rover_shift_l.linear() = Eigen::Matrix3f::Identity();
    rover_shift_l.translation() = rover_align * Eigen::Vector3f(-0.2, -0.4, 0);
    rover_shift_r.linear() = Eigen::Matrix3f::Identity();
    rover_shift_r.translation() = rover_align * Eigen::Vector3f(-0.2, 0.4, 0);

    pcl::PointCloud<pcl::PointXYZ> rover_shifted_l, rover_shifted_r;
    pcl::transformPointCloud(*rover_cloud, rover_shifted_l, rover_shift_l);
    pcl::transformPointCloud(*rover_cloud, rover_shifted_r, rover_shift_r);

    rover_shifted_l += rover_shifted_r;

    box_filter.setNegative(true);
    box_filter.filter(*output);
}

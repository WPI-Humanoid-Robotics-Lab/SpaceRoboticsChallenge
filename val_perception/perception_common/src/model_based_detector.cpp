//
// Created by will on 5/24/17.
//

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
// OctreePointCloudVoxelCentroid<PointNormals> is not precompiled, so we need to include the impl file
#include <pcl/octree/octree_impl.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <visualization_msgs/MarkerArray.h>

#include "perception_common/model_based_detector.h"

// Bring the convenience typedefs into this scope, because typing the qualified name is anticonvenient
using InputPoint = model_based_detector::InputPoint;
using InputCloud = model_based_detector::InputCloud;
using InputNormalPoint = model_based_detector::InputNormalPoint;
using InputNormalCloud = model_based_detector::InputNormalCloud;
using FeaturePoint = model_based_detector::FeaturePoint;
using FeatureCloud = model_based_detector::FeatureCloud;

using MLSType = pcl::MovingLeastSquares<InputPoint, InputNormalPoint>;

model_based_detector::model_based_detector(ros::NodeHandle nh, const std::string &cloud_topic, InputCloud::ConstPtr model)
        : model_(model), tf_listener_() {
    cloud_sub_ = nh.subscribe(cloud_topic, 10, &model_based_detector::cloudCallback, this);
    debug_points_ = nh.advertise<InputNormalCloud>("mbd_debug", 10);
    env_points_ = nh.advertise<InputNormalCloud>("mbd_environment", 10);
    model_points_ = nh.advertise<InputNormalCloud>("mbd_model", 10);
    debug_markers_ = nh.advertise<visualization_msgs::MarkerArray>("mbd_markers", 10);
}

InputCloud::ConstPtr model_based_detector::readModelFromPath(const std::string &path) {
    std::string resolved_path = path;
    if (path.find("package://") == 0){
        resolved_path.erase(0, strlen("package://"));
        size_t pos = resolved_path.find("/");
        if (pos == std::string::npos) {
            throw std::runtime_error("Could not parse package:// format from '" + path + "'");
        }

        std::string package = resolved_path.substr(0, pos);
        resolved_path.erase(0, pos);
        std::string package_path = ros::package::getPath(package);

        if (package_path.empty()) {
            throw std::runtime_error("Package [" + package + "] does not exist (while parsing '" + path + "'");
        }

        resolved_path = package_path + resolved_path;
    }

    InputCloud::Ptr model_cloud = boost::make_shared<InputCloud>();
    if (pcl::io::loadPCDFile<InputPoint>(resolved_path, *model_cloud) == -1) {
       throw std::runtime_error("Couldn't read PCD file '" + path + "'");
    }

    // Make sure it's not empty
    if (model_cloud->size() == 0) {
        throw std::runtime_error("PCD file empty: '" + path + "'");
    }

    for (auto &point : model_cloud->points) {
        point.x *= 0.0254;
        point.y *= 0.0254;
        point.z *= 0.0254;
    }

    ROS_DEBUG_STREAM("Loaded model cloud with " << model_cloud->size() << " points");

    return model_cloud;
}

void model_based_detector::setResolution(const float res) { downsample_resolution_ = res; }

void model_based_detector::setCrop() { use_crop_box_ = true; }
void model_based_detector::setNoCrop() { use_crop_box_ = false; }
void model_based_detector::setCrop(const Eigen::Vector4f &max, const Eigen::Vector4f &min) {
    setCrop();
    crop_box_max_ = max;
    crop_box_min_ = min;
}

void model_based_detector::cloudCallback(const InputCloud::ConstPtr &cloud_in) {
    Eigen::Matrix4f detection;
    if (findModelTransformation(cloud_in, detection)) {
        // Todo: parameter for describing a transform relative to the object?

        detected_ = true;

        pcl_conversions::fromPCL(cloud_in->header, latest_detection_.header);

        latest_detection_.pose.position.x = detection(0, 3);
        latest_detection_.pose.position.y = detection(1, 3);
        latest_detection_.pose.position.z = detection(2, 3);

        Eigen::Quaternionf detection_quat(detection.topLeftCorner<3, 3>());
        latest_detection_.pose.orientation.x = detection_quat.x();
        latest_detection_.pose.orientation.y = detection_quat.y();
        latest_detection_.pose.orientation.z = detection_quat.z();
        latest_detection_.pose.orientation.w = detection_quat.w();
    }
}

bool model_based_detector::findModelTransformation(const InputCloud::ConstPtr &cloud_in,
                                                   Eigen::Matrix4f &detection) {
    ROS_DEBUG_STREAM("[MBD] Cloud received with " << cloud_in->size() << " points");

//    const geometry_msgs::PointStamped robot_viewpoint = getCurrentViewpoint(cloud_in->header.frame_id);

    const InputNormalCloud::ConstPtr &model_filtered = filterModel();
    const FeatureCloud::ConstPtr &model_features = computeFeatures(model_filtered);

    ROS_DEBUG_STREAM("[MBD] Filtered model has " << model_filtered->size() << " points, "
                                                 << model_features->size() << " features");

    const InputNormalCloud::ConstPtr &input_filtered = filterInput(cloud_in);
    const FeatureCloud::ConstPtr &input_features = computeFeatures(input_filtered);

    ROS_DEBUG_STREAM("[MBD] Filtered input has " << input_filtered->size() << " points, "
                                                 << input_features->size() << " features");

    auto te = boost::make_shared<pcl::registration::TransformationEstimationLM<InputNormalPoint, InputNormalPoint>>();
    auto warp_fn = boost::make_shared<pcl::registration::WarpPointRigid3D<InputNormalPoint, InputNormalPoint>>();
    te->setWarpFunction(warp_fn);

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    viewer->addPointCloud<InputNormalPoint> (input_filtered, "sample cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//    viewer->addPointCloudNormals<InputNormalPoint, InputNormalPoint> (input_filtered, input_filtered, 2, 0.05, "normals");
//    viewer->addCoordinateSystem (1.0);
//    viewer->initCameraParameters ();
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce (100);
//        ros::spinOnce();
//    }
    const float inlier_frac = input_filtered->size() * 0.3f / model_filtered->size();

    ROS_DEBUG_STREAM("[MBD] Requiring " << inlier_frac << " inliers");

    {
        pcl::ScopeTime t("Compute alignment");
        pcl::SampleConsensusPrerejective<InputNormalPoint, InputNormalPoint, FeaturePoint> align;
        align.setInputTarget(model_filtered);
        align.setTargetFeatures(model_features);
        align.setInputSource(input_filtered);
        align.setSourceFeatures(input_features);
        align.setMaximumIterations(500000); // Number of RANSAC iterations
        align.setNumberOfSamples(4); // Number of points to sample for generating/prerejecting a pose
        align.setCorrespondenceRandomness(5); // Number of nearest features to use
        align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance(1 * downsample_resolution_); // Inlier threshold
        align.setInlierFraction(inlier_frac); // Required inlier fraction for accepting a pose hypothesis
//        align.setTransformationEstimation(te);
        ROS_DEBUG_STREAM("[MBD] Alignment beginning");

        // I don't need the model_aligned cloud, but PCL needs it
        const auto model_aligned = boost::make_shared<InputNormalCloud>();
        align.align(*model_aligned);

        if (!align.hasConverged()) {
            ROS_DEBUG_STREAM("[MBD] Did not converge");
            return false;
        }

        auto model = *model_aligned;
        model.header.frame_id = "world";
        ros::Time time_st = ros::Time::now();
        model.header.stamp = static_cast<uint64_t>(time_st.toNSec() / 1e3);
        debug_points_.publish(model);

        ROS_DEBUG_STREAM("[MBD] Converged");

        detection = align.getFinalTransformation();

        return true;
    }
}

InputNormalCloud::ConstPtr model_based_detector::filterModel() {
    auto filtered_model = boost::make_shared<InputCloud>();
    {
        pcl::ScopeTime t("Model downsampling");

        pcl::octree::OctreePointCloudVoxelCentroid<InputPoint> oct(downsample_resolution_);
        oct.setInputCloud(model_);
        oct.addPointsFromInputCloud();

        filtered_model->header = model_->header;

        InputCloud::VectorType pts;
        oct.getVoxelCentroids(pts);
        ROS_DEBUG_STREAM("There's " << pts.size() << " centroids");
        for (const auto &pt : pts) {
            filtered_model->push_back(InputPoint());
            filtered_model->back().x = pt.x;
            filtered_model->back().y = pt.y;
            filtered_model->back().z = pt.z;
        }
    }

    // Finally, estimate normals
    auto model_normals = boost::make_shared<InputNormalCloud>();
    {
        pcl::ScopeTime t("Model normal estimation");

        MLSType mls;
        mls.setComputeNormals(true);
        mls.setPolynomialFit(false);
        mls.setSearchRadius(5 * downsample_resolution_);
        mls.setInputCloud(filtered_model);
        ///////
//        mls.setUpsamplingMethod(MLSType::RANDOM_UNIFORM_DENSITY);
//        mls.setPointDensity(1);
        ///////
        mls.process(*model_normals);
    }

    auto model_to_publish = boost::make_shared<InputNormalCloud>();
    pcl::transformPointCloudWithNormals(*model_normals, *model_to_publish, Eigen::Affine3f(Eigen::Translation3f(0, 0, 4)));

    model_to_publish->header.frame_id = "world";

    model_points_.publish(model_to_publish);
    // Publish the normals
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    pcl_conversions::fromPCL(model_to_publish->header, marker.header);
    marker.ns = "model_normals";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.005;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 1;
    marker.color.a = 1;
    for (const auto &pt : model_to_publish->points) {
        const Eigen::Vector3f endpt = pt.getVector3fMap() + pt.getNormalVector3fMap() * 0.05;

        geometry_msgs::Point ros_start, ros_end;
        ros_start.x = pt.x;
        ros_start.y = pt.y;
        ros_start.z = pt.z;
        ros_end.x = endpt.x();
        ros_end.y = endpt.y();
        ros_end.z = endpt.z();

        marker.points.push_back(ros_start);
        marker.points.push_back(ros_end);
    }
    markers.markers.push_back(marker);
    debug_markers_.publish(markers);

    return model_normals;
}

InputNormalCloud::ConstPtr model_based_detector::filterInput(const InputCloud::ConstPtr &cloud_in) {
    pcl::octree::OctreePointCloudVoxelCentroid<InputPoint> oct(downsample_resolution_);

    {
        pcl::ScopeTime t("Input crop");

        // First, crop
        if (use_crop_box_) {
            auto indices = boost::make_shared<std::vector<int>>();
            pcl::getPointsInBox(*cloud_in, crop_box_min_, crop_box_max_, *indices);
            oct.setInputCloud(cloud_in, indices);
        } else {
            oct.setInputCloud(cloud_in);
        }
        oct.addPointsFromInputCloud();
    }

    // Then, downsample
    auto filtered_input = boost::make_shared<InputCloud>();
    {
        pcl::ScopeTime t("Input downsampling");

        InputCloud::VectorType pts;
        oct.getVoxelCentroids(pts);
        for (const auto &pt : pts) {
            filtered_input->push_back(InputPoint());
            filtered_input->back().x = pt.x;
            filtered_input->back().y = pt.y;
            filtered_input->back().z = pt.z;
        }
    }

    // Finally, estimate normals
    auto input_normals = boost::make_shared<InputNormalCloud>();
    {
        pcl::ScopeTime t("Input normal estimation");

        MLSType mls;
        mls.setComputeNormals(true);
        mls.setPolynomialFit(false);
        mls.setSearchRadius(5 * downsample_resolution_);
        mls.setInputCloud(filtered_input);
        ///////
//        mls.setUpsamplingMethod(MLSType::RANDOM_UNIFORM_DENSITY);
//        mls.setPointDensity(1);
        ///////

        mls.process(*input_normals);
    }

    input_normals->header = cloud_in->header;
    env_points_.publish(input_normals);

    // Publish the normals
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    pcl_conversions::fromPCL(cloud_in->header, marker.header);
    marker.ns = "input_normals";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.005;
    marker.color.r = 0.5;
    marker.color.g = 1;
    marker.color.b = 0.5;
    marker.color.a = 1;
    for (const auto &pt : input_normals->points) {
        const Eigen::Vector3f endpt = pt.getVector3fMap() + pt.getNormalVector3fMap() * 0.05;

        geometry_msgs::Point ros_start, ros_end;
        ros_start.x = pt.x;
        ros_start.y = pt.y;
        ros_start.z = pt.z;
        ros_end.x = endpt.x();
        ros_end.y = endpt.y();
        ros_end.z = endpt.z();

        marker.points.push_back(ros_start);
        marker.points.push_back(ros_end);
    }
    markers.markers.push_back(marker);
    debug_markers_.publish(markers);

    return input_normals;
}

FeatureCloud::ConstPtr model_based_detector::computeFeatures(const InputNormalCloud::ConstPtr &cloud_in) {
    ROS_DEBUG("Starting computeFeatures");
    pcl::ScopeTime t("Compute features");

    pcl::FPFHEstimation<InputNormalPoint, InputNormalPoint, FeaturePoint> fest;
    fest.setRadiusSearch(0.2);
    fest.setInputCloud(cloud_in);
    fest.setInputNormals(cloud_in);
    auto cloud_features = boost::make_shared<FeatureCloud>();
    ROS_DEBUG("Starting feature estimation");
    fest.compute(*cloud_features);

    // Very brute force: overwrite the first histogram value with height
    // Slightly less brute force but still not great woudl be find the value with lowest variance and overwrite that one
//    for (std::size_t i = 0; i < cloud_features->size(); i++) {
//        cloud_features->at(i).histogram[0] = cloud_in->at(i).z;
//    }

    return cloud_features;
}

bool model_based_detector::getDetection(geometry_msgs::PoseStamped &detection) {
    if (detected_) {
        detection = latest_detection_;
    }

    return detected_;
}

void model_based_detector::clearDetection() {
    detected_ = false;
}

geometry_msgs::PointStamped model_based_detector::getCurrentViewpoint(const std::string &cloud_frame_name) {
    // Robot viewpoint is, by definintion, 0, 0, 0 in the hokuyo_link
    geometry_msgs::PointStamped hokuyo_frame, cloud_frame;
    hokuyo_frame.header.frame_id = "hokuyo_link";
    hokuyo_frame.header.stamp = ros::Time(0);
    hokuyo_frame.point.x = 0; // are these initialized to 0 already?
    hokuyo_frame.point.y = 0;
    hokuyo_frame.point.z = 0;

    tf_listener_.transformPoint(cloud_frame_name, hokuyo_frame, cloud_frame);

    return cloud_frame;
}
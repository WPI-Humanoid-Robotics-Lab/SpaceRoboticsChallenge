//
// Created by will on 5/24/17.
//

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>

#include "perception_common/model_based_detector.h"

// Bring the convenience typedefs into this scope, because typing the qualified name is anticonvenient
using ModelPoint = model_based_detector::ModelPoint;
using ModelCloud = model_based_detector::ModelCloud;
using InputPoint = model_based_detector::InputPoint;
using InputCloud = model_based_detector::InputCloud;

model_based_detector::model_based_detector(ros::NodeHandle nh, const std::string &cloud_topic, ModelCloud::ConstPtr model)
        : model_(model) {
    cloud_sub_ = nh.subscribe<InputCloud>(cloud_topic, 10, &model_based_detector::cloudCallback, this);
}

ModelCloud::ConstPtr model_based_detector::readModelFromPath(const std::string &path) {
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

    ModelCloud::Ptr model_cloud = boost::make_shared<ModelCloud>();
    if (pcl::io::loadPCDFile<ModelPoint>(resolved_path, *model_cloud) == -1) {
       throw std::runtime_error("Couldn't read PCD file '" + path + "'");
    }

    // Make sure it's not empty
    if (model_cloud->height * model_cloud->width == 0) {
        throw std::runtime_error("PCD file empty: '" + path + "'");
    }

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
    // Step 1: condition inputs
    const auto cloud = getConditionedInput(cloud_in);
    const auto model = getConditionedModel();
}
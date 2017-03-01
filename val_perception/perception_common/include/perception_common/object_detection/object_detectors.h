#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
//#include <pcl/filters/uniform_sampling.h>  //uncomment this for any other version of pcl
#include <pcl/keypoints/uniform_sampling.h> //comment this for any other version of pcl than 1.7
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>


#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose.h>

namespace perception_utils{

typedef pcl::PointXYZ       PointType;
typedef pcl::Normal         NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352        DescriptorType;

/**
 * @brief The model_based_object_detector class provides ability to detect an object using model matching techniques
 * in a pointcloud. This class matching using the algorithms defined in enum class detection_algorithm
 */
class model_based_object_detector{
public:
    /**
     * @brief model_based_object_detector default constructor
     */
    model_based_object_detector();
    /**
     * @brief The detection_algorithm enum provides a list of algorithms which can be used for detection.
     */
    enum class detection_algorithm{
        HOUGH = 0,
        GC = 1,
        ICP
    };

    /**
     * @brief match_model method matches a model in a scene and returns SE3 matrix for the position and orientation of origin of the model wrt the scene
     * @param model pointer to the pcl pointcloud of model
     * @param scene pointer to the pcl pointcloud of scene
     * @param algo  detection algorithm to be used.
     */
    geometry_msgs::Pose match_model(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, model_based_object_detector::detection_algorithm algo);

    /**
     * @brief match_model method matches a model in a scene and returns SE3 matrix for the position and orientation of origin of the model wrt the scene
     * @param model pointer to ros pointcloud2 of model
     * @param scene pointer to ros pointcloud2 of scene
     * @param algo  detection algorithm to be used.
     */
    geometry_msgs::Pose match_model(const sensor_msgs::PointCloud2::Ptr model, const sensor_msgs::PointCloud2::Ptr scene, model_based_object_detector::detection_algorithm algo);

    /**
     * @brief match_model method matches a model in a scene and returns SE3 matrix for the position and orientation of origin of the model wrt the scene
     * @param model_cloud pointer to the pcl pointcloud of model
     * @param scene pointer to ros pointcloud2 of scene
     * @param algo detection algorithm to be used.
     */
    geometry_msgs::Pose match_model(const pcl::PointCloud<PointType>::Ptr model_cloud, const sensor_msgs::PointCloud2::Ptr scene, model_based_object_detector::detection_algorithm algo);
    /**
     * @brief match_using_corrs matches a model in a scene using correspondence grouping and returns SE3 matrix for the position and orientation of the origin of the model wrt the scene
     * @param model pointer to the pcl pointcloud of model
     * @param scene pointer to the pcl pointcloud of scene
     * @param algo  detection algorithm to be used. It can be either HOUGH or GC.
     */
    geometry_msgs::Pose match_using_corrs(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, model_based_object_detector::detection_algorithm algo);

    geometry_msgs::Pose match_using_ICP(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene);

    void SE3_to_geometry_pose(Eigen::Matrix4f Transformation_matrix, geometry_msgs::Pose &goal);

    bool use_cloud_resolution() const;
    void setUse_cloud_resolution(bool use_cloud_resolution);

    bool use_hough() const;
    void setUse_hough(bool use_hough);

    float model_ss() const;
    void setModel_ss(float model_ss);

    float scene_ss() const;
    void setScene_ss(float scene_ss);

    float rf_rad() const;
    void setRf_rad(float rf_rad);

    float descr_rad() const;
    void setDescr_rad(float descr_rad);

    float cg_size() const;
    void setCg_size(float cg_size);

    float cg_thresh() const;
    void setCg_thresh(float cg_thresh);


protected:
    bool use_cloud_resolution_;
    bool use_hough_;
    float model_ss_;
    float scene_ss_;
    float rf_rad_;
    float descr_rad_;
    float cg_size_;
    float cg_thresh_;

    double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud);

};

} // end of namespace perception_utils

#endif // OBJECT_DETECTOR_H

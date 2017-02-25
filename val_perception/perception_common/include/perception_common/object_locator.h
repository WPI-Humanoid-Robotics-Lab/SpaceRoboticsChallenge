#ifndef OBJECT_LOCATOR_H
#define OBJECT_LOCATOR_H

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

#include <sensor_msgs/point_cloud_conversion.h>


namespace perception_utils{

typedef pcl::PointXYZ       PointType;
typedef pcl::Normal         NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352        DescriptorType;

class model_based_object_detector{
public:
    model_based_object_detector();
    enum class detection_algorithm{
        HOUGH = 0,
        GC = 1,
        ICP
    };

    void match_model(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, model_based_object_detector::detection_algorithm algo);
    void match_model(const sensor_msgs::PointCloud2::Ptr model, const sensor_msgs::PointCloud2::Ptr scene, model_based_object_detector::detection_algorithm algo);
    void match_model(const pcl::PointCloud<PointType>::Ptr model_cloud, const sensor_msgs::PointCloud2::Ptr scene, model_based_object_detector::detection_algorithm algo);

    void match_using_ICP(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene);


    bool show_keypoints() const;
    void setShow_keypoints(bool show_keypoints);

    bool show_correspondences() const;
    void setShow_correspondences(bool show_correspondences);

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
    bool show_keypoints_;
    bool show_correspondences_;
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

#endif // OBJECT_LOCATOR_H

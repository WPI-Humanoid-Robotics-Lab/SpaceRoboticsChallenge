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
#include <tf/tf.h>

namespace perception_utils{

class object_detection_algorithm;
class object_detection_ICP;
class object_detection_Correspondence;

typedef pcl::PointXYZ       PointType;
typedef pcl::Normal         NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352        DescriptorType;

void SE3_to_geometry_pose( Eigen::Matrix4f transformation_matrix, geometry_msgs::Pose &goal){

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(transformation_matrix(0,0)), static_cast<double>(transformation_matrix(0,1)), static_cast<double>(transformation_matrix(0,2)),
                  static_cast<double>(transformation_matrix(1,0)), static_cast<double>(transformation_matrix(1,1)), static_cast<double>(transformation_matrix(1,2)),
                  static_cast<double>(transformation_matrix(2,0)), static_cast<double>(transformation_matrix(2,1)), static_cast<double>(transformation_matrix(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);


    goal.position.x = static_cast<double>(transformation_matrix(0,3));
    goal.position.y = static_cast<double>(transformation_matrix(1,3));
    goal.position.z = static_cast<double>(transformation_matrix(2,3));

    goal.orientation.x = tfqt.getX();
    goal.orientation.y = tfqt.getY();
    goal.orientation.z = tfqt.getZ();
    goal.orientation.w = tfqt.getW();

    return;
}


/**
 * @brief The detection_algorithm enum provides a list of algorithms which can be used for detection.
 */
enum class detection_algorithm{
    CORRESPONDENCE=0,
    ICP
};

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
     * @brief match_model method matches a model in a scene and returns SE3 matrix for the position and orientation of origin of the model wrt the scene
     * @param model pointer to the pcl pointcloud of model
     * @param scene pointer to the pcl pointcloud of scene
     * @param algo  detection algorithm to be used.
     */
    geometry_msgs::Pose match_model(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, perception_utils::object_detection_algorithm* algo);

    /**
     * @brief match_model method matches a model in a scene and returns SE3 matrix for the position and orientation of origin of the model wrt the scene
     * @param model pointer to ros pointcloud2 of model
     * @param scene pointer to ros pointcloud2 of scene
     * @param algo  detection algorithm to be used.
     */
    geometry_msgs::Pose match_model(const sensor_msgs::PointCloud2::Ptr model, const sensor_msgs::PointCloud2::Ptr scene, perception_utils::object_detection_algorithm* algo);

    /**
     * @brief match_model method matches a model in a scene and returns SE3 matrix for the position and orientation of origin of the model wrt the scene
     * @param model_cloud pointer to the pcl pointcloud of model
     * @param scene pointer to ros pointcloud2 of scene
     * @param algo detection algorithm to be used.
     */
    geometry_msgs::Pose match_model(const pcl::PointCloud<PointType>::Ptr model_cloud, const sensor_msgs::PointCloud2::Ptr scene, perception_utils::object_detection_algorithm* algo);
    /**
     * @brief match_using_corrs matches a model in a scene using correspondence grouping and returns SE3 matrix for the position and orientation of the origin of the model wrt the scene
     * @param model pointer to the pcl pointcloud of model
     * @param scene pointer to the pcl pointcloud of scene
     * @param algo  detection algorithm to be used. It can be either HOUGH or GC.
     */
    geometry_msgs::Pose match_using_corrs(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, perception_utils::object_detection_Correspondence* algo);

    geometry_msgs::Pose match_using_ICP(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, perception_utils::object_detection_ICP* algo);

protected:
    double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud);

};

class object_detection_algorithm{
public:
    virtual detection_algorithm get_algorithm_name()=0;

};

class object_detection_ICP: public object_detection_algorithm {
public:
    object_detection_ICP(unsigned int max_iterations=5, float fitness_epsilon=0.01f);
    inline unsigned int get_max_iterations() const{
        return max_iterations_;
    }
    inline void set_max_iterations(unsigned int max_iterations = 5){
        max_iterations_ = max_iterations;
    }

    inline float get_fitness_epsilon() const{
        return fitness_epsilon_;
    }
    inline void set_fitness_epsilon(float fitness_epsilon=0.5){
        fitness_epsilon_ = fitness_epsilon;
    }

    virtual detection_algorithm get_algorithm_name() override{
        return detection_algorithm::ICP;
    }

protected:
    unsigned int max_iterations_;
    float fitness_epsilon_;
};

class object_detection_Correspondence: public object_detection_algorithm {
public:
    object_detection_Correspondence(bool use_cloud_resolution  = true,
                                    bool use_hough           = true,
                                    float model_ss          = 0.03f,
                                    float scene_ss          = 0.03f,
                                    float rf_rad           = 0.015f,
                                    float descr_rad         = 0.02f,
                                    float cg_size           = 0.01f,
                                    float cg_thresh         = 5.0f);

    inline bool get_use_cloud_resolution() const{
        return use_cloud_resolution_;
    }
    void set_use_cloud_resolution(bool use_cloud_resolution){
        use_cloud_resolution_ = use_cloud_resolution;
    }

    inline bool get_use_hough() const{
        return use_hough_;
    }
    void set_use_hough(bool use_hough){
        use_hough_ = use_hough;
    }

    inline float get_model_ss() const{
        return model_ss_;
    }
    void set_model_ss(float model_ss){
        model_ss_ = model_ss;
    }

    inline float get_scene_ss() const{
        return scene_ss_;
    }
    void set_scene_ss(float scene_ss){
        scene_ss_ = scene_ss;
    }

    inline float get_rf_rad() const{
        return rf_rad_;
    }
    void set_rf_rad(float rf_rad){
        rf_rad_ = rf_rad;
    }

    inline float get_descr_rad() const{
        return descr_rad_;
    }
    void set_descr_rad(float descr_rad){
        descr_rad_ = descr_rad;
    }

    float get_cg_size() const{
        return cg_size_;
    }
    void set_cg_size(float cg_size){
        cg_size_ = cg_size;
    }

    inline float get_cg_thresh() const{
        return cg_thresh_;
    }
    void set_cg_thresh(float cg_thresh){
        cg_thresh_ = cg_thresh;
    }

    virtual detection_algorithm get_algorithm_name() override{
        return detection_algorithm::CORRESPONDENCE;
    }

protected:
    bool use_cloud_resolution_;
    bool use_hough_;
    float model_ss_;
    float scene_ss_;
    float rf_rad_;
    float descr_rad_;
    float cg_size_;
    float cg_thresh_;
};

} // end of namespace perception_utils

#endif // OBJECT_DETECTOR_H

#include "perception_common/object_detection/object_detectors.h"

#include <visualization_msgs/Marker.h>
#include <perception_common/FeatureCloud.h>
#include <perception_common/TemplateAlignment.h>

namespace perception_utils {

model_based_object_detector::model_based_object_detector() {
}

double model_based_object_detector::computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud) {
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<PointType> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->size (); ++i) {
        if (! pcl_isfinite ((*cloud)[i].x)) {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2) {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0) {
        res /= n_points;
    }
    return res;
}

geometry_msgs::Pose model_based_object_detector::match_model(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, perception_utils::object_detection_algorithm* algo){

    switch (algo->get_algorithm_name()) {

    case perception_utils::detection_algorithm::CORRESPONDENCE:
    {
        perception_utils::object_detection_Correspondence* corrs_algo = static_cast<perception_utils::object_detection_Correspondence*>(algo);
        return match_using_corrs(model, scene, corrs_algo);
    }
    case perception_utils::detection_algorithm::SACIA:
    {
        perception_utils::object_detection_SACIA* sacia_algo = static_cast<perception_utils::object_detection_SACIA*>(algo);
        return match_using_SACIA(model, scene, sacia_algo);
    }
    case perception_utils::detection_algorithm::ICP:
    {
        perception_utils::object_detection_ICP* icp_algo = static_cast<perception_utils::object_detection_ICP*>(algo);
        return match_using_ICP(model, scene, icp_algo);
    }
    default:
    {
        break;
    }
    }
}

// This might never be used
geometry_msgs::Pose model_based_object_detector::match_model(const sensor_msgs::PointCloud2::Ptr model, const sensor_msgs::PointCloud2::Ptr scene, perception_utils::object_detection_algorithm* algo){

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*model,pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*model_cloud);
    }
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*scene,pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*scene_cloud);
    }

    return match_model(model_cloud, scene_cloud, algo);

}

geometry_msgs::Pose model_based_object_detector::match_model(const pcl::PointCloud<PointType>::Ptr model_cloud, const sensor_msgs::PointCloud2::Ptr scene, perception_utils::object_detection_algorithm* algo){

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*scene,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*scene_cloud);

    //    pcl::io::savePCDFile("/home/ninja/Downloads/PCL_test/scene.pcd",*scene);

    return match_model(model_cloud, scene_cloud, algo);

}

geometry_msgs::Pose model_based_object_detector::match_using_ICP(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, perception_utils::object_detection_ICP* algo) {

    std::cout << "Saved " << model->points.size () << " data points to input:"
              << std::endl;

    std::cout << "size:" << scene->points.size() << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //     icp.setInputCloud(model);
    icp.setInputSource(model);
    icp.setInputTarget(scene);
    icp.setMaximumIterations(algo->get_max_iterations());
    icp.setEuclideanFitnessEpsilon(algo->get_fitness_epsilon());
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    geometry_msgs::Pose goal;
    SE3_to_geometry_pose(icp.getFinalTransformation(), goal);
    return goal;
}

geometry_msgs::Pose model_based_object_detector::match_using_SACIA(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, perception_utils::object_detection_SACIA* algo) {
    std::vector<FeatureCloud> object_templates;
    object_templates.resize (0);

    FeatureCloud template_cloud;
    template_cloud.loadInputCloud (model);
    object_templates.push_back (template_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = scene;
    // pcl::io::loadPCDFile (scene, *cloud);

    // Filtering the cloud (should use camera initial guess)
    const float depth_limit = 1.0;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.1, 2);
    pass.filter (*cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.6, 2.7);
    pass.filter (*cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0, 4);
    pass.filter (*cloud);
    // downsampling the point cloud
    const float voxel_grid_size = 0.005f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud (cloud);
    vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
    vox_grid.filter (*tempCloud);
    cloud = tempCloud; 

    // Assign to the target FeatureCloud
    FeatureCloud target_cloud;
    target_cloud.setInputCloud (cloud);

    // Set the TemplateAlignment inputs
    TemplateAlignment template_align;
    for (size_t i = 0; i < object_templates.size (); ++i)
    {
    template_align.addTemplateCloud (object_templates[i]);
    }
    template_align.setTargetCloud (target_cloud);

    // Find the best template alignment
    TemplateAlignment::Result best_alignment;
    int best_index = template_align.findBestAlignment (best_alignment);
    const FeatureCloud &best_template = object_templates[best_index];

    // Print the alignment fitness score (values less than 0.00002 are good)
    printf ("Best fitness score: %f\n", best_alignment.fitness_score);

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
    Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

    printf ("\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));


    geometry_msgs::Pose goal;
    SE3_to_geometry_pose(best_alignment.final_transformation, goal);
    return goal;
}


geometry_msgs::Pose model_based_object_detector::match_using_corrs(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, perception_utils::object_detection_Correspondence* algo){
    pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

    //
    //  Set up resolution invariance
    //
    if (algo->get_use_cloud_resolution())
    {
        float resolution = static_cast<float> (computeCloudResolution (model));
        if (resolution != 0.0f)
        {
            algo->set_model_ss( algo->get_model_ss()*resolution);
            algo->set_scene_ss(algo->get_scene_ss()*resolution);
            algo->set_rf_rad(algo->get_rf_rad()*resolution);
            algo->set_descr_rad(algo->get_descr_rad()* resolution);
            algo->set_cg_size(algo->get_cg_size()*resolution);
        }

        std::cout << "Model resolution:       " << resolution << std::endl;
        std::cout << "Model sampling size:    " << algo->get_model_ss() << std::endl;
        std::cout << "Scene sampling size:    " << algo->get_scene_ss()<< std::endl;
        std::cout << "LRF support radius:     " << algo->get_rf_rad()<< std::endl;
        std::cout << "SHOT descriptor radius: " << algo->get_descr_rad()<< std::endl;
        std::cout << "Clustering bin size:    " << algo->get_cg_size()<< std::endl << std::endl;
    }

    //
    //  Compute Normals
    //
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch (10);   // why is this 10?
    norm_est.setInputCloud (model);
    norm_est.compute (*model_normals);

    norm_est.setInputCloud (scene);
    norm_est.compute (*scene_normals);

    //
    //  Downsample Clouds to Extract keypoints
    //
    // This is modified to allow use of PCL 1.7
    ///TODO: Sumanth: please add preprocessor directives to use the right call based on pcl version
    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (algo->get_model_ss());
    //uniform_sampling.filter (*model_keypoints);
    pcl::PointCloud<int> keypointIndices1;
    uniform_sampling.compute(keypointIndices1);
    pcl::copyPointCloud(*model, keypointIndices1.points, *model_keypoints);
    std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;


    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (algo->get_scene_ss());
    //uniform_sampling.filter (*scene_keypoints);
    pcl::PointCloud<int> keypointIndices2;
    uniform_sampling.compute(keypointIndices2);
    pcl::copyPointCloud(*scene, keypointIndices2.points, *scene_keypoints);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

    //
    //  Compute Descriptor for keypoints
    //
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (algo->get_descr_rad());

    descr_est.setInputCloud (model_keypoints);
    descr_est.setInputNormals (model_normals);
    descr_est.setSearchSurface (model);
    descr_est.compute (*model_descriptors);

    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (scene);
    descr_est.compute (*scene_descriptors);


    //
    //  Find Model-Scene Correspondences with KdTree
    //
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < scene_descriptors->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

    //
    //  Actual Clustering
    //
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Using Hough3D
    if (algo->get_use_hough())
    {
        //
        //  Compute (Keypoints) Reference Frames only for Hough
        //
        pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
        pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

        pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
        rf_est.setFindHoles (true);
        rf_est.setRadiusSearch (algo->get_rf_rad());

        rf_est.setInputCloud (model_keypoints);
        rf_est.setInputNormals (model_normals);
        rf_est.setSearchSurface (model);
        rf_est.compute (*model_rf);

        rf_est.setInputCloud (scene_keypoints);
        rf_est.setInputNormals (scene_normals);
        rf_est.setSearchSurface (scene);
        rf_est.compute (*scene_rf);

        //  Clustering
        pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
        clusterer.setHoughBinSize (algo->get_cg_size());
        clusterer.setHoughThreshold (algo->get_cg_thresh());
        clusterer.setUseInterpolation (true);
        clusterer.setUseDistanceWeight (false);

        clusterer.setInputCloud (model_keypoints);
        clusterer.setInputRf (model_rf);
        clusterer.setSceneCloud (scene_keypoints);
        clusterer.setSceneRf (scene_rf);
        clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //clusterer.cluster (clustered_corrs);
        clusterer.recognize (rototranslations, clustered_corrs);
    }
    else // Using GeometricConsistency
    {
        pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
        gc_clusterer.setGCSize (algo->get_cg_size());
        gc_clusterer.setGCThreshold (algo->get_cg_thresh());

        gc_clusterer.setInputCloud (model_keypoints);
        gc_clusterer.setSceneCloud (scene_keypoints);
        gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize (rototranslations, clustered_corrs);
    }

    //
    //  Output results
    //
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    }

    geometry_msgs::Pose goal;
    Eigen::Matrix4f transformation_matrix;
    if (!rototranslations.empty()) {
        transformation_matrix = rototranslations[0].block<4,4>(0, 0);
        SE3_to_geometry_pose(transformation_matrix, goal);
    }

    return goal;
}



object_detection_Correspondence::object_detection_Correspondence(bool use_cloud_resolution, bool use_hough, float model_ss, float scene_ss, float rf_rad, float descr_rad, float cg_size, float cg_thresh)
{
    use_cloud_resolution_   = use_cloud_resolution;
    use_hough_              = use_hough;
    model_ss_               = model_ss;
    scene_ss_               = scene_ss;
    rf_rad_                 = rf_rad;
    descr_rad_              = descr_rad;
    cg_size_                = cg_size;
    cg_thresh_              = cg_thresh;
}

object_detection_ICP::object_detection_ICP(unsigned int max_iterations, float fitness_epsilon)
{
    max_iterations_ = max_iterations;
    fitness_epsilon_ = fitness_epsilon;
}

object_detection_SACIA::object_detection_SACIA()
{
    
}


}

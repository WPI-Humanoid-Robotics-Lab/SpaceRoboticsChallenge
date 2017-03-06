README
======

* This library acts as an interface so we can talk to the multisense driver easily. It acts as a one point change of all multisense behaviour in our end.
* It connects to the topics that perception_common describes for multisense autonatically. Using roslaunch parameters it is possible to over write it. All library and code that depend on this library dont have to specify the multisense topics unless they want to overwrite the normal behaviour.

Notes
-----
* It does not subscibe to the topics unless a request has been made. This prevents the ros node to be attached to a topic without needing the information.

**External dependencies** - OpenCV2, PCL 

**ROS dependencies**  - cv_bridge, roscpp 

**Nodes created**  - test_laser, test_image, test_organizedRGBD

***

perception_common provides the basic functionality to access data from the ros topics to the native PCL/OpenCV data type. It can be thought of as a wrapper that ensures that everyone follows the same convention to access the topics. Also ensures that quirks or problems in the way multisense sends data are dealt with in one location.

***

**usage**

* for getting the image data from multisense     
 check the **test_image.cpp**                                 
 running the file:               
 `rosrun perception_common test_image default`    --> displayes the raw left image              
 `rosrun perception_common test_image disparoty`  --> displays the disparity image         
 also displays the fps(frames per sec) on a gnu-plot                

* for getting the laser scan data from multisense                   
 check the **test_laser.cpp**                                      
 running the file:                
 `rosrun perception common test_laser`  --> gives the point cloud                
 use the rviz and to visuslaise the point cloud                    

  **note:** for testing these the simulation should be running `roslaunch val_gazebo valkyrie_empty_world.launch`               

***
**Comments**
* Multisense publishes intensity as uint32 but pcl::PointXYZI expects intensity to be a float datatype. The type casting is done appropriately in this package.
* All the classes are under the namespace src_perception, this will prevent namespace colllsion
* perception common can be exported as a library in different package.


#### Using model_based_object_detector class
The model_based_object_detector class provides ability to detect an object in a pointcloud if a pointcloud model of that object is available. For example following lines can be used to detect using iterative closest point algorithm:
```
perception_utils::model_based_object_detector detector;
perception_utils::object_detection_ICP icp_algo;

# if pointcloud of model is loaded in *model object and that of scene is loaded in *scene then,
geometry_msgs::Pose pose = detector.match_model(model, scene, &icp_algo);
```

Parameters to be passed to ICP can be modified using its getters and setters as follows:
```
perception_utils::model_based_object_detector detector;
perception_utils::object_detection_ICP icp_algo;

# customize the parameters for ICP
icp_algo.set_fitness_epsilon(0.01f);
icp_algo.set_max_iterations(10);

# if pointcloud of model is loaded in *model object and that of scene is loaded in *scene then,
geometry_msgs::Pose pose = detector.match_model(model, scene, &icp_algo);
```

##### Adding a new algorithm

1. Add a new entry to enum class detection\_algorithm in `include/perception_common/object_detection/object_detectors.h`
```
enum class detection_algorithm{
    CORRESPONDENCE=0,
    SACIA,
    ICP,
    NDT,
    NEW_ALGORITHM
};
```
2. Define a new class for algorithm that extends `object_detection_algorithm`
```
class object_detection_NEW_ALGORITHM: public object_detection_algorithm {
public:
    object_detection_NEW_ALGORITHM(unsigned int param1=100);

    inline unsigned int get_param1() const{
        return param1_;
    }
    void set_param1(unsigned int param1 = 20){
        param1_ = param1;
    }
virtual detection_algorithm get_algorithm_name() override{
        return detection_algorithm::NEW_ALGORITHM;
    }
protected:
    unsigned int param1_;
};
```
3. Ensure that the new class has a default constructor that sets all the required parameters for the algorithm to their default values.
4. Declare and define a function with signature like this 
```
geometry_msgs::Pose match_using_ICP(const pcl::PointCloud<PointType>::Ptr model, const pcl::PointCloud<PointType>::Ptr scene, perception_utils::object_detection_ICP* algo);
```

5. Modify switch case in match\_model function in `src/perception_common/object_detection/object_detectors.cpp` file
```
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
    case perception_utils::detection_algorithm::NDT:
    {
        perception_utils::object_detection_NDT* ndt_algo = static_cast<perception_utils::object_detection_NDT*>(algo);
        return match_using_NDT(model, scene, ndt_algo);
    }
    case perception_utils::detection_algorithm::NEW_ALGORITHM:
    {
        perception_utils::object_detection_NEW_ALGORITHM* new_algo = static_cast<perception_utils::object_detection_NEW_ALGORITHM*>(algo);
        return match_using_NDT(model, scene, new_algo);
    }
    default:
    {
        break;
    }
    }
```
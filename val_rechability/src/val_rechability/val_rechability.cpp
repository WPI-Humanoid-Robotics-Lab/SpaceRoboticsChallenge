#include <val_rechability/val_rechability.h>

// default constructor
rechabilityMap::rechabilityMap(ros::NodeHandle nh): nh_(nh)
{
    // instatntiate robot model
    robot_model_loader_ = robot_model_loader::RobotModelLoader("robot_description");

    // get the kinematic model
    kinematic_model_ = robot_model_loader_.getModel();

    ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

    // construct the robot state
    kinematic_state_ptr_.reset(new robot_state::RobotState(kinematic_model_));
    // set all joints to default state
    kinematic_state_ptr_->setToDefaultValues();
    kinematic_state_ptr_->update();

    // initiate planning scene
    //    planning_scene_monitor_ptr_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    //    planning_scene_monitor_ptr_->requestPlanningSceneState();
    //    planning_scene_monitor_ptr_->startSceneMonitor();

    //locked_planning_scene_ = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_ptr_);

    //planning_scene_ = planning_scene::PlanningScene(kinematic_model_);
    // planning_scene::PlanningScene planning_scene_(kinematic_model_);
    //planning_scene_ptr_.reset(new planning_scene::PlanningScenePtr(kinematic_model_);
    //planning_scene_ptr_ = planning_scene_monitor_ptr_->getPlanningScene();

    vis_pub_array_ = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
    vis_pub_obs_ = nh_.advertise<visualization_msgs::Marker>( "visualization_markerObs", 0 );
}

// default deconstructor
rechabilityMap::~rechabilityMap()
{

}


void rechabilityMap::fkMapGenerator(const std::string group_name)
{
    planning_scene::PlanningScene planning_scene_(kinematic_model_);

    robot_state::RobotState& current_state_ = planning_scene_.getCurrentStateNonConst();
    current_state_.setToDefaultValues();
    // get the move group
    const robot_state::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);

    // get the joint names in the move group
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    // get the number of joints
    std::size_t jointCount = joint_names.size();

    // fetch the joint values of move group
    std::vector<double> joint_values;
    kinematic_state_ptr_->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < jointCount; ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // set collision request for the move group
    collision_request_.group_name = group_name;

    // get the joint limits
    robot_model::JointBoundsVector joint_bounds;
    joint_bounds = joint_model_group->getActiveJointModelsBounds();

    double total=1;
    long z=1;

    float sample_dt = 1;

    for(int i=0;i<jointCount;i++)
    {
        /*
    // find mod of last
    float last_mod = fmod((*joint_bounds[i])[0].max_position_,sample_dt);
    // fetch the last value
    float last_value = (last_mod == 0) ? (*joint_bounds[i])[0].max_position_ : (*joint_bounds[i])[0].max_position_-last_mod;
    // compute the values in this range
        total*=(((last_value-(*joint_bounds[i])[0].min_position_)/sample_dt) + 1);
        */

        total *= (round(((*joint_bounds[i])[0].max_position_ - (*joint_bounds[i])[0].min_position_)/sample_dt) + 1);
    }

    FILE* yaml_handle;
    const std::string filename ;
    yaml_handle = initYaml(group_name, filename);

    visualization_msgs::MarkerArray markerArray; //initMarker()
    //visualization_msgs::Marker markerObs = initObsMarker();

    visualization_msgs::Marker markerObs;
    markerObs.header.frame_id = "pelvis";
    markerObs.header.stamp = ros::Time();
    markerObs.ns = "my_namespace_no";
    markerObs.id = 0;
    markerObs.type = visualization_msgs::Marker::CUBE_LIST;
    markerObs.action = visualization_msgs::Marker::ADD;
    markerObs.pose.position.x = 0;
    markerObs.pose.position.y = 0;
    markerObs.pose.position.z = 0;
    markerObs.pose.orientation.x = 0.0;
    markerObs.pose.orientation.y = 0.0;
    markerObs.pose.orientation.z = 0.0;
    markerObs.pose.orientation.w = 1.0;
    markerObs.scale.x = 0.03;
    markerObs.scale.y = 0.03;
    markerObs.scale.z = 0.03;
    markerObs.color.a = 1.0; // Don't forget to set the alpha!
    markerObs.color.r = 0.7;
    markerObs.color.g = 0.0;
    markerObs.color.b = 0.1;
    markerObs.lifetime = ros::Duration(0);

    //    for(float j1 = (*joint_bounds[0])[0].min_position_; j1 <= (*joint_bounds[0])[0].max_position_; j1=j1+sample_dt)
    //    {
    //        for(float j2 = (*joint_bounds[1])[0].min_position_; j2 <= (*joint_bounds[1])[0].max_position_; j2=j2+sample_dt)
    //        {
    //            for(float j3 = (*joint_bounds[2])[0].min_position_; j3 <= (*joint_bounds[2])[0].max_position_; j3=j3+sample_dt)
    //            {
    //                for(float j4 = (*joint_bounds[3])[0].min_position_; j4 <= (*joint_bounds[3])[0].max_position_; j4=j4+sample_dt)
    //                {
    for(float j5 = (*joint_bounds[4])[0].min_position_; j5 <= (*joint_bounds[4])[0].max_position_; j5=j5+sample_dt)
    {
        for(float j6 = (*joint_bounds[5])[0].min_position_; j6 <= (*joint_bounds[5])[0].max_position_; j6=j6+sample_dt)
        {
            for(float j7 = (*joint_bounds[6])[0].min_position_; j7 <= (*joint_bounds[6])[0].max_position_; j7=j7+sample_dt)
            {
                for(float j8 = (*joint_bounds[7])[0].min_position_; j8 <= (*joint_bounds[7])[0].max_position_; j8=j8+sample_dt)
                {
                    for(float j9 = (*joint_bounds[8])[0].min_position_; j9 <= (*joint_bounds[8])[0].max_position_; j9=j9+sample_dt)
                    {
                        for(float j10 = (*joint_bounds[9])[0].min_position_; j10 <= (*joint_bounds[9])[0].max_position_;j10=j10+sample_dt)
                        {
                            //                                            joint_values[0] = j1;
                            //                                            joint_values[1] = j2;
                            //                                            joint_values[2] = j3;
                            //                                            joint_values[3] = j4;
                            joint_values[4] = j5;
                            joint_values[5] = j6;
                            joint_values[6] = j7;
                            joint_values[7] = j8;
                            joint_values[8] = j9;
                            joint_values[9] = j10;

                            print_status(z, total);
                            z++;

                            // kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
                            // kinematic_state->update(true);
                            // kinematic_state->updateCollisionBodyTransforms();

                            current_state_ = planning_scene_.getCurrentStateNonConst();
                            current_state_.setJointGroupPositions(joint_model_group, joint_values);
                            // current_state_.setToRandomPositions();
                            // ROS_INFO_STREAM("Current state is "<< (current_state_.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));
                            collision_request_.contacts = true;
                            collision_request_.max_contacts = 1000;

                            // check the slef collision
                            collision_result_.clear();
                            planning_scene_.checkSelfCollision(collision_request_, collision_result_);
                            //  ROS_INFO_STREAM("Current state is "<< (collision_result.collision ? "in" : "not in")
                            // << " self collision");
                            collision_detection::CollisionResult::ContactMap::const_iterator it;
                            for(it = collision_result_.contacts.begin();
                                it != collision_result_.contacts.end();
                                ++it)
                            {
                                // ROS_INFO("Contact between: %s and %s",it->first.first.c_str(),it->first.second.c_str());
                            }

                            std::vector<double> joint_values;
                            current_state_.copyJointGroupPositions(joint_model_group, joint_values);
                            for(std::size_t i = 0; i < jointCount; ++i)
                            {
                                //  ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
                            }

                            // if there is no self collision
                            //                                            if (collision_result.collision != true)
                            //                                            {
                            // get the transform and update the rechability map
                            const Eigen::Affine3d &end_effector_state = current_state_.getGlobalLinkTransform("leftPalm"); // kinematic_state->getGlobalLinkTransform("leftPalm");

                            geometry_msgs::Point p;
                            // Print end-effector pose. Remember that this is in the model frame
                            // ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
                            //   ROS_INFO("Valid State");

                            p.x = end_effector_state.translation()[0];
                            p.y = end_effector_state.translation()[1];
                            p.z = end_effector_state.translation()[2];


                            // get the orientation
                            Eigen::Quaterniond o(end_effector_state.rotation());

                            if (collision_result_.collision != true)
                            {
                                fprintf(yaml_handle, "map: %s\nresolution: %f\nself collision: %s\npoints: [%f, %f, %f, %f, %f, %f, %f]\n\n", filename.c_str(), sample_dt, "false", p.x, p.y, p.z, o.x(), o.y(), o.z(), o.w());

                                visualization_msgs::Marker marker;
                                marker.header.frame_id = "pelvis";
                                marker.header.stamp = ros::Time();
                                marker.ns = "my_namespace";
                                marker.id = z;
                                marker.type = visualization_msgs::Marker::CUBE;
                                marker.action = visualization_msgs::Marker::ADD;
                                marker.pose.position.x = p.x;
                                marker.pose.position.y = p.y;
                                marker.pose.position.z = p.z;
                                marker.pose.orientation.x = o.x();
                                marker.pose.orientation.y = o.y();
                                marker.pose.orientation.z = o.z();
                                marker.pose.orientation.w = o.w();
                                marker.scale.x = 0.1;
                                marker.scale.y = 0.01;
                                marker.scale.z = 0.01;
                                marker.color.a = 1.0; // Don't forget to set the alpha!
                                marker.color.r = 0.0;
                                marker.color.g = 0.7;
                                marker.color.b = 0.0;
                                marker.lifetime = ros::Duration(0);

                                //marker.points.push_back(p);
                                markerArray.markers.push_back(marker);
                            }
                            else
                            {
                                fprintf(yaml_handle, "map: %s\nresolution: %f\nself collision: %s\npoints: [%f, %f, %f, %f, %f, %f, %f]\n\n", filename.c_str(), sample_dt, "true", p.x, p.y, p.z, o.x(), o.y(), o.z(), o.w());
                                markerObs.points.push_back(p);
                                //      ROS_WARN("collison");
                            }
                        }
                    }
                }
            }
        }
    }
    //                }
    //            }
    //        }
    //    }

    closeYaml(yaml_handle);

    // publish the markers
    vis_pub_array_.publish(markerArray);
    vis_pub_obs_.publish(markerObs);
}

FILE* rechabilityMap::initYaml(const std::string group_name, std::string filename)
{
    char the_path[256];

    getcwd(the_path, 255);
    strcat(the_path, "/");

    ROS_INFO("%s\n", the_path);

    time_t currentTime;
    struct tm *localTime;
    time( &currentTime );
    localTime = localtime( &currentTime );
    int Day    = localTime->tm_mday;
    int Month  = localTime->tm_mon + 1;
    int Year   = localTime->tm_year + 1900;
    int Hour   = localTime->tm_hour;
    int Min    = localTime->tm_min;
    //int Sec    = localTime->tm_sec;

    filename = group_name+"_"+SSTR(Hour<<":"<<Min<<"_"<<Month<<":"<<Day<<":"<<Year<<"_rechability.yaml");
    struct stat st;

    // open yaml file to write
    std::string path(ros::package::getPath("val_rechability") + "/maps/");
    if (stat(path.c_str(), &st) != 0)
    {
        ROS_WARN("Path does not exist. Creating folder for maps");
    }
    const int dir_err = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (1 == dir_err)
    {
        ROS_ERROR("Error creating directory");
        exit(1);
    }

    std::string fullpath = path + filename;
    ROS_INFO("Saving map %s", filename.c_str());

    FILE* yaml = fopen(fullpath.c_str(), "w");

    return yaml;
}

void rechabilityMap::closeYaml(FILE* yaml)
{
    // close the file
    fclose(yaml);
}

//int main(int argc, char **argv)
//{
//    ros::init (argc, argv, "Reachability_Matrix");
//    ros::AsyncSpinner spinner(1);
//    spinner.start();

// get the max and min joint limits
//    double min_position_= (*joint_bounds[0])[0].min_position_;
//    double max_position_= (*joint_bounds[0])[0].max_position_;

//    float jointSampleIt[jointCount];
//    std::size_t last_joint_it = jointCount - 1;

//    for (std::size_t it = 0; it<jointCount - 1; it++)
//    {
//        sample();

//        if (it = last_joint_it)
//        {
//            while(jointSampleIt[it] < ul_10)
//            {
//                jointSampleIt[it] += sample_dt

//                updateJointValues(jointSampleIt);
//                kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

//                // check the slef collision
//                collision_result.clear();
//                planning_scene.checkSelfCollision(collision_request, collision_result);

//                // if there is no self collision
//                if (collision_result.collision != true)
//                {
//                    // get the transform and update the rechability map
//                    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("leftPalm");

//                    geometry_msgs::Point p;
//                    // Print end-effector pose. Remember that this is in the model frame
//                    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());


//                    p.x = end_effector_state.translation()[0];
//                    p.y = end_effector_state.translation()[1];
//                    p.z = end_effector_state.translation()[2];

//                    marker.points.push_back(p);
//                }
//            }

//            it = 0;
//        }
//    }

//}



//void updateJointValues (float* jointval)

//{
//    for (i=0;i<jointCount; i++)
//    {
//        joint_values[i] = jointval[i];
//    }
//}


void rechabilityMap::print_status(long current_count, double total)
{
    // compute the percentage
    double percent=((double)current_count/total);
    //   ROS_INFO("%ld %lf %lf",current_count, total, Y);

    int left_end = (int) (percent * STATUS_BAR_WIDTH);
    int right_end = STATUS_BAR_WIDTH - left_end;
    printf ("\r"GREEN"%.3lf"RESET"%% ["BOLDRED"%.*s%*s"RESET"]", (percent * 100), left_end, STATUS_BAR, right_end, "");
    fflush (stdout);
}

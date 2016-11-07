#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_robot.h>
#include <ros/package.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "H5Cpp.h"
#include <hdf5.h>

void print_status(long current_count, double total);

int main(int argc, char **argv)
{
    ros::init (argc, argv, "Reachability_Matrix");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // instatinate robot model and get the description
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    // construct the robot state
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    // set all joints to default state
    kinematic_state->setToDefaultValues();
    kinematic_state->update();

    // instantiate planning scene
    planning_scene::PlanningScene planning_scene(kinematic_model);
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    current_state.setToDefaultValues();

    // create collision variables for self collision
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    // get the move group
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("left_palm");

    // get the joint names in the move group
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    // get the number of joints
    std::size_t jointCount = joint_names.size();

    // fetch the joint values of move group
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < jointCount; ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // set collision request for the move group
    collision_request.group_name = "left_palm";

    // get the joint limits
    robot_model::JointBoundsVector joint_bounds;
    joint_bounds = joint_model_group->getActiveJointModelsBounds();

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

    float sample_dt = 0.4;

    ros::NodeHandle n1;
    ros::Publisher vis_pub = n1.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    ros::Publisher vis_pub_no = n1.advertise<visualization_msgs::Marker>( "visualization_marker_no", 0 );

    //visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "pelvis";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.7;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0);

    visualization_msgs::Marker marker_no;
    marker_no.header.frame_id = "pelvis";
    marker_no.header.stamp = ros::Time();
    marker_no.ns = "my_namespace_no";
    marker_no.id = 0;
    marker_no.type = visualization_msgs::Marker::CUBE_LIST;
    marker_no.action = visualization_msgs::Marker::ADD;
    marker_no.pose.position.x = 0;
    marker_no.pose.position.y = 0;
    marker_no.pose.position.z = 0;
    marker_no.pose.orientation.x = 0.0;
    marker_no.pose.orientation.y = 0.0;
    marker_no.pose.orientation.z = 0.0;
    marker_no.pose.orientation.w = 1.0;
    marker_no.scale.x = 0.03;
    marker_no.scale.y = 0.03;
    marker_no.scale.z = 0.03;
    marker_no.color.a = 1.0; // Don't forget to set the alpha!
    marker_no.color.r = 0.7;
    marker_no.color.g = 0.0;
    marker_no.color.b = 0.1;
    marker_no.lifetime = ros::Duration(0);

    char the_path[256];

    getcwd(the_path, 255);
    strcat(the_path, "/");

    ROS_INFO("%s\n", the_path);

    const std::string filename = "rechability.yaml";
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


    const char* fp =  "rechability.h5";
    // writing to h5f file
    hid_t file, group_poses;
    // create file
    file = H5Fcreate(fp, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    // create a group for poses
    group_poses = H5Gcreate(file, "/Poses", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    // create dataset for group_poses


    FILE* yaml = fopen(fullpath.c_str(), "w");

    double total=1;
    long z=0;
    for(int i=0;i<jointCount;i++)
    {
        total*=((((*joint_bounds[i])[0].max_position_)-((*joint_bounds[i])[0].min_position_))/sample_dt);
    }

    for(float j1 = (*joint_bounds[0])[0].min_position_; j1 < (*joint_bounds[0])[0].max_position_; j1=j1+sample_dt)
    {
        for(float j2 = (*joint_bounds[1])[0].min_position_; j2 < (*joint_bounds[1])[0].max_position_; j2=j2+sample_dt)
        {
            for(float j3 = (*joint_bounds[2])[0].min_position_; j3 < (*joint_bounds[2])[0].max_position_; j3=j3+sample_dt)
            {
                for(float j4 = (*joint_bounds[3])[0].min_position_; j4 < (*joint_bounds[3])[0].max_position_; j4=j4+sample_dt)
                {
                    for(float j5 = (*joint_bounds[4])[0].min_position_; j5 < (*joint_bounds[4])[0].max_position_; j5=j5+sample_dt)
                    {
                        for(float j6 = (*joint_bounds[5])[0].min_position_; j6 < (*joint_bounds[5])[0].max_position_; j6=j6+sample_dt)
                        {
                            for(float j7 = (*joint_bounds[6])[0].min_position_; j7 < (*joint_bounds[6])[0].max_position_; j7=j7+sample_dt)
                            {
                                for(float j8 = (*joint_bounds[7])[0].min_position_; j8 < (*joint_bounds[7])[0].max_position_; j8=j8+sample_dt)
                                {
                                    for(float j9 = (*joint_bounds[8])[0].min_position_; j9 < (*joint_bounds[8])[0].max_position_; j9=j9+sample_dt)
                                    {
                                        for(float j10 = (*joint_bounds[9])[0].min_position_; j10 < (*joint_bounds[9])[0].max_position_;j10=j10+sample_dt)
                                        {
                                            joint_values[0] = j1;
                                            joint_values[1] = j2;
                                            joint_values[2] = j3;
                                            joint_values[3] = j4;
                                            joint_values[4] = j5;
                                            joint_values[5] = j6;
                                            joint_values[6] = j7;
                                            joint_values[7] = j8;
                                            joint_values[8] = j9;
                                            joint_values[9] = j10;

                                            //print_status(z, total);
                                            z++;

                                            // kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
                                            // kinematic_state->update(true);
                                            // kinematic_state->updateCollisionBodyTransforms();

                                            current_state = planning_scene.getCurrentStateNonConst();
                                            current_state.setJointGroupPositions(joint_model_group, joint_values);
                                            // current_state.setToRandomPositions();
                                            // ROS_INFO_STREAM("Current state is "<< (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));
                                            collision_request.contacts = true;
                                            collision_request.max_contacts = 1000;

                                            // check the slef collision
                                            collision_result.clear();
                                            planning_scene.checkSelfCollision(collision_request, collision_result);
                                            //  ROS_INFO_STREAM("Current state is "<< (collision_result.collision ? "in" : "not in")
                                            // << " self collision");
                                            collision_detection::CollisionResult::ContactMap::const_iterator it;
                                            for(it = collision_result.contacts.begin();
                                                it != collision_result.contacts.end();
                                                ++it)
                                            {
                                                // ROS_INFO("Contact between: %s and %s",it->first.first.c_str(),it->first.second.c_str());
                                            }

                                            std::vector<double> joint_values;
                                            current_state.copyJointGroupPositions(joint_model_group, joint_values);
                                            for(std::size_t i = 0; i < jointCount; ++i)
                                            {
                                                //  ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
                                            }

                                            // if there is no self collision
                                            //                                            if (collision_result.collision != true)
                                            //                                            {
                                            // get the transform and update the rechability map
                                            const Eigen::Affine3d &end_effector_state = current_state.getGlobalLinkTransform("leftPalm"); // kinematic_state->getGlobalLinkTransform("leftPalm");

                                            geometry_msgs::Point p;
                                            // Print end-effector pose. Remember that this is in the model frame
                                            // ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
                                            //   ROS_INFO("Valid State");

                                            p.x = end_effector_state.translation()[0];
                                            p.y = end_effector_state.translation()[1];
                                            p.z = end_effector_state.translation()[2];

                                            if (collision_result.collision != true)
                                            {
                                                fprintf(yaml, "map: %s\nresolution: %f\nself collision: %s\npoints: [%f, %f, %f]\n\n", filename.c_str(), sample_dt, "false", p.x, p.y, p.z);
                                                marker.points.push_back(p);
                                            }
                                            else
                                            {
                                                fprintf(yaml, "map: %s\nresolution: %f\nself collision: %s\npoints: [%f, %f, %f]\n\n", filename.c_str(), sample_dt, "true", p.x, p.y, p.z);
                                                marker_no.points.push_back(p);
                                                //      ROS_WARN("collison");
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // close the files
    fclose(yaml);
    H5Fclose(file);
    H5Gclose(group_poses);

    ROS_INFO("Done\n");
    // publish the markers
    vis_pub.publish(marker);
    vis_pub_no.publish(marker_no);

    return 0;
}


//void updateJointValues (float* jointval)

//{
//    for (i=0;i<jointCount; i++)
//    {
//        joint_values[i] = jointval[i];
//    }
//}





void print_status(long current_count, double total)
{
    double Y=(current_count/total)*100;
    //ROS_INFO("%ld %lf",current_count, total);

    if (Y>0 && Y<10) std::cout << "Completed    " << Y << " %-" << std::endl;
    if (Y>10 && Y<20) std::cout << "Completed   "<<Y<< " %--" << std::endl;
    if (Y>20 && Y<30) std::cout << "Completed   "<<Y<< " %---" << std::endl;
    if (Y>30 && Y<40) std::cout << "Completed   "<<Y<< " %----" << std::endl;
    if (Y>40 && Y<50) std::cout << "Completed   "<<Y<< " %-----" << std::endl;
    if (Y>50 && Y<60) std::cout << "Completed   "<<Y<< " %------" << std::endl;
    if (Y>60 && Y<70) std::cout << "Completed   "<<Y<< " %--------" << std::endl;
    if (Y>70 && Y<80) std::cout << "Completed   "<<Y<< " %---------" << std::endl;
    if (Y>80 && Y<90) std::cout << "Completed   "<<Y<< " %----------" << std::endl;
    if (Y>90) std::cout << "Completed  "<<Y<< " %----------" << std::endl;
}

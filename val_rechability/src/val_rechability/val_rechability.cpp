#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

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

    // instantiate planning scene
    planning_scene::PlanningScene planning_scene(kinematic_model);

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

    ros::NodeHandle n1;
    ros::Publisher vis_pub = n1.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

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
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0);

    float sample_dt = 0.6;

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

                                            kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

                                            // check the slef collision
                                            collision_result.clear();
                                            planning_scene.checkSelfCollision(collision_request, collision_result);

                                            // if there is no self collision
                                            if (collision_result.collision != true)
                                            {
                                                // get the transform and update the rechability map
                                                const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("leftPalm");

                                                geometry_msgs::Point p;
                                                // Print end-effector pose. Remember that this is in the model frame
                                                ROS_INFO_STREAM("Translation: " << end_effector_state.translation());


                                                p.x = end_effector_state.translation()[0];
                                                p.y = end_effector_state.translation()[1];
                                                p.z = end_effector_state.translation()[2];

                                                marker.points.push_back(p);
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

    // publish the markers
    vis_pub.publish(marker);

    return 0;
}


//void updateJointValues (float* jointval)

//{
//    for (i=0;i<jointCount; i++)
//    {
//        joint_values[i] = jointval[i];
//    }
//}


/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#include <cstdio>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "robot_self_filter/self_mask.h"
#include "val_common/val_common_names.h"


class val_self_filter
{
public:

    val_self_filter(void)
    {
        id_ = 1;
        vmPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
        vmOutputPub_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("assembled_filtered_cloud2", 10);
        vmSub_ = nodeHandle_.subscribe("assembled_cloud2",100, &val_self_filter::run, this);
        std::vector<robot_self_filter::LinkInfo> links;

        if (!nodeHandle_.hasParam("self_see_links")){
            robot_self_filter::LinkInfo li;
            li.name="base_link";
            li.padding = .05f;
            li.scale = 1.0f;
            links.push_back(li);
        }
        else {
            std::vector<std::string> ssl_vals;
            nodeHandle_.getParam("self_see_links", ssl_vals);

            if(ssl_vals.size() == 0) {
                ROS_WARN("Self see links need to be an array with size >=1");

            }
            else {
                for(int i = 0; i < ssl_vals.size(); i++) {
                    robot_self_filter::LinkInfo li;
                    li.name = ssl_vals.at(i);
                    li.padding = .05f;
                    li.scale = 1.0f;
                    links.push_back(li);
                }
            }
        }

        sf_ = new robot_self_filter::SelfMask<pcl::PointXYZ>(tf_, links);
    }

    ~val_self_filter(void)
    {
        delete sf_;
    }

    void gotIntersection(const tf::Vector3 &pt)
    {
        sendPoint(pt.x(), pt.y(), pt.z());
    }

    void sendPoint(double x, double y, double z)
    {
        pcl::PointXYZ pt(x, y, z);
        maskCloud_.insert(maskCloud_.end(), pt);
        visualization_msgs::Marker mk;

        mk.header.stamp = ros::Time::now();

        mk.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;

        mk.ns = "";
        mk.id = id_++;
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = x;
        mk.pose.position.y = y;
        mk.pose.position.z = z;
        mk.pose.orientation.w = 1.0;

        mk.scale.x = mk.scale.y = mk.scale.z = 0.01;

        mk.color.a = 1.0;
        mk.color.r = 1.0;
        mk.color.g = 0.04;
        mk.color.b = 0.04;

        mk.lifetime = ros::Duration(10);

        vmPub_.publish(mk);
    }

    void run(sensor_msgs::PointCloud2::ConstPtr msg)
    {

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*in);
        ROS_INFO("Size of pointcloud : %d ", in->size());

        std::vector<int> mask;
        sf_->maskIntersection(*in, VAL_COMMON_NAMES::HOKUYO_LINK_TF, 0.01, mask, boost::bind(&val_self_filter::gotIntersection, this, _1) );
//            sf_->maskContainment(*in, mask);

//        assert(mask.size() == in->size());
//        for (unsigned int i = 0 ; i < mask.size() ; ++i)
//        {
//            if (mask[i] != robot_self_filter::INSIDE) {
//                 in->points.erase(in->points.begin() + i);
//                 in->erase(in->begin() + i);
//                 sendPoint(in->points[i].x, in->points[i].y, in->points[i].z);
//            }
//        }

        sensor_msgs::PointCloud2 cloud2;
        pcl::toPCLPointCloud2(maskCloud_, pcl_pc2);
        pcl_conversions::moveFromPCL(pcl_pc2, cloud2);
        cloud2.header.frame_id.assign(in->header.frame_id);
        cloud2.header.stamp = ros::Time::now();
        ROS_INFO("Size of filtered pointcloud: %d", in->size());
        vmOutputPub_.publish(cloud2);
//        ros::spin();
    }

protected:

    double uniform(double magnitude)
    {
        return (2.0 * drand48() - 1.0) * magnitude;
    }

    tf::TransformListener                           tf_;
    robot_self_filter::SelfMask<pcl::PointXYZ>      *sf_;
    ros::Publisher                                  vmPub_;
    ros::Publisher                                  vmOutputPub_;
    ros::Subscriber                                 vmSub_;
    ros::NodeHandle                                 nodeHandle_;
    pcl::PointCloud<pcl::PointXYZ>                  maskCloud_;

    int                                             id_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "val_self_filter");

    val_self_filter t;
    ros::spin();
//    t.run();

    return 0;
}

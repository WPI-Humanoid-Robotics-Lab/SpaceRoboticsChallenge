#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;


public:
    static int index;
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/multisense/camera/left/image_rect_color", 1,
                                   &ImageConverter::imageCb, this);

    }

    ~ImageConverter()
    {
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            std::string filename = ros::package::getPath("val_task1") + "/models/images/" + "training_img_" + std::to_string(index++) + ".png";
            cv::imwrite(filename, cv_ptr->image);

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

    }
};
int ImageConverter::index = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_training_images");
    ImageConverter::index = int(ros::WallTime::now().toSec());
    ImageConverter ic;
    ros::spin();
    return 0;
}


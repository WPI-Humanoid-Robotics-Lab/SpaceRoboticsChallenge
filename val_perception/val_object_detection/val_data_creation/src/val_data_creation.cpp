#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <boost/bind.hpp>
#include <tf/transform_listener.h>

int count = 0;
cv::Mat left_image, right_image;
bool left_updated, right_updated;
tf::StampedTransform transform;
/*
static void write(cv::FileStorage& fs, const std::string&, const tf::StampedTransform& x)
{
    //fs << "x" << x.getOrigin().x();
    //fs << "y" << x.getOrigin().y();
    x.write(fs);
}*/

void writeToFile()
{
  std::stringstream filename;
  filename<<ros::package::getPath("val_data_creation")<<"/saved_images/"<<"training_img_"<<count<<".xml";
  cv::FileStorage fs(filename.str(), cv::FileStorage::WRITE);
  fs << "left" << left_image;
  fs << "right" << right_image;
  fs << "xpos" << transform.getOrigin().x();
  fs << "ypos" << transform.getOrigin().y();
  count++;
  fs.release();
  left_updated = right_updated = false;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, char side)
{
  cv_bridge::CvImagePtr cv_ptr;
  if (side == 'L')
  {
    left_updated = true;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    left_image = cv_ptr->image;
  }
  else if (side == 'R')
  {
    right_updated = true;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    right_image = cv_ptr->image;
  }

  if (left_updated == true && right_updated == true)
    {writeToFile();}
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();

  bool success = false;
  tf::TransformListener listener;
  
  while (!success) {
    try {
      listener.waitForTransform("/valve_frame", "/pelvis", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/valve_frame", "/pelvis", ros::Time(0), transform);
      success = true;
    }
    catch (tf::LookupException e){

    }
    sleep(0.1);
  }

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber subL = it.subscribe("/multisense/camera/left/image_raw", 1, boost::bind(imageCallback, _1, 'L'));
  image_transport::Subscriber subR = it.subscribe("/multisense/camera/right/image_raw", 1, boost::bind(imageCallback, _1, 'R'));
  ros::spin();
  cv::destroyWindow("view");
}



//-------------------------
/*
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
        image_sub_ = it_.subscribe("/multisense/camera/left/image_raw", 1,
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
            //std::string filename = ros::package::getPath("val_data_creation") + "/saved_images/" + "training_img_" + std::to_string(index++) + ".png";
            std::stringstream filename;
            filename<<ros::package::getPath("val_data_creation")<<"/saved_images/"<<"training_img_"<<index<<".json";
            cv::imwrite(filename.str(), cv_ptr->image);

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
} */

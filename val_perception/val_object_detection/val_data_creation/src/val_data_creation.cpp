#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <boost/bind.hpp>
#include <tf/transform_listener.h>

// too many header files ^

int count = 0;
cv::Mat left_image, right_image;
bool left_updated, right_updated;
tf::StampedTransform transform;

// /valve_frame is at (2.15 0.60 0 0 0 0 1 ) with respect to /world
// This node is calculating the transform from /pelvis to /valve_frame

void writeToFile()
{
  // create file names
  std::stringstream filenameL, filenameR, filetext;
  filenameL<<ros::package::getPath("val_data_creation")<<"/left_images/"<<"L_"<<count<<".png";
  filenameR<<ros::package::getPath("val_data_creation")<<"/right_images/"<<"R_"<<count<<".png";
  filetext<<ros::package::getPath("val_data_creation")<<"/labels/"<<"Label_"<<count<<".txt";
  cv::imwrite(filenameL.str(),left_image);
  cv::imwrite(filenameR.str(),right_image);
  std::ofstream fs;
  fs.open(filetext.str().c_str());          // save transform to text file
  fs << "xpos" << transform.getOrigin().x()<<"\n";
  fs << "ypos" << transform.getOrigin().y()<<"\n";
  fs << "zpos" << transform.getOrigin().z()<<"\n";
  fs << "q1" << transform.getRotation()[1]<<"\n";
  fs << "q2" << transform.getRotation()[2]<<"\n";
  fs << "q3" << transform.getRotation()[3]<<"\n";
  count++;
  fs.close();
  left_updated = right_updated = false;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, char side)
{
  cv_bridge::CvImagePtr cv_ptr;
  if (side == 'L')
  {
    left_updated = true;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    left_image = cv_ptr->image; // convert msg to image
  }
  else if (side == 'R')
  {
    right_updated = true;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    right_image = cv_ptr->image;
  }

  if (left_updated == true && right_updated == true)
    { writeToFile(); }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

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
    sleep(0.1); // not sure if required
  }

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber subL = it.subscribe("/multisense/camera/left/image_raw", 1, boost::bind(imageCallback, _1, 'L'));
  image_transport::Subscriber subR = it.subscribe("/multisense/camera/right/image_raw", 1, boost::bind(imageCallback, _1, 'R'));
  ros::spin();

}

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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// too many header files ^

int count = 0;
cv::Mat left_image, right_image;
tf::StampedTransform transform;
tf::TransformListener listener;
bool success = false;

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
  std::ofstream fs;
  fs.open(filetext.str().c_str());          // save transform to text file
  fs << "xpos " << transform.getOrigin().x()<<"\n";
  fs << "ypos " << transform.getOrigin().y()<<"\n";
  fs << "zpos " << transform.getOrigin().z()<<"\n";
  fs << "q1 " << transform.getRotation()[1]<<"\n";
  fs << "q2 " << transform.getRotation()[2]<<"\n";
  fs << "q3 " << transform.getRotation()[3]<<"\n";
  count++;
  fs.close();

}

void imageCallback(const sensor_msgs::ImageConstPtr& msgL, const sensor_msgs::ImageConstPtr& msgR)
{
  cv_bridge::CvImagePtr cv_ptr;

  cv_ptr = cv_bridge::toCvCopy(msgL, sensor_msgs::image_encodings::BGR8);
  left_image = cv_ptr->image; // convert msg to image

  cv_ptr = cv_bridge::toCvCopy(msgR, sensor_msgs::image_encodings::BGR8);
  right_image = cv_ptr->image;

  writeToFile();



}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> subL(nh, "/multisense/camera/left/image_raw", 1); //boost::bind(imageCallback, _1, 'L')
  message_filters::Subscriber<sensor_msgs::Image> subR(nh, "/multisense/camera/right/image_raw", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(subL, subR, 10);
  sync.registerCallback(boost::bind(&imageCallback, _1, _2));

  ros::spin();

}

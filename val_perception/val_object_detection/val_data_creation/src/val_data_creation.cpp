#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <fstream>
#include <boost/bind.hpp>

#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>


int count = int(ros::WallTime::now().toSec());
cv::Mat left_image, right_image, composed_image;
bool left_updated, right_updated;
tf::StampedTransform transform;

// /valve_frame is at (2.15 0.60 0 0 0 0 1 ) with respect to /world
// This node is calculating the transform from /pelvis to /valve_frame

void writeToFile() //std_msgs::Header h
{
  // create file names
  std::stringstream filenameL, filenameR, filenameI, filetext;

//  filenameL<<ros::package::getPath("val_data_creation")<<"/left_images/"<<"L_"<<count<<".png";
//  filenameR<<ros::package::getPath("val_data_creation")<<"/right_images/"<<"R_"<<count<<".png";
  filenameI<<ros::package::getPath("val_data_creation")<<"/images/"<<count<<".png";
  filetext<<ros::package::getPath("val_data_creation")<<"/labels/"<<count<<".txt";

//  cv::imwrite(filenameL.str(),left_image);
//  cv::imwrite(filenameR.str(),right_image);
  cv::imwrite(filenameI.str(),composed_image);

  bool success = false;
  tf::TransformListener listener;

  while (!success) {
    try {
      listener.waitForTransform("/valve_frame", "/pelvis", ros::Time(0), ros::Duration(2.0));
      listener.lookupTransform("/valve_frame", "/pelvis", ros::Time(0), transform); //h.stamp to get image's timestamp
      success = true;
    }
    catch (tf::LookupException e){}
  }

  std::ofstream fs;
  fs.open(filetext.str().c_str());          // save transform to text file
  // table 0 0 0 761.0 1.0 1024.0 153.0 1 1 1 0 0 0 0 1
  // fs << "xpos" << transform.getOrigin().x()<<"\n";
  // fs << "ypos" << transform.getOrigin().y()<<"\n";
  // fs << "zpos" << transform.getOrigin().z()<<"\n";
  // fs << "q1" << transform.getRotation()[1]<<"\n";
  // fs << "q2" << transform.getRotation()[2]<<"\n";
  // fs << "q3" << transform.getRotation()[3]<<"\n";
  fs << "table " << transform.getOrigin().x() << " ";
  fs << transform.getOrigin().y() << " ";
  fs << transform.getOrigin().z() << " ";
  fs << transform.getRotation()[1] << " ";
  fs << transform.getRotation()[2] << " ";
  fs << transform.getRotation()[3];
  count++;

  fs.close();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msgL, const sensor_msgs::ImageConstPtr& msgR )
{
  cv_bridge::CvImagePtr cv_ptr;

  cv_ptr = cv_bridge::toCvCopy(msgL, sensor_msgs::image_encodings::BGR8);
  left_image = cv_ptr->image; // convert msg to image

  cv_ptr = cv_bridge::toCvCopy(msgR, sensor_msgs::image_encodings::BGR8);
  right_image = cv_ptr->image;

  cv::hconcat(left_image, right_image, composed_image);
  //std_msgs::Header h = msgR->header;

  writeToFile();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter itfilterL, itfilterR;

  itfilterL.subscribe(it,"/multisense/camera/left/image_raw", 1);
  itfilterR.subscribe(it,"/multisense/camera/right/image_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> mysync;

  message_filters::Synchronizer<mysync> sync(mysync(10),itfilterL,itfilterR);
  sync.registerCallback(boost::bind(&imageCallback, _1, _2));

  ros::spin();
}

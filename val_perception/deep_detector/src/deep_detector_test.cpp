#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "DetectNet.h"

const std::string RECEIVE_IMG_TOPIC_NAME = "/multisense/camera/left/image_rect_color";
const std::string PUBLISH_RET_TOPIC_NAME = "/caffe_ret";
const std::string PUBLISH_IMAGE_TOPIC_NAME = "/detectnet/image/table";

DetectNet* detector;
std::string model_path;
std::string weights_path;
std::string mean_file;
std::string image_path;

ros::Publisher gPublisher;
image_transport::Publisher imagePublisher;

void publishRet(const std::vector<int>& predictions);

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr;
        // ROS_INFO("Image encoding : %s",msg->encoding.c_str());
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        //cv::imwrite("rgb.png", cv_ptr->image);
		cv::Mat img = cv_ptr->image;
        if(img.empty()){
            ROS_INFO("Image is empty");
            return;
        }
		
        std::vector<int> predictions = detector->Detect(img);
        sensor_msgs::ImagePtr msgImage;
        // top left
        cv::Point pt1(predictions[0], predictions[1]);
        // bottom right corner.
        cv::Point pt2(predictions[2], predictions[3]);
        cv::rectangle(img, pt1, pt2, cv::Scalar(0, 255, 0),3);


        msgImage = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img).toImageMsg();
        imagePublisher.publish(msgImage);
        publishRet(predictions);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
}

void publishRet(const std::vector<int>& predictions)  {
    std_msgs::String msg;
    std::stringstream ss;
    for (size_t i = 0; i < predictions.size(); ++i) {
        int p = predictions[i];
        ss << "[" << p << "]" << std::endl;
        // 
    }
    msg.data = ss.str();
    gPublisher.publish(msg);
}

void publishImage(const std::vector<int>& predictions)  {
    for (size_t i = 0; i < predictions.size(); ++i) {
        int p = predictions[i];
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "deep_detector_test");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    const std::string ROOT_SAMPLE = ros::package::getPath("deep_detector");
    model_path = ROOT_SAMPLE + "/data/deploy.prototxt";
    weights_path = ROOT_SAMPLE + "/data/snapshot_iter_1110.caffemodel";
    mean_file = ROOT_SAMPLE + "/data/mean.binaryproto";
    detector = new DetectNet(model_path, weights_path, mean_file);

    // To receive an image from the topic, PUBLISH_RET_TOPIC_NAME
    image_transport::Subscriber sub = it.subscribe(RECEIVE_IMG_TOPIC_NAME, 1, imageCallback);
	

    gPublisher = nh.advertise<std_msgs::String>(PUBLISH_RET_TOPIC_NAME, 100);
    imagePublisher = it.advertise(PUBLISH_IMAGE_TOPIC_NAME, 100);
    

    // weights_path = ROOT_SAMPLE + "/data/bvlc_reference_caffenet.caffemodel";
    // mean_file = ROOT_SAMPLE + "/data/imagenet_mean.binaryproto";
    //label_file = ROOT_SAMPLE + "/data/backup/synset_words.txt";
    // label_file = ROOT_SAMPLE + "/data/synset_words.txt";
    //--- THIS WORKS
    // image_path = ROOT_SAMPLE + "/data/wasabi.png";
    // detector = new DetectNet(model_path, weights_path, mean_file);

    ros::spin();
    delete detector;
    ros::shutdown();
    return 0;
}

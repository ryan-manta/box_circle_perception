/*  
    This header lays out the master perception pipeline running on the
    Green Pick station. It handles conversion from ROS Image msgs to OpenCV
    images and initiates pickpoint generation or item calibration depending
    on the station's state machine

    Ted Lutkus
    6/26/20
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include "detection_algos/hough_circle_detector.cpp"
#include "detection_algos/cluster_featurematch_box_detector.cpp"
#include "green_pick/GeneratePickpoint.h"

#include <string>

#define BOX 0
#define CIRCLE 1

class PerceptionPipeline {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transporter;
    image_transport::Subscriber image_subscriber;
    image_transport::Publisher image_publisher;
    cv_bridge::CvImagePtr cv_ptr;

    // Construct image transporter after ros node is initialized
public:
    PerceptionPipeline();
  
    // Callback function for the image transport ROS node
    void perception_callback(const sensor_msgs::ImageConstPtr& msg);

    // Generate pickpoint service callback
    bool generate_pickpoint(green_pick::GeneratePickpoint::Request &req,
                        green_pick::GeneratePickpoint::Response &res);
                        
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_pipeline");
    PerceptionPipeline greenpick_perception_pipeline;

    ros::init(argc, argv, "pickpoint_generator_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("generate_pickpoint", &PerceptionPipeline::generate_pickpoint, &greenpick_perception_pipeline);
    ROS_INFO("Pick service ready for calling.");
    
    ros::spin();
    return 0;
}
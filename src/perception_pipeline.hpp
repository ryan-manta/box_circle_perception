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

#include "state_machine.hpp"
#include <iostream>

class PerceptionPipeline {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transporter;
    image_transport::Subscriber image_subscriber;
    image_transport::Publisher image_publisher;

    // Internal reference to state machine that's controlled by external script
    StateMachine& state_machine;

public:
    PerceptionPipeline(StateMachine& state_machine);
  
    // Callback function for the image transport ROS node
    void perception_callback(const sensor_msgs::ImageConstPtr& msg);

    ~PerceptionPipeline();
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_pipeline");
    StateMachine perception_state_machine = StateMachine();
    PerceptionPipeline greenpick_perception_pipeline(perception_state_machine);
    ros::spin();
    return 0;
}
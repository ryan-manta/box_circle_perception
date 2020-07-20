/* 
    This script can be run to calibrate a new rectangular item
    for 2D segmentation in Green Pick. The user will run the script,
    be presented a gui with an image from our sensor, select the box,
    and have any necessary feature data stored 

    Ted Lutkus
    6/26/20
*/

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include "ros/ros.h"

static const std::string OPENCV_WINDOW = "Image window";

class Calibrator {
private:
    //ros::NodeHandle nh_;
    image_transport::ImageTransport image_transporter;
    image_transport::Subscriber image_subscriber;
    bool do_calibration;

public:
    Calibrator(ros::NodeHandle n_converter);

    static void onMouse(int event, int x, int y, int, void* ptr);

    void calibrate_box_item(cv::Mat& source_image);

    void image_converter_callback(const sensor_msgs::ImageConstPtr& msg);

    ~Calibrator();
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "calibrator_image_converter");
    ros::NodeHandle n_converter;
    Calibrator calibrator(n_converter);

    ros::spin();
    return 0;
}
/* 
    This script can be run to calibrate perception algorithms
    for 2D segmentation in Green Pick. The user will run the script,
    be presented a gui with a stream from our camera, and be able to adjust
    sliders for algorithms supported:
    - surf feature map box detection

    Ted Lutkus
    6/30/20
*/

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
//#include <opencv4/opencv2/highgui.hpp>
#include "ros/ros.h"
#include "detection_algos/cluster_featurematch_box_detector.cpp"

static const std::string OPENCV_WINDOW = "Image window";

int hessian_slider = 100;
int hessian_slider_max = 1000;
int hessian = 100;

int K_slider = 20;
int K_max = 24;
int K = 20;

class AlgoCalibrator {
private:
    image_transport::ImageTransport image_transporter;
    image_transport::Subscriber image_subscriber;
    cv_bridge::CvImagePtr cv_ptr;
    bool do_calibration;

public:
    AlgoCalibrator(ros::NodeHandle n_converter);

    static void on_trackbar_hessian(int, void* ptr);

    static void on_trackbar_kmeans(int, void* ptr);

    void image_converter_callback(const sensor_msgs::ImageConstPtr& msg);

    ~AlgoCalibrator();
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "algorithm_calibrator");
    ros::NodeHandle n_converter;
    AlgoCalibrator AlgoCalibrator(n_converter);

    ros::spin();
    return 0;
}
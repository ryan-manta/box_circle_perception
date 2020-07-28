/* 
    This script can be run to calibrate perception algorithms
    for 2D segmentation in Green Pick. The user will run the script,
    be presented a gui with a stream from our camera, and be able to adjust
    sliders for algorithms supported:
    - cluster feature map box detection
    - hough circle detector

    Ted Lutkus
    6/30/20
*/

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
//#include <opencv4/opencv2/highgui.hpp>
#include "ros/ros.h"
#include "cluster_featurematch_box_detector.hpp"
#include "hough_circle_detector.hpp"
#include <string>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";

#define BOX 1
#define CIRCLE 2

class AlgoCalibrator {
private:
    image_transport::ImageTransport image_transporter;
    image_transport::Subscriber image_subscriber;
    cv_bridge::CvImagePtr cv_ptr;
    int box_or_circle_algo;

public:
    AlgoCalibrator(ros::NodeHandle n_converter, int box_or_circle_algo);

    /* BOX DETECTION PARAMETERS */
    static void on_trackbar_hessian(int, void* ptr);
    static void on_trackbar_kmeans(int, void* ptr);
    static void on_trackbar_parallel_angle_threshold(int, void* ptr);
    static void on_trackbar_min_parallelogram_edge_length(int, void* ptr);
    static void on_trackbar_right_angle_threshold(int, void* ptr);

    /* CIRCLE DETECTOR PARAMETERS */
    static void on_trackbar_circle_dist(int, void* ptr);
    static void on_trackbar_canny_thresh(int, void* ptr);
    static void on_trackbar_hough_thresh(int, void* ptr);
    static void on_trackbar_circle_radius(int, void* ptr);
    static void on_trackbar_radius_tolerance(int, void* ptr);

    void image_converter_callback(const sensor_msgs::ImageConstPtr& msg);

    ~AlgoCalibrator();
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "algorithm_calibrator");
    ros::NodeHandle n_converter;

    // Loop until user inputs valid input of box or circle, then start the algorithm calibrator for the specified algo
    bool valid_input = false;
    std::string chosen_algo;
    int box_or_circle_algo = 0;
    while (!valid_input) {
        std::cout << "Please enter the desired algo calibration ('box' or 'circle'): ";
        std::cin >> chosen_algo;
        if (chosen_algo.compare("box") == 0) {
            box_or_circle_algo = BOX;
            valid_input = true;
        } else if (chosen_algo.compare("circle") == 0) {
            box_or_circle_algo = CIRCLE;
            valid_input = true;
        } else {
            std::cout << "\nInvalid input!\n";
        }
    }

    AlgoCalibrator AlgoCalibrator(n_converter, box_or_circle_algo);

    ros::spin();
    return 0;
}
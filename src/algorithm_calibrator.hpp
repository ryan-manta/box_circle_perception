/*
 *  This node can be run to calibrate perception algorithms for 2D segmentation and pickpoint generation.
 *
 *  To use:
 *  1. Start the algorithm_calibrator node using rosrun and type to specify whether the 'box' detector or 'circle' detector is being used.
 *  2. A frame from the camera will be presented:
 *      - Select the region of interest for the algorithms to be calibrated on by clicking and dragging on the frame.
 *      - A blue box should appear while dragging to show the selected region of interest.
 *  3. After confirming your selection by pressing the space key or enter key, the chosen algorithm will be performed continuously on the
 *     selected region of interest.
 *  4. Use the GUI sliders to adjust parameters and see what works best for the target objects
 *
 *  Ted Lutkus
 *  6/30/20
 */

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include "ros/ros.h"
#include "cluster_featurematch_box_detector.hpp"
#include "hough_circle_detector.hpp"
#include <string>
#include <iostream>
#include <sqlite3.h>

static const std::string OPENCV_WINDOW = "Algorithm Calibrator";

#define BOX		  1
#define CIRCLE	  2

class AlgoCalibrator {
private:
    image_transport::ImageTransport image_transporter;
    image_transport::Subscriber image_subscriber;
    cv_bridge::CvImagePtr cv_ptr;
    int box_or_circle_algo;   // algorithm to calibrate
    std::string grocery_item; //grocery item being calibrated

    // Variables for selecting calibration region of interest
    bool first_time = true;
    cv::Rect2d selected_rectangle;

public:
    AlgoCalibrator(ros::NodeHandle n_converter, int box_or_circle_algo,
                   std::string grocery_item);

    /* BOX DETECTION PARAMETERS */
    static void on_trackbar_hessian(int, void *ptr);
    static void on_trackbar_kmeans(int, void *ptr);
    static void on_trackbar_parallel_angle_threshold(int, void *ptr);
    static void on_trackbar_min_parallelogram_edge_length(int, void *ptr);
    static void on_trackbar_right_angle_threshold(int, void *ptr);

    /* CIRCLE DETECTOR PARAMETERS */
    static void on_trackbar_circle_dist(int, void *ptr);
    static void on_trackbar_canny_thresh(int, void *ptr);
    static void on_trackbar_hough_thresh(int, void *ptr);
    static void on_trackbar_circle_radius(int, void *ptr);
    static void on_trackbar_radius_tolerance(int, void *ptr);

    // Callback for each camera frame
    void image_converter_callback(const sensor_msgs::ImageConstPtr& msg);

    // Callback to print Grocery Item DB Entry Data
    static void callbackSQLPrint(void *data, int argc, char **argv, char **azColName);

    ~AlgoCalibrator();
};

int main(int argc, char **argv) {
    // Initialize the calibrator node
    ros::init(argc, argv, "algorithm_calibrator");
    ros::NodeHandle n_converter;

    /* Loop until user inputs valid input of box or circle, then start the
     * algorithm calibrator for the specified algo */
    std::string chosen_algo;
    std::string grocery_item;
    std::cout << "Please enter the grocery item being calibrated (i.e. 'penne_pasta'): ";
    std::getline(std::cin, grocery_item);
    int box_or_circle_algo = 0;
    std::cout << endl << "Please enter the desired algo calibration ('box' or 'circle'): ";
    std::getline(std::cin, chosen_algo);

    if (chosen_algo.compare("box") == 0) {
        box_or_circle_algo = BOX;
        //Run box item calibrator??
    } else if (chosen_algo.compare("circle") == 0) {
        box_or_circle_algo = CIRCLE;
    } else {
        std::cout << "\nInvalid input!\n";
        return (0);
    }

    // Create the algorithm calibrator object using the desired algorithm
    AlgoCalibrator AlgoCalibrator(n_converter, box_or_circle_algo, grocery_item);

    ros::spin();

    return (0);
}

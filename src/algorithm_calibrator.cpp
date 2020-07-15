/* 
    This script can be run to calibrate perception algorithms
    for 2D segmentation in Green Pick. The user will run the script,
    be presented a gui with a stream from our camera, and be able to adjust
    sliders for algorithms supported:
    - hough circle detection
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
    AlgoCalibrator(ros::NodeHandle n_converter) : image_transporter(n_converter) {
        image_subscriber = image_transporter.subscribe("/camera/color/image_raw", 1,
            &AlgoCalibrator::image_converter_callback, this);

        // Set calibration to false at start
        do_calibration = false;

        // OpenCV window
        cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);

        // SURF Hessian threshold trackbar
        char TrackbarName_hessian[50];
        std::sprintf(TrackbarName_hessian, "Hessian x %d", hessian_slider_max);
        cv::createTrackbar( TrackbarName_hessian, OPENCV_WINDOW, &hessian_slider, hessian_slider_max, &AlgoCalibrator::on_trackbar_hessian, this);

        // SURF Hessian threshold trackbar
        char TrackbarName_kmeans[50];
        std::sprintf(TrackbarName_kmeans, "K-Means x %d", K_max);
        cv::createTrackbar( TrackbarName_kmeans, OPENCV_WINDOW, &K_slider, K_max, &AlgoCalibrator::on_trackbar_kmeans, this);
    }

    static void on_trackbar_hessian(int, void* ptr)
        {
            // Adjust hessian value
            AlgoCalibrator* this_c = (AlgoCalibrator*) ptr;
            hessian = hessian_slider;
            ROS_INFO("hessian: %u", hessian);
        }

    static void on_trackbar_kmeans(int, void* ptr)
        {
            // Adjust hessian value
            AlgoCalibrator* this_c = (AlgoCalibrator*) ptr;
            K = K_slider;
            ROS_INFO("K: %u", K);
        }

    void image_converter_callback(const sensor_msgs::ImageConstPtr& msg) {
        // Try to copy ROS image to OpenCV image
        try {
            this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Check if image has data
        if (!this->cv_ptr->image.data)
        {
            ROS_ERROR("No image data found!");
            return;
        }

        // Detect box using new hessian threshold
        //cv::Mat matches_image;
        bool show_feature_matches = true;
        std::vector<cv::Point2f> pickpoints_xy;
        try {
            detect_boxes(pickpoints_xy, this->cv_ptr->image, hessian, K, show_feature_matches);
        } catch (cv::Exception e) {
            ROS_INFO("Segmentation failed!");
        }

        // Show modified image
        cv::imshow(OPENCV_WINDOW, this->cv_ptr->image);
        cv::waitKey(500);
        //cv::waitKey(1000);
    }

    ~AlgoCalibrator() {
        cv::destroyWindow(OPENCV_WINDOW);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "algorithm_calibrator");
    ros::NodeHandle n_converter;
    AlgoCalibrator AlgoCalibrator(n_converter);

    ros::spin();
    return 0;
}
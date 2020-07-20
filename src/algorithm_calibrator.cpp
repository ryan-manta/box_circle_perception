#include "algorithm_calibrator.hpp"

AlgoCalibrator::AlgoCalibrator(ros::NodeHandle n_converter) : image_transporter(n_converter) {
    image_subscriber = image_transporter.subscribe("/camera/color/image_raw", 1,
                                                   &AlgoCalibrator::image_converter_callback, this);

    // Set calibration to false at start
    do_calibration = false;

    // OpenCV window
    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);

    // SURF Hessian threshold trackbar
    char TrackbarName_hessian[50];
    std::sprintf(TrackbarName_hessian, "Hessian x %d", hessian_slider_max);
    cv::createTrackbar(TrackbarName_hessian, OPENCV_WINDOW, &hessian_slider, hessian_slider_max, &AlgoCalibrator::on_trackbar_hessian, this);

    // SURF Hessian threshold trackbar
    char TrackbarName_kmeans[50];
    std::sprintf(TrackbarName_kmeans, "K-Means x %d", K_max);
    cv::createTrackbar(TrackbarName_kmeans, OPENCV_WINDOW, &K_slider, K_max, &AlgoCalibrator::on_trackbar_kmeans, this);
}

void AlgoCalibrator::on_trackbar_hessian(int, void *ptr) {
    // Adjust hessian value
    AlgoCalibrator *this_c = (AlgoCalibrator *)ptr;
    hessian = hessian_slider;
    ROS_INFO("hessian: %u", hessian);
}

void AlgoCalibrator::on_trackbar_kmeans(int, void *ptr) {
    // Adjust hessian value
    AlgoCalibrator *this_c = (AlgoCalibrator *)ptr;
    K = K_slider;
    ROS_INFO("K: %u", K);
}

void AlgoCalibrator::image_converter_callback(const sensor_msgs::ImageConstPtr &msg) {
    // Try to copy ROS image to OpenCV image
    try {
        this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Check if image has data
    if (!this->cv_ptr->image.data) {
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

AlgoCalibrator::~AlgoCalibrator() {
    cv::destroyWindow(OPENCV_WINDOW);
}
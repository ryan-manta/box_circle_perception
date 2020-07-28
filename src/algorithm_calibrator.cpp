#include "algorithm_calibrator.hpp"

/* BOX DETECTOR PARAMETERS */
// Hessian value used for feature detection
int hessian_slider = 100;
int hessian_slider_max = 1000;
int hessian = 100;

// Number of clusters to run segmentation on in image (need at least one for each object)
int K_slider = 20;
int K_max = 100;
int K = 20;

// Angle threshold for parallel sides of detected box
int parallel_angle_threshold_slider = 8;
int parallel_angle_threshold_max = 90;
int parallel_angle_threshold = 8;

// Minimum edge length for detected box
int min_parallelogram_edge_length_slider = 10;
int min_parallelogram_edge_length_max = 200;
int min_parallelogram_edge_length = 10;

// Right angle threshold for detected box (applicable for top-down view of inventory)
int right_angle_threshold_slider = 10;
int right_angle_threshold_max = 90;
int right_angle_threshold = 10;

/* CIRCLE DETECTOR PARAMETERS */
// Minimum distance between each circle
int min_circle_dist_slider = 20;
int min_circle_dist_max = 500;
int min_circle_dist = 20;

// Threshold for canny edge detector
int canny_edge_detector_thresh_slider = 50;
int canny_edge_detector_thresh_max = 500;
int canny_edge_detector_thresh = 50;

// Hough accumulator threshold
int hough_accumulator_thresh_slider = 50;
int hough_accumulator_thresh_max = 500;
int hough_accumulator_thresh = 50;

// Circle radius for detection
int circle_radius_slider = 20;
int circle_radius_max = 500;
int circle_radius = 20;

// Circle percentage error tolerance
int circle_radius_perc_tolerance_slider = 10;
int circle_radius_perc_tolerance_max = 50;
int circle_radius_perc_tolerance = 10;

AlgoCalibrator::AlgoCalibrator(ros::NodeHandle n_converter, int box_or_circle_algo) : image_transporter(n_converter) {
    image_subscriber = image_transporter.subscribe("/camera/color/image_raw", 1,
                                                   &AlgoCalibrator::image_converter_callback, this);

    // OpenCV window
    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
    
    this->box_or_circle_algo = box_or_circle_algo;
    if (box_or_circle_algo == BOX) {
        /* BOX DETECTOR PARAMETER SLIDERS INIT*/
        char TrackbarName_hessian[50];
        std::sprintf(TrackbarName_hessian, "Hessian x %d", hessian_slider_max);
        cv::createTrackbar(TrackbarName_hessian, OPENCV_WINDOW, &hessian_slider, hessian_slider_max, &AlgoCalibrator::on_trackbar_hessian, this);

        char TrackbarName_kmeans[50];
        std::sprintf(TrackbarName_kmeans, "K-Means x %d", K_max);
        cv::createTrackbar(TrackbarName_kmeans, OPENCV_WINDOW, &K_slider, K_max, &AlgoCalibrator::on_trackbar_kmeans, this);

        char TrackbarName_parallel_angle[50];
        std::sprintf(TrackbarName_parallel_angle, "Parallel angle threshold x %d", parallel_angle_threshold_max);
        cv::createTrackbar(TrackbarName_parallel_angle, OPENCV_WINDOW, &parallel_angle_threshold_slider, parallel_angle_threshold_max, &AlgoCalibrator::on_trackbar_parallel_angle_threshold, this);

        char TrackbarName_min_parallelogram_edge_length[50];
        std::sprintf(TrackbarName_min_parallelogram_edge_length, "Minimum parallelogram edge length x %d", min_parallelogram_edge_length_max);
        cv::createTrackbar(TrackbarName_min_parallelogram_edge_length, OPENCV_WINDOW, &min_parallelogram_edge_length_slider, min_parallelogram_edge_length_max, &AlgoCalibrator::on_trackbar_min_parallelogram_edge_length, this);

        char TrackbarName_right_angle_threshold[50];
        std::sprintf(TrackbarName_right_angle_threshold, "Right angle threshold x %d", right_angle_threshold_max);
        cv::createTrackbar(TrackbarName_right_angle_threshold, OPENCV_WINDOW, &right_angle_threshold_slider, right_angle_threshold_max, &AlgoCalibrator::on_trackbar_right_angle_threshold, this);

    } else if (box_or_circle_algo == CIRCLE) {
        /* CIRCLE DETECTOR PARAMETER SLIDERS INIT */
        char TrackbarName_circle_dist[50];
        std::sprintf(TrackbarName_circle_dist, "Minimum circle distance x %d", min_circle_dist_max);
        cv::createTrackbar(TrackbarName_circle_dist, OPENCV_WINDOW, &min_circle_dist_slider, min_circle_dist_max, &AlgoCalibrator::on_trackbar_circle_dist, this);
    
        char TrackbarName_canny_thresh[50];
        std::sprintf(TrackbarName_canny_thresh, "Canny edge detector threshold x %d", canny_edge_detector_thresh_max);
        cv::createTrackbar(TrackbarName_canny_thresh, OPENCV_WINDOW, &canny_edge_detector_thresh_slider, canny_edge_detector_thresh_max, &AlgoCalibrator::on_trackbar_canny_thresh, this);
    
        char TrackbarName_hough_thresh[50];
        std::sprintf(TrackbarName_hough_thresh, "Hough circle detector threshold x %d", hough_accumulator_thresh_max);
        cv::createTrackbar(TrackbarName_hough_thresh, OPENCV_WINDOW, &hough_accumulator_thresh_slider, hough_accumulator_thresh_max, &AlgoCalibrator::on_trackbar_hough_thresh, this);
        
        char TrackbarName_circle_radius[50];
        std::sprintf(TrackbarName_circle_radius, "Detecting circle radius x %d", circle_radius_max);
        cv::createTrackbar(TrackbarName_circle_radius, OPENCV_WINDOW, &circle_radius_slider, circle_radius_max, &AlgoCalibrator::on_trackbar_circle_radius, this);
         
        char TrackbarName_radius_tolerance[50];
        std::sprintf(TrackbarName_radius_tolerance, "Circle radius percentage tolerance x %d", circle_radius_perc_tolerance_max);
        cv::createTrackbar(TrackbarName_radius_tolerance, OPENCV_WINDOW, &circle_radius_perc_tolerance_slider, circle_radius_perc_tolerance_max, &AlgoCalibrator::on_trackbar_radius_tolerance, this);
    }
}

/* BOX DETECTOR SLIDER CALLBACKS */
void AlgoCalibrator::on_trackbar_hessian(int, void *ptr) {
    hessian = hessian_slider;
    ROS_INFO("hessian: %u", hessian);
}
void AlgoCalibrator::on_trackbar_kmeans(int, void *ptr) {
    K = K_slider;
    ROS_INFO("K: %u", K);
}
void AlgoCalibrator::on_trackbar_parallel_angle_threshold(int, void *ptr) {
    parallel_angle_threshold = parallel_angle_threshold_slider;
    ROS_INFO("parallel_angle_threshold: %u", parallel_angle_threshold);
}
void AlgoCalibrator::on_trackbar_min_parallelogram_edge_length(int, void *ptr) {
    min_parallelogram_edge_length = min_parallelogram_edge_length_slider;
    ROS_INFO("min_parallelogram_edge_length: %u", min_parallelogram_edge_length);
}
void AlgoCalibrator::on_trackbar_right_angle_threshold(int, void *ptr) {
    right_angle_threshold = right_angle_threshold_slider;
    ROS_INFO("right_angle_threshold: %u", right_angle_threshold);
}

/* CIRCLE DETECTOR SLIDER CALLBACKS */
void AlgoCalibrator::on_trackbar_circle_dist(int, void *ptr) {
    min_circle_dist = min_circle_dist_slider;
    ROS_INFO("min_circle_dist: %u", min_circle_dist);
}
void AlgoCalibrator::on_trackbar_canny_thresh(int, void *ptr) {
    canny_edge_detector_thresh = canny_edge_detector_thresh_slider;
    ROS_INFO("canny_edge_detector_thresh: %u", canny_edge_detector_thresh);
}
void AlgoCalibrator::on_trackbar_hough_thresh(int, void *ptr) {
    hough_accumulator_thresh = hough_accumulator_thresh_slider;
    ROS_INFO("hough_accumulator_thresh: %u", hough_accumulator_thresh);
}
void AlgoCalibrator::on_trackbar_circle_radius(int, void *ptr) {
    circle_radius = circle_radius_slider;
    ROS_INFO("circle_radius: %u", circle_radius);
}
void AlgoCalibrator::on_trackbar_radius_tolerance(int, void *ptr) {
    circle_radius_perc_tolerance = circle_radius_perc_tolerance_slider;
    ROS_INFO("circle_radius_perc_tolerance: %u", circle_radius_perc_tolerance);
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

    std::vector<cv::Point2f> pickpoints_xy;
    if (this->box_or_circle_algo == BOX) {
        // Detect boxes in image using current calibrator values
        bool show_feature_matches = false;
        try {
            detect_boxes(pickpoints_xy, this->cv_ptr->image, hessian, K, parallel_angle_threshold, min_parallelogram_edge_length, right_angle_threshold, show_feature_matches);
        } catch (cv::Exception e) {
            ROS_INFO("Segmentation failed!");
        }
    } else if (this->box_or_circle_algo == CIRCLE) {
        // Detect circles in image using current calibrator values
        try {
            detect_circles(pickpoints_xy, this->cv_ptr->image, (double) min_circle_dist, (double) canny_edge_detector_thresh, 
                                (double) hough_accumulator_thresh, circle_radius, circle_radius_perc_tolerance);
        } catch (cv::Exception e) {
            ROS_INFO("Segmentation failed!");
        }
    }

    // Show modified image
    cv::imshow(OPENCV_WINDOW, this->cv_ptr->image);
    cv::waitKey(100);
}

AlgoCalibrator::~AlgoCalibrator() {
    cv::destroyWindow(OPENCV_WINDOW);
}
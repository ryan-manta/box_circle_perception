/* Circle Detection */
//https://docs.opencv.org/master/d4/d70/tutorial_hough_circle.html

#include "hough_circle_detector.hpp"

bool detect_circles(std::vector<cv::Point2f>& pickpoints_xy_output, cv::Mat& source_img_ptr, double min_circle_dist, double canny_edge_detector_thresh, 
                        double hough_accumulator_thresh, int circle_radius, int circle_radius_perc_tolerance) {
    
    // Transfer from rgb to grayscale color space
    cv::Mat gray_img;
    cv::cvtColor(source_img_ptr, gray_img, cv::COLOR_BGR2GRAY);
    
    // Blur image to avoid false circles
    cv::GaussianBlur(gray_img, gray_img, cv::Size(9, 9), 2, 2);
    
    // Specify detected circle radius tolerance based on percentage
    double circle_radius_decimal_tolerance = (double) circle_radius_perc_tolerance / 100.0;
    int min_radius = circle_radius - (circle_radius * circle_radius_decimal_tolerance);
    int max_radius = circle_radius + (circle_radius * circle_radius_decimal_tolerance);
    
    // Detect circles in image with Hough transform
    std::vector<cv::Vec3f> detected_circles;
    HoughCircles(gray_img, detected_circles, cv::HOUGH_GRADIENT, 1,
                min_circle_dist,
                canny_edge_detector_thresh,
                hough_accumulator_thresh, 
                min_radius, 
                max_radius
    );
    
    // Draw found circles on source image
    for(int i=0; i<detected_circles.size(); i++)
    {
        cv::Vec3i circle = detected_circles[i];
        cv::Point circle_center = cv::Point(circle[0], circle[1]);

        // Draw center
        cv::circle(source_img_ptr, circle_center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);

        // Draw outline
        int radius = circle[2];
        cv::circle(source_img_ptr, circle_center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);

        // Add circle center to pickpoint vector
        pickpoints_xy_output.push_back((cv::Point2f) circle_center);
    }

    // Check if we found at least one circle
    if (detected_circles.size() > 1) {
        return true;
    } else {
        return false;
    }

}
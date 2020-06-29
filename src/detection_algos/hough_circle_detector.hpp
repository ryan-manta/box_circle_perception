/* Circle Detection */
//https://docs.opencv.org/master/d4/d70/tutorial_hough_circle.html

#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void detect_circles(cv::Mat* source_img_ptr) {
    // Transfer from rgb to grayscale color space
    cv::Mat gray_img;
    cv::cvtColor(*source_img_ptr, gray_img, cv::COLOR_BGR2GRAY);
    
    // Blur image to avoid false circles
    cv::GaussianBlur(gray_img, gray_img, cv::Size(9, 9), 2, 2 );
    
    // Detect circles in image with Hough transform
    std::vector<cv::Vec3f> detected_circles;
    double min_dist = gray_img.rows/16; // Minimum circle distance to each other
    double canny_edge_detector_thresh = 50; // 200 Threshold for canny edge detector
    double hough_accumulator_thresh = 70; // 100 Threshold for hough accumulator
    int min_radius = 1;
    int max_radius = 0; // 0 sets max to image dim
    HoughCircles(gray_img, detected_circles, cv::HOUGH_GRADIENT, 1,
                min_dist,
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
        cv::circle(*source_img_ptr, circle_center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);

        // Draw outline
        int radius = circle[2];
        cv::circle(*source_img_ptr, circle_center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }
}
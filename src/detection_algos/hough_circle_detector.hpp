#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

bool detect_circles(std::vector<cv::Point2f>& pickpoints_xy_output, cv::Mat& source_img_ptr, double min_circle_dist, double canny_edge_detector_thresh, 
                        double hough_accumulator_thresh, int circle_radius, int circle_radius_perc_tolerance);
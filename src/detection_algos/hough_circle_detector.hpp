/* 
    This function performs detection of multiple circular objects in an input image using parameters passed in. This code closely follows a tutorial from
    OpenCV: https://docs.opencv.org/master/d4/d70/tutorial_hough_circle.html
    This algorithm works quite well out of the box with some tuning. For tuning I would recommend bringing down the canny edge detector threshold
    and hough accumulator threshold until a lot of circles are covering the objects. Then increase the minimum circle distance and circle radius to get one circle
    on each object.

    Ted Lutkus
    7/6/2020
*/

#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

bool detect_circles(std::vector<cv::Point2f>& pickpoints_xy_output, cv::Mat& source_img_ptr, double min_circle_dist, double canny_edge_detector_thresh, 
                        double hough_accumulator_thresh, int circle_radius, int circle_radius_perc_tolerance);
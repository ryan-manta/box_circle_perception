/*  
    This will take two images, one a reference image of an object, and the second
    an image of a scene that contains that object, and use feature matching to locate the
    object in the scene.

    Ted Lutkus
    6/25/20

    Adapted from tutorial:
    https://docs.opencv.org/3.4/d7/dff/tutorial_feature_homography.html

    The primary change from the tutorial is the introduction of K-Means clustering to perform detection on multiple items.
    The tutorial's algorithm is designed to find one instance of the reference object. By clustering and running the algorithm on each cluster,
    we can find multiple instances of objects separately from each other and then combine the results at the end through some filtering.

    This function can be called multiple times to perform a batch of detections. New pickpoints will be appended onto the vector passed in without overwriting.
*/

#include <ros/ros.h>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv4/opencv2/xfeatures2d/nonfree.hpp"
#include <vector>
#include <chrono>
#include <iostream>

#include "lib/KMeansRex_light/src/KMeansRexCore.cpp"
#include <Eigen/Dense>

bool detect_boxes(std::vector<cv::Point2f>& pickpoints_xy_output, cv::Mat& source_img_ptr, int hessian_threshold, int K, 
                    float parallel_angle_threshold, float min_parallelogram_edge_length, float right_angle_threshold, bool draw_feature_matches);
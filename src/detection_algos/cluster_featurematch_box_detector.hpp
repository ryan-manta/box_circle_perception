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
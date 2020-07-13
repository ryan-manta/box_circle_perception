#include <ros/ros.h>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>
#include <chrono>

#include "lib/KMeansRex_light/src/KMeansRexCore.cpp"
#include <Eigen/Dense>

bool detect_boxes(std::vector<cv::Point2f>& pickpoints_xy_output, cv::Mat& source_img_ptr, int hessian_threshold, int K, bool draw_feature_matches);
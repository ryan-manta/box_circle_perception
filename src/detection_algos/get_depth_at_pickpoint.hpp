/*
    This function will take an unstructured pointcloud and a opencv point, the output is the depth value at the supplied opencv point. 
    The function assumes that the point and the pointcloud are already in the same reference frame.

    Ted Lutkus
    7/6/2020
*/

#include <opencv2/core/types.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

float get_depth_at_pickpoint(const cv::Point2f& point_xy, const PointCloud::ConstPtr& pointcloud);
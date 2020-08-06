/*
    This function will take an unstructured pointcloud and a opencv point, the output is the depth value at the supplied opencv point. 
    The function assumes that the point and the pointcloud are already in the same reference frame.

    Ted Lutkus
    7/6/2020
*/

#include "get_depth_at_pickpoint.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

float get_depth_at_pickpoint(const cv::Point2f& point_xy, const PointCloud::ConstPtr& pointcloud) {
    // Pull 2D coordinates and extract the depth
    float x = point_xy.x;
    float y = point_xy.y;

    // Since this is an unorganized pointcloud, set threshold for when we are close enough to desired
    // point [x, y] in pointcloud
    float point_threshold = 1.02;
    BOOST_FOREACH (const pcl::PointXYZ& point, pointcloud->points) {
        if ( ((point.x / x) < point_threshold && (point.y / y) < point_threshold) ) {
            return point.z;
        }
    }

    return 0;
}
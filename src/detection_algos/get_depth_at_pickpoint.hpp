#include <opencv2/core/types.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

float get_depth_at_pickpoint(const cv::Point2f& point_xy, const PointCloud::ConstPtr& pointcloud);
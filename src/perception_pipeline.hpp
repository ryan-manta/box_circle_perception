/*
 *  This perception pipeline is what should be used after calibrating items and algorithms.
 *  The pipeline is intialized and then provides a service that can be called to generate a pickpoint [x, y, z].
 *  The service takes an input vector (that will be used to store the pickpoints generated) as well as the name of the item to be picked.
 *  Currently, the item names and their associated types are hardcoded in, it should be straight forward to have this read from a file if needed.
 *  The reference image for box detection currently defaults to the "cropped_image.jpg" file in the data/ folder.
 *
 * NOTE *
 *  While the perception algorithms are implemented, further integration is needed to get the pipeline ready for robotic picking:
 *  For development, a realsense camera was used and pickpoints were generated in the realsense camera frame.
 *  In order to use multiple sensors and/or use the generated pickpoint for robotic manipulation, the sensor frames will need to be
 *  calibrated with respect to the robot frame so that the robot can properly move to the position output of the perception pipeline.
 *
 *  For a tutorial on camera calibration, see:
 *  https://industrial-training-master.readthedocs.io/en/melodic/_source/demo3/Setting-up-a-3D-sensor.html?highlight=camera%20calibration
 *
 *  Ted Lutkus
 *  6/26/20
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include "hough_circle_detector.hpp"
#include "cluster_featurematch_box_detector.hpp"
#include "get_depth_at_pickpoint.hpp"
#include "green_pick/GeneratePickpoint.h"
#include <string>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "Grocery_Items_List.hpp"

#define BOX		  0
#define CIRCLE	  1

typedef pcl::PointCloud <pcl::PointXYZ> PointCloud;

class PerceptionPipeline {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transporter;
    image_transport::Subscriber image_subscriber;
    image_transport::Publisher image_publisher;
    cv_bridge::CvImagePtr cv_ptr;

    // Pointcloud data
    PointCloud current_pointcloud;
    ros::Publisher pointcloud_publisher;
    bool get_pointcloud;

public:
    PerceptionPipeline();
    // Setup for the output pointcloud publisher
    void set_pointcloud_publisher(ros::Publisher pointcloud_publisher);

    // Callback function for the image transport ROS node
    void perception_callback(const sensor_msgs::ImageConstPtr& msg);

    // Callback function for the pointcloud node
    void pointcloud_callback(const PointCloud::ConstPtr& msg);

    // Generate pickpoint service callback
    bool generate_pickpoint(green_pick::GeneratePickpoint::Request&req,
                            green_pick::GeneratePickpoint::Response&res);
};

int main(int argc, char **argv) {
    // Initialize vision node
    ros::init(argc, argv, "perception_pipeline");
    PerceptionPipeline greenpick_perception_pipeline;

    // Initialize pointcloud subscriber
    ros::init(argc, argv, "pointcloud_sub");
    ros::NodeHandle pointcloud_sub_nh;
    ros::Subscriber pointcloud_sub = pointcloud_sub_nh.subscribe <PointCloud>("/camera/depth/color/points", 1, &PerceptionPipeline::pointcloud_callback, &greenpick_perception_pipeline);

    // Initialize pointcloud publisher
    ros::init(argc, argv, "pointcloud_pub");
    ros::NodeHandle pointcloud_pub_nh;
    ros::Publisher	pointcloud_pub = pointcloud_pub_nh.advertise <PointCloud>("/perception_pipeline/output_points", 1);
    greenpick_perception_pipeline.set_pointcloud_publisher(pointcloud_pub);

    ros::init(argc, argv, "pickpoint_generator_server");
    ros::NodeHandle	   n;
    ros::ServiceServer service = n.advertiseService("generate_pickpoint", &PerceptionPipeline::generate_pickpoint, &greenpick_perception_pipeline);
    ROS_INFO("Pick service ready for calling.");

    ros::spin();
    return (0);
}

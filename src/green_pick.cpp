#include <ros/ros.h>
#include "perception_pipeline.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perception_pipeline");
    PerceptionPipeline greenpick_perception_pipeline();
    ros::spin();
    return 0;
}
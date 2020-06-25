//http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";


/*

DEMO:
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
rosrun green_pick image_converter
rviz

*/

/* Circle Detection */
//https://docs.opencv.org/master/d4/d70/tutorial_hough_circle.html
void detect_circles(cv::Mat* source_img_ptr) {
    // Transfer from rgb to grayscale color space
    cv::Mat gray_img;
    cv::cvtColor(*source_img_ptr, gray_img, cv::COLOR_BGR2GRAY);
    
    // Blur image to avoid false circles
    cv::GaussianBlur(gray_img, gray_img, cv::Size(9, 9), 2, 2 );
    
    // Detect circles in image with Hough transform
    std::vector<cv::Vec3f> detected_circles;
    double min_dist = gray_img.rows/16; // Minimum circle distance to each other
    double canny_edge_detector_thresh = 50; // 200 Threshold for canny edge detector
    double hough_accumulator_thresh = 70; // 100 Threshold for hough accumulator
    int min_radius = 1;
    int max_radius = 0; // 0 sets max to image dim
    HoughCircles(gray_img, detected_circles, cv::HOUGH_GRADIENT, 1,
                min_dist,
                canny_edge_detector_thresh,
                hough_accumulator_thresh, 
                min_radius, 
                max_radius
    );
    
    // Draw found circles on source image
    for(int i=0; i<detected_circles.size(); i++)
    {
        cv::Vec3i circle = detected_circles[i];
        cv::Point circle_center = cv::Point(circle[0], circle[1]);

        // Draw center
        cv::circle(*source_img_ptr, circle_center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);

        // Draw outline
        int radius = circle[2];
        cv::circle(*source_img_ptr, circle_center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }
}



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Run Hough circle detection
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
      detect_circles(&cv_ptr->image);
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
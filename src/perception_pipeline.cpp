#include "perception_pipeline.hpp"

static const std::string OPENCV_WINDOW = "Image window";

PerceptionPipeline::PerceptionPipeline(StateMachine& state_machine)
        : image_transporter(nh_), state_machine(state_machine) {

    // Subscribe to input video feed and publish output video feed
    image_subscriber = image_transporter.subscribe("/camera/color/image_raw", 1,
    &PerceptionPipeline::perception_callback, this);
    image_publisher = image_transporter.advertise("/image_converter/output_video", 1);
    //cv::namedWindow(OPENCV_WINDOW);
}

void PerceptionPipeline::perception_callback(const sensor_msgs::ImageConstPtr& msg) {
    // Try to copy ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Check if image has data
    if (!cv_ptr->image.data) {
        ROS_ERROR("No image data found!");
        return;
    }

    // Run object detection if the robot is ready for it based on the state machine
    int state = this->state_machine.get_state();
    if (state == READY_FOR_PICKPOINT) {
        // pickpoint_generation
    } else if (state == CALIBRATE_NEW_ITEM) {
        // item calibration
    } else {
        // else do nothing
    }

    // Run Hough circle detection
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
      //detect_circles(&cv_ptr->image);
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);

    // Output modified video stream
    image_publisher.publish(cv_ptr->toImageMsg());
}

PerceptionPipeline::~PerceptionPipeline() {
    //cv::destroyWindow(OPENCV_WINDOW);
}
#include "box_item_calibrator.hpp"

Calibrator::Calibrator(ros::NodeHandle n_converter) : image_transporter(n_converter) {
    //image_subscriber = image_transporter.subscribe("/camera/color/image_raw", 1,
    //                                               &Calibrator::image_converter_callback, this);
    image_subscriber = image_transporter.subscribe("/pylon_camera_node/image_raw", 1,
                                                   &Calibrator::image_converter_callback, this);

    // Set calibration to false at start
    do_calibration = false;

    // OpenCV window
    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
    //cv::resizeWindow(OPENCV_WINDOW, 1920, 1080);
    cv::setMouseCallback(OPENCV_WINDOW, &Calibrator::onMouse, this);

    //cv::namedWindow("Select ROI", cv::WINDOW_NORMAL);
    //cv::resizeWindow(OPENCV_WINDOW, 1920, 1080);

    //cv::namedWindow("Selected Item", cv::WINDOW_NORMAL);
    //cv::resizeWindow(OPENCV_WINDOW, 1920, 1080);

}

void Calibrator::onMouse(int event, int x, int y, int, void *ptr) {
    if (event != cv::EVENT_LBUTTONDOWN) {
        return;
    }

    // Set do_calibration to true
    Calibrator *this_c = (Calibrator *)ptr;
    this_c->do_calibration = true;
    return;
}

void Calibrator::calibrate_box_item(cv::Mat &source_image) {
    // Select region of interest (ROI)
    //cv::namedWindow("Select ROI", cv::WINDOW_NORMAL);
    cv::Rect2d selected_rectangle = cv::selectROI("Select ROI", source_image);
    //cv::resizeWindow("Select ROI", 1920, 1080);


    // Crop image down to selected ROI
    cv::Mat cropped_image = source_image(selected_rectangle);

    // Save cropped image to ./data/
    cv::imwrite("./data/cropped_image.jpg", cropped_image);

    // Display cropped image
    //cv::namedWindow("Selected Item", cv::WINDOW_NORMAL);
    cv::imshow("Selected Item", cropped_image);
    //cv::resizeWindow("Select Item", 1920, 1080);
    cv::waitKey(0);
}

void Calibrator::image_converter_callback(const sensor_msgs::ImageConstPtr &msg) {
    // Try to copy ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Check if image has data
    if (!cv_ptr->image.data) {
        ROS_ERROR("No image data found!");
        return;
    }

    // Show current camera view
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Do calibration
    if (this->do_calibration) {
        calibrate_box_item(cv_ptr->image);
        this->do_calibration = false;
    }
}

Calibrator::~Calibrator(){
    cv::destroyWindow;
}
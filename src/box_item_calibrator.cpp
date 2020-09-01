/*
 *  This node can be run to calibrate a new rectangular item for 2D segmentation.
 *
 *  To use:
 *  1. *FROM ROOT OF REPO* Run the box item calibrator node with rosrun
 *      - A live camera feed should pop up
 *  2. When ready to calibrate, click anywhere on the live camera feed
 *      - A single frame captured at the current time should be presented
 *  3. Click and drag from one corner of the item to the opposite corner of the item
 *      - A blue crosshair should follow the mouse while dragging to show the selection
 *  4. Release the mouse button, the selected region should pop up.
 *      - This image is saved to ./data/cropped_image.jpg (Currently no option for naming the saved item)
 *      - Currently this will overwrite the current "cropped_image.jpg" file in that directory.
 *      - Saving different items and calling different items by name is a feature that needs to be added.
 *
 *  Ted Lutkus
 *  6/26/20
 */

#include "box_item_calibrator.hpp"

Calibrator::Calibrator(ros::NodeHandle n_converter, std::string grocery_item) : image_transporter(n_converter) {
    // Subscribe to the raw image topic published by a Realsense camera
    image_subscriber = image_transporter.subscribe("/camera/color/image_raw", 1,
                                                   &Calibrator::image_converter_callback, this);

    // Set calibration to false at start
    do_calibration	   = false;
    this->grocery_item = grocery_item;

    // OpenCV window
    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(OPENCV_WINDOW, &Calibrator::onMouse, this);
}

void Calibrator::onMouse(int event, int x, int y, int, void *ptr) {
    if (event != cv::EVENT_LBUTTONDOWN) {
        return;
    }

    // Set do_calibration to true
    Calibrator *this_c = (Calibrator *) ptr;
    this_c->do_calibration = true;
    return;
}

void Calibrator::calibrate_box_item(cv::Mat&source_image) {
    // Select region of interest (ROI)
    cv::Rect2d selected_rectangle = cv::selectROI("Select ROI", source_image);

    // Crop image down to selected ROI
    cv::Mat cropped_image = source_image(selected_rectangle);

    // Save cropped image to ./data/
    std::string image_name = "./data/" + this->grocery_item + ".jpg";

    cv::imwrite(image_name, cropped_image);

    // Display cropped image
    cv::imshow("Selected Item", cropped_image);
    cv::waitKey(0);
}

void Calibrator::image_converter_callback(const sensor_msgs::ImageConstPtr&msg) {
    // Try to copy ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception&e) {
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

Calibrator::~Calibrator() {
    cv::destroyWindow;
}

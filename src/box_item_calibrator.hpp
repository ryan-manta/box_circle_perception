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

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <sqlite3.h>

static const std::string OPENCV_WINDOW = "Box Item Feature Matching Selection";

class Calibrator {
private:
    image_transport::ImageTransport image_transporter;
    image_transport::Subscriber image_subscriber;
    bool do_calibration;
    std::string grocery_item;

public:
    Calibrator(ros::NodeHandle n_converter, std::string grocery_item);

    // Callback for first mouse press to start calibration
    static void onMouse(int event, int x, int y, int, void *ptr);

    // Function called once mouse is pressed to start calibration
    void calibrate_box_item(cv::Mat& source_image);

    // Callback to receive new images from camera feed
    void image_converter_callback(const sensor_msgs::ImageConstPtr& msg);

    ~Calibrator();
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "calibrator_image_converter");
    ros::NodeHandle n_converter;

    std::string grocery_item;
    std::cout << "Please enter the grocery item being calibrated (i.e. penne_pasta): ";
    std::getline(std::cin, grocery_item);
    Calibrator calibrator(n_converter, grocery_item);

    ros::spin();
    return (0);
}

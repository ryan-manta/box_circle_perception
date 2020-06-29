/* 
    This script can be run to calibrate a new rectangular item
    for 2D segmentation in Green Pick. The user will run the script,
    be presented a gui with an image from our sensor, select the box,
    and have any necessary feature data stored 

    Ted Lutkus
    6/26/19
*/

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include "ros/ros.h"
#include "green_pick/CalibrateBoxItem.h"

static const std::string OPENCV_WINDOW = "Image window";

class Calibrator {
private:
    //ros::NodeHandle nh_;
    image_transport::ImageTransport image_transporter;
    image_transport::Subscriber image_subscriber;
    bool do_calibration;

public:
    Calibrator(ros::NodeHandle n_converter) : image_transporter(n_converter) {
        image_subscriber = image_transporter.subscribe("/camera/color/image_raw", 1,
            &Calibrator::image_converter_callback, this);

        // Set calibration to false at start
        do_calibration = false;

        // OpenCV window
        cv::namedWindow(OPENCV_WINDOW);
        cv::setMouseCallback(OPENCV_WINDOW, &Calibrator::onMouse, this);
        cv::createButton("CALIBRATE ITEM", &Calibrator::calibrate_button_callback, this, CV_PUSH_BUTTON,1);
    }

    bool calibrate_switch(green_pick::CalibrateBoxItem::Request &req,
                            green_pick::CalibrateBoxItem::Response &res) {
        // Switch calibration on
        do_calibration = true;
        return true;
    }

    static void calibrate_button_callback(int state, void* ptr) {
        // Set do_calibration to true
        Calibrator* this_c = (Calibrator*) ptr;
        this_c->do_calibration = true;
        return;
    }

    static void onMouse(int event, int x, int y, int, void* ptr) {
        if(event != cv::EVENT_LBUTTONDOWN) {
            return;
        }   

        // Set do_calibration to true
        Calibrator* this_c = (Calibrator*) ptr;
        this_c->do_calibration = true;
        return;
    }

    void calibrate_box_item(cv::Mat& source_image) {

        // Select region of interest (ROI)
        cv::Rect2d selected_rectangle = cv::selectROI(source_image);

        // Crop image down to selected ROI
        cv::Mat cropped_image = source_image(selected_rectangle);

        // Save cropped image to ./
        cv::imwrite("./data/cropped_image.jpg", cropped_image);

        // Display cropped image
        cv::imshow("Selected Item", cropped_image);
        cv::waitKey(0);
    }

    void image_converter_callback(const sensor_msgs::ImageConstPtr& msg) {
        // Try to copy ROS image to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Check if image has data
        if (!cv_ptr->image.data)
        {
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

    ~Calibrator() {
        cv::destroyWindow(OPENCV_WINDOW);
    }
};



int main(int argc, char **argv) {
    ros::init(argc, argv, "calibrator_image_converter");
    ros::NodeHandle n_converter;
    Calibrator calibrator(n_converter);

    ros::init(argc, argv, "calibrate_box_item_server");
    ros::NodeHandle n_calibrator;
    //Calibrator* calibrator_ptr = &calibrator;
    ros::ServiceServer service = n_calibrator.advertiseService("calibrate_box_item", &Calibrator::calibrate_switch, &calibrator);
    ROS_INFO("Ready to calibrate a new item.");
    ros::spin();
    return 0;
}
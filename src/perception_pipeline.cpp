#include "perception_pipeline.hpp"

static const std::string OPENCV_WINDOW = "Image window";

PerceptionPipeline::PerceptionPipeline()
      : image_transporter(nh_) {

    // Subscribe to input video feed and publish output video feed
    image_subscriber = image_transporter.subscribe("/camera/color/image_raw", 1,
    &PerceptionPipeline::perception_callback, this);
    image_publisher = image_transporter.advertise("/perception_pipeline/output_video", 1);
}

void PerceptionPipeline::perception_callback(const sensor_msgs::ImageConstPtr& msg) {
    // Try to copy ROS image to OpenCV image
    try {
        this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Check if image has data
    if (!this->cv_ptr->image.data) {
        ROS_ERROR("No image data found!");
        return;
    }

    // Output modified video stream
    // image_publisher.publish(cv_ptr->toImageMsg());
}

bool PerceptionPipeline::generate_pickpoint(green_pick::GeneratePickpoint::Request &req,
                        green_pick::GeneratePickpoint::Response &res) {
    /* CURRENTLY HARDCODED IN */
    // See if item has a circle top or box top
    std::vector<std::string> items = {"penne_pasta", "elbows_pasta", "tin_coffee"};
    std::vector<int> box_or_circle = {BOX, BOX, CIRCLE};

    // Check our item set for the item we want to pick
    for (int i = 0; i < items.size(); i++) {
        if (req.item_name.compare(items[i]) == 0) {

            // If we hit item, perform its associated detection method
            if (box_or_circle[i] == BOX) {
                // SURF
                int hessian_threshold = 400;
                int K = 12;
                bool draw_feature_matches = true;
                bool pick_success = detect_boxes(this->cv_ptr->image, hessian_threshold, K, draw_feature_matches);
                
                if (pick_success) {
                    for (int i = 0; i < 3; i++) {
                        res.pick_coordinates[i] = 0;
                    }

                    image_publisher.publish(cv_ptr->toImageMsg());
                }

                return pick_success;
            } else if (box_or_circle[i] == CIRCLE) {
                // Hough circle detection
                detect_circles(this->cv_ptr->image);
                
                for (int i = 0; i < 3; i++) {
                    res.pick_coordinates[i] = 0;
                }

                image_publisher.publish(cv_ptr->toImageMsg());
                return true;
                
            } else {
                ROS_ERROR("Item is not properly labeled.");
                return false;
            }

        }
    }
    
    image_publisher.publish(cv_ptr->toImageMsg());
    return true;
}
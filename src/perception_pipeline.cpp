#include "perception_pipeline.hpp"

static const std::string OPENCV_WINDOW = "Image window";

PerceptionPipeline::PerceptionPipeline()
      : image_transporter(nh_) {

    // Subscribe to input video feed and publish output video feed
    image_subscriber = image_transporter.subscribe("/camera/color/image_raw", 1,
    &PerceptionPipeline::perception_callback, this);
    image_publisher = image_transporter.advertise("/perception_pipeline/output_images", 1);

    get_pointcloud = false;
}

void PerceptionPipeline::set_pointcloud_publisher(ros::Publisher pointcloud_publisher) {
    this->pointcloud_publisher = pointcloud_publisher;
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

void PerceptionPipeline::pointcloud_callback(const PointCloud::ConstPtr& msg) {
    if (get_pointcloud) {
        this->current_pointcloud = *msg;
        get_pointcloud = false;
    }
}

template <class T>
static void DoNotFree(T*)
{
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
            std::vector<cv::Point2f> pickpoints_xy;
            this->get_pointcloud = true;
            int pickpoint_sample_size = 10;
            int minimum_required_pickpoints = 8;
            int number_of_success = 0;

            // If we hit item, perform its associated detection method
            if (box_or_circle[i] == BOX) {
                // Collect multiple batches of pickpoints using cluster feature map box detection
                int hessian_threshold = 100;
                int K = 20;
                bool draw_feature_matches = false;
                for (int j = 0; j < pickpoint_sample_size; j++) {
                    bool pick_success = detect_boxes(pickpoints_xy, this->cv_ptr->image, hessian_threshold, K, draw_feature_matches);
                    if (pick_success) {
                        number_of_success++;
                    }
                }
            } else if (box_or_circle[i] == CIRCLE) {
                // Collect multiple batches of pickpoints using hough circle detection
                for (int j = 0; j < pickpoint_sample_size; j++) {
                    double min_circle_dist = 10;
                    double canny_edge_detector_thresh = 70;
                    double hough_accumulator_thresh = 100;
                    int circle_radius = 50;
                    int circle_radius_perc_tolerance = 50;
                    bool pick_success = detect_circles(pickpoints_xy, this->cv_ptr->image, min_circle_dist, canny_edge_detector_thresh, 
                                hough_accumulator_thresh, circle_radius, circle_radius_perc_tolerance);
                    if (pick_success) {
                        number_of_success++;
                    }
                }           
            } else {
                ROS_ERROR("Item is not properly labeled.");
                return false;
            }

            // Check that we have minimum number of pickpoints required to select from
            if (number_of_success <= minimum_required_pickpoints) {
                return false;
            }

            // Assume that all pickpoints returned are good pickpoints to choose from (ie. detect_boxes should only return good pickpoints)
            // Go with pickpoint closest to bottom left of inventory (minimum magnitude)
            float min_magnitude = std::sqrt(std::pow(pickpoints_xy[0].x, 2) + std::pow(pickpoints_xy[0].y, 2));
            int min_point_index = 0;
            for (int j = 0; j < pickpoints_xy.size(); j++) {
                float point_magnitude = std::sqrt(std::pow(pickpoints_xy[j].x, 2) + std::pow(pickpoints_xy[j].y, 2));
                if (point_magnitude < min_magnitude) {
                    min_magnitude = min_magnitude;
                    min_point_index = j;
                }
            }

            // Get the z_coordinate for the point picked out
            PointCloud::ConstPtr pointcloud_ptr(&this->current_pointcloud, &DoNotFree<PointCloud>);
            float z_coordinate = get_depth_at_pickpoint(pickpoints_xy[min_point_index], pointcloud_ptr);
            res.pick_coordinates[0] = pickpoints_xy[min_point_index].x;
            res.pick_coordinates[1] = pickpoints_xy[min_point_index].y;
            res.pick_coordinates[2] = z_coordinate;
            image_publisher.publish(cv_ptr->toImageMsg());
            return true;
        }
    }
    
    image_publisher.publish(cv_ptr->toImageMsg());
    return true;
}
/*  
    This perception pipeline is what should be used after calibrating items and algorithms.
    The pipeline is intialized and then provides a service that can be called to generate a pickpoint [x, y, z].
    The service takes an input vector (that will be used to store the pickpoints generated) as well as the name of the item to be picked.
    Currently, the item names and their associated types are hardcoded in, it should be straight forward to have this read from a file if needed.
    The reference image for box detection currently defaults to the "cropped_image.jpg" file in the data/ folder.
    
    * NOTE *
    While the perception algorithms are implemented, further integration is needed to get the pipeline ready for robotic picking:
    For development, a realsense camera was used and pickpoints were generated in the realsense camera frame.
    In order to use multiple sensors and/or use the generated pickpoint for robotic manipulation, the sensor frames will need to be
    calibrated with respect to the robot frame so that the robot can properly move to the position output of the perception pipeline.
    
    For a tutorial on camera calibration, see:
    https://industrial-training-master.readthedocs.io/en/melodic/_source/demo3/Setting-up-a-3D-sensor.html?highlight=camera%20calibration

    Ted Lutkus
    6/26/20
*/

#include "perception_pipeline.hpp"

static const std::string OPENCV_WINDOW = "Image window";

PerceptionPipeline::PerceptionPipeline()
      : image_transporter(nh_) {

    // Subscribe to input video feed from Realsense and publish output video feed
    image_subscriber = image_transporter.subscribe("/camera/color/image_raw", 1,
    &PerceptionPipeline::perception_callback, this);
    image_publisher = image_transporter.advertise("/perception_pipeline/output_images", 1);

    // Start without request to fetch pointcloud
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
}

void PerceptionPipeline::pointcloud_callback(const PointCloud::ConstPtr& msg) {
    // Check if a new pointcloud is neeeded
    if (get_pointcloud) {
        this->current_pointcloud = *msg;
        get_pointcloud = false;
    }
}

// Workaround for issue with pointcloud data getting freed
template <class T>
static void DoNotFree(T*) {
}

bool PerceptionPipeline::generate_pickpoint(green_pick::GeneratePickpoint::Request &req,
                        green_pick::GeneratePickpoint::Response &res) {

    /* CURRENTLY HARDCODED IN */
    // Items available for picking and their associated algorithm types
    std::vector<std::string> items = {"penne_pasta", "elbows_pasta", "coffee_tin"};
    std::vector<int> box_or_circle = {BOX, BOX, CIRCLE};

    // Check our item set for the item we want to pick
    for (int i = 0; i < items.size(); i++) {
        if (req.item_name.compare(items[i]) == 0) {
            std::vector<cv::Point2f> pickpoints_xy;
            this->get_pointcloud = true;
            
            // Initialize variables used to run a batch of detections for the given item and varify that pickpoints are being generated
            int pickpoint_sample_size = 10;
            int minimum_required_pickpoints = 8;
            int number_of_success = 0;

            // Perform item's associated detection method
            if (box_or_circle[i] == BOX) {
                // Collect multiple batches of pickpoints using cluster feature map box detection (use algorithm calibrator to find ideal parameters)
                int hessian_threshold = 100;
                int K = 20;
                int parallel_angle_threshold = 8;
                int min_parallelogram_edge_length = 10;
                int right_angle_threshold = 10;
                bool draw_feature_matches = false;
                for (int j = 0; j < pickpoint_sample_size; j++) {
                    bool pick_success = detect_boxes(pickpoints_xy, this->cv_ptr->image, hessian_threshold, K, parallel_angle_threshold, min_parallelogram_edge_length, right_angle_threshold, draw_feature_matches);
                    if (pick_success) {
                        number_of_success++;
                    }
                }
            } else if (box_or_circle[i] == CIRCLE) {
                // Collect multiple batches of pickpoints using hough circle detection (use algorithm calibrator to find ideal parameters)
                for (int j = 0; j < pickpoint_sample_size; j++) {
                    double min_circle_dist = 20;
                    double canny_edge_detector_thresh = 200;
                    double hough_accumulator_thresh = 25;
                    int circle_radius = 25;
                    int circle_radius_perc_tolerance = 30;
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

            // Assume that all pickpoints returned are good pickpoints to choose from (Could use another added layer of selection/filtering for security)
            // Go with pickpoint closest to bottom left of inventory (minimum magnitude to pickpoint)
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
    
    ROS_ERROR("Could not find item in list of pickable items.");
    return false;
}
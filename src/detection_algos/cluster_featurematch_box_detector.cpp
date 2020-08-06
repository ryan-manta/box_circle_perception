/*  
    This will take two images, one a reference image of an object, and the second
    an image of a scene that contains that object, and use feature matching to locate the
    object in the scene.

    Ted Lutkus
    6/25/20

    Adapted from tutorial:
    https://docs.opencv.org/3.4/d7/dff/tutorial_feature_homography.html

    The primary change from the tutorial is the introduction of K-Means clustering to perform detection on multiple items.
    The tutorial's algorithm is designed to find one instance of the reference object. By clustering and running the algorithm on each cluster,
    we can find multiple instances of objects separately from each other and then combine the results at the end through some filtering.

    This function can be called multiple times to perform a batch of detections. New pickpoints will be appended onto the vector passed in without overwriting.
*/

#include "cluster_featurematch_box_detector.hpp"

Eigen::IOFormat CleanFmt(3, 0, " ", "\n", "[", "]");

bool detect_boxes(std::vector<cv::Point2f>& pickpoints_xy_output, cv::Mat& source_img_ptr, int hessian_threshold, int K, 
                    float parallel_angle_threshold, float min_parallelogram_edge_length, float right_angle_threshold, bool draw_feature_matches) {

    // Load reference and sampled scene images
    cv::Mat object_reference_image = cv::imread("data/cropped_image.jpg");

    // Check that reference image has data
    if (object_reference_image.empty()){
        ROS_ERROR("Could not open or find object reference image for box detection");
        return false;
    }
    if (source_img_ptr.empty()) {
        ROS_ERROR("Sampled scene image is empty!");
        return false;
    }

    //Detect the keypoints using SURF Detector, compute the descriptors
    static cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(hessian_threshold);
    std::vector<cv::KeyPoint> object_reference_keypoints, sampled_scene_keypoints;
    cv::Mat object_reference_descriptors, sampled_scene_descriptors;
    cv::InputOutputArray detector_mask = cv::noArray();
    detector->detectAndCompute(object_reference_image, detector_mask, object_reference_keypoints, object_reference_descriptors);
    detector->detectAndCompute(source_img_ptr, detector_mask, sampled_scene_keypoints, sampled_scene_descriptors);
    if (object_reference_keypoints.empty() || object_reference_descriptors.empty() || sampled_scene_keypoints.empty() || sampled_scene_descriptors.empty()) {
        return false;
    }

    // Convert keypoint vector to 2D double vector
    int N_rows = sampled_scene_keypoints.size();
    int N_cols = 2;
    Eigen::ArrayXXd cluster_data(N_rows, N_cols);
    int i = 0;
    for (auto const & scene_keypoint : sampled_scene_keypoints) {
        cluster_data(i,0) = (double) scene_keypoint.pt.x;
        cluster_data(i,1) = (double) scene_keypoint.pt.y;
        i++;
    }

    // K-Means clustering on the scene keypoints
    // K is the number of clusters to identify in the image
    int point_dim = 2;
    int n_iters = 200;
    int seed = 42;
    int number_of_points_total = N_rows;
    char* plusplus = (char*) "plusplus";//std::string::c_str("plusplus");
    Eigen::ArrayXXd clusters = Eigen::ArrayXXd::Zero(K, point_dim);
    Eigen::ArrayXd cluster_labels = Eigen::ArrayXd::Zero(number_of_points_total);

    // Run K-Means algorithm
    RunKMeans(cluster_data.data(), number_of_points_total, point_dim, K, n_iters, seed, plusplus, clusters.data(), cluster_labels.data());

    // Perform feature matching for each cluster
    std::vector<cv::KeyPoint> scene_keypoints_cluster;
    cv::Mat scene_descriptors_cluster(sampled_scene_descriptors.row(0));
    cv::Mat matches_image;
    bool first_drawmatches = true;
    bool at_least_one_pickpoint = false;
    for (int cluster = 0; cluster < K; cluster++) {
        // Clear the keypoints vector
        scene_keypoints_cluster.clear();
        
        // Identify number of points in each cluster to initialize size of keypoint/descriptor containers
        int num_points_in_cluster = 0;
        for (int i = 0; i < number_of_points_total; i++) {
            if (cluster_labels(i,0) == cluster) {
                num_points_in_cluster++;
            }
        }

        // Extract the cluster subset of keypoints and descriptors from the reference and scene
        scene_keypoints_cluster.resize(num_points_in_cluster);
        scene_descriptors_cluster.resize(num_points_in_cluster);
        int ind = 0;
        for (int i = 0; i < number_of_points_total; i++) {
            if (cluster_labels(i,0) == cluster) {
                scene_keypoints_cluster[ind] = sampled_scene_keypoints[i];
                sampled_scene_descriptors.row(i).copyTo(scene_descriptors_cluster.row(ind));
                ind++;
            }
        }
        
        // Matching descriptor vectors with a FLANN based matcher
        cv::DescriptorMatcher::MatcherType descriptor_matcher_type = cv::DescriptorMatcher::FLANNBASED;
        cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create(descriptor_matcher_type);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        int desired_number_of_matches = 2;
        try {
            descriptor_matcher->knnMatch(object_reference_descriptors, scene_descriptors_cluster, knn_matches, desired_number_of_matches);
        } catch (cv::Exception e) {
            continue;
        }

        // Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.7f;
        std::vector<cv::DMatch> good_matches;
        for (int i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        // Draw matches
        if (draw_feature_matches) {
            const cv::Scalar match_color = cv::Scalar::all(-1);
            const cv::Scalar single_point_color = cv::Scalar::all(-1);
            const std::vector<char> matches_mask = std::vector<char>();
            cv::DrawMatchesFlags drawing_flag = cv::DrawMatchesFlags::DEFAULT;
            if (first_drawmatches) {
                // For first cluster, create new image
                first_drawmatches = false;
            } else {
                // For subsequent clusters, write over existing image
                drawing_flag = cv::DrawMatchesFlags::DRAW_OVER_OUTIMG;
            }
            cv::drawMatches(object_reference_image, object_reference_keypoints, source_img_ptr, scene_keypoints_cluster,
                            good_matches, matches_image, match_color,
                            single_point_color, matches_mask, drawing_flag);
        } else {
            matches_image = source_img_ptr;
        }

        // Localize the object in the scene
        std::vector<cv::Point2f> object_match_keypoints;
        std::vector<cv::Point2f> scene_match_keypoints;
        for (int i = 0; i < good_matches.size(); i++) {
            // Get the keypoints from the good matches
            object_match_keypoints.push_back(object_reference_keypoints[good_matches[i].queryIdx].pt);
            scene_match_keypoints.push_back(scene_keypoints_cluster[good_matches[i].trainIdx].pt);
        }
        cv::Mat object_scene_transform;
        if (scene_match_keypoints.empty()) {
            continue;
        }
        try {
            double ransacReprojThreshold = 3;
            object_scene_transform = cv::findHomography(object_match_keypoints, scene_match_keypoints, cv::RANSAC, ransacReprojThreshold);
            if (object_scene_transform.size().height == 0) {
                continue;
            }
        } catch (cv::Exception e) {
            continue;
        }

        // Get the object corners from the reference image
        // Scene corner indexing with respect to box corners:
        //      (0) - top left        (1) - top right
        //      (3) - bot left        (2) - bot right
        std::vector<cv::Point2f> object_corners(4);
        object_corners[0] = cv::Point2f(0, 0);
        object_corners[1] = cv::Point2f((float)object_reference_image.cols, 0);
        object_corners[2] = cv::Point2f((float)object_reference_image.cols, (float)object_reference_image.rows);
        object_corners[3] = cv::Point2f(0, (float)object_reference_image.rows);
        std::vector<cv::Point2f> scene_corners(4);
        try {
            cv::perspectiveTransform(object_corners, scene_corners, object_scene_transform);
        } catch (cv::Exception e) {
            continue;
        }

        // Check if the detected box is a parallelogram by comparing the angles of opposite lines
        // Angle_threshold is the maximum angular tolerance allowed for each set of parallel lines
        float parallel_angle_threshold_rad = parallel_angle_threshold * M_PI / 180.0;
        std::vector<float> angles(4);
        std::vector<int> edge_start_ind = {0, 3, 3, 2};
        std::vector<int> edge_end_ind = {1, 2, 0, 1};
        bool edge_too_short = false;
        for (int i = 0; i < 4; i++) {
            // Calculate angles of all edge lines
            float y_length = scene_corners[edge_end_ind[i]].y - scene_corners[edge_start_ind[i]].y;
            float x_length = scene_corners[edge_end_ind[i]].x - scene_corners[edge_start_ind[i]].x;
            angles[i] = std::atan2(y_length, x_length);
            
            // Determine if edge is long enough for to be considered as a potential pick candidate
            float edge_magnitude = std::sqrt(std::pow(x_length,2) + std::pow(y_length,2));
            if (edge_magnitude < (float) min_parallelogram_edge_length) {
                edge_too_short = true;
            }
        }
        if (edge_too_short) {
            continue;
        }
        if ( std::fabs(angles[0] - angles[1]) > parallel_angle_threshold_rad) {
            continue;
        }
        if ( std::fabs(angles[2] - angles[3]) > parallel_angle_threshold_rad) {
            continue;
        }

        // Check that angles are within set tolerance of 90 degrees
        float right_angle_threshold_rad = right_angle_threshold * M_PI / 180.0;
        float right_angle = 90 * M_PI / 180.0;
        float top_left_corner_angle = std::fabs(angles[0] - angles[2]);
        float bot_right_corner_angle = std::fabs(angles[1] - angles[3]);
        if (top_left_corner_angle > (right_angle + right_angle_threshold_rad) || top_left_corner_angle < (right_angle - right_angle_threshold_rad)) {
            continue;
        }
        if (bot_right_corner_angle > (right_angle + right_angle_threshold_rad) || bot_right_corner_angle < (right_angle - right_angle_threshold_rad)) {
            continue;
        }

        // Draw object boundaries in scene image
        cv::Scalar line_color = cv::Scalar(0, 255, 0);
        for (int i = 0; i < 4; i++) {
            if (draw_feature_matches) {
                cv::line(matches_image, scene_corners[i] + cv::Point2f((float)object_reference_image.cols, 0),
                     scene_corners[(i + 1) % 4] + cv::Point2f((float)object_reference_image.cols, 0), line_color, 4);
            } else {
                cv::line(matches_image, scene_corners[i], scene_corners[(i + 1) % 4], line_color, 4);
            }
        }

        // Calculate 2D centroid of pickpoint
        cv::Point2f object_centroid;
        int number_of_corners = 4;
        for (int i = 0; i < number_of_corners; i++) {
            object_centroid.x += scene_corners[i].x;
            object_centroid.y += scene_corners[i].y;
        }
        object_centroid.x = object_centroid.x / (float) number_of_corners;
        object_centroid.y = object_centroid.y / (float) number_of_corners;
        pickpoints_xy_output.push_back(object_centroid);

        // If we made it to the end of this loop, we have at least one detected valid pickpoint
        at_least_one_pickpoint = true;
    }

    // Draw cluster centers if draw_feature_matches is true
    if (draw_feature_matches) {
        for (int i = 0; i < K; i++) {
            cv::Point circle_center;
            if (draw_feature_matches) {
                circle_center = cv::Point(clusters(i, 0) + object_reference_image.cols, clusters(i, 1));
            } else {
                circle_center = cv::Point(clusters(i, 0), clusters(i, 1));
            }
            cv::circle(matches_image, circle_center, 50, cv::Scalar(255,0,255), 3, cv::LINE_AA);
        }
    }
    
    // Edit the original image and return whether at least on pickpoint was returned
    source_img_ptr = matches_image;
    return at_least_one_pickpoint;
}
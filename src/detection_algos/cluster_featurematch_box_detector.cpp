/* This will take two images, one a reference image of an object, and the second
    an image of a scene that contains that object, and use feature matching to locate the
    object in the scene. 

    Ted Lutkus
    6/25/20

    Adapted from tutorial:
    https://docs.opencv.org/3.4/d7/dff/tutorial_feature_homography.html
*/

#include <ros/ros.h>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>
#include <chrono>

#include "lib/KMeansRex_light/src/KMeansRexCore.cpp"
#include "lib/Eigen/Dense"

Eigen::IOFormat CleanFmt(3, 0, " ", "\n", "[", "]");

bool detect_boxes(cv::Mat& source_img_ptr, int hessian_threshold, int K, bool draw_feature_matches) {
    // Start timer to track perception time
    //auto start = chrono::high_resolution_clock::now();

    // Load reference and sampled scene images
    cv::Mat object_reference_image = cv::imread("data/cropped_image.jpg");
    //cv::Mat object_reference_image = cv::imread("cropped_image.jpg");
    //source_img_ptr = cv::imread("top_cereal.jpg");

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
    /* TODO: CONSIDER DOING THIS ON CALIBRATION THEN SAVING */
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(hessian_threshold);
    std::vector<cv::KeyPoint> object_reference_keypoints, sampled_scene_keypoints;
    cv::Mat object_reference_descriptors, sampled_scene_descriptors;
    cv::InputOutputArray detector_mask = cv::noArray();
    detector->detectAndCompute(object_reference_image, detector_mask, object_reference_keypoints, object_reference_descriptors);
    detector->detectAndCompute(source_img_ptr, detector_mask, sampled_scene_keypoints, sampled_scene_descriptors);

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
    // Note that if you increase K in multiples of number of items in scene, you'll eventually reach 1 cluster per item (for top_cereal its 12)
    //int K = 12;
    int point_dim = 2;
    int n_iters = 200;
    int seed = 42;
    int number_of_points = N_rows;
    char* plusplus = (char*) "plusplus";//std::string::c_str("plusplus");
    Eigen::ArrayXXd clusters = Eigen::ArrayXXd::Zero(K, point_dim);
    Eigen::ArrayXd cluster_labels = Eigen::ArrayXd::Zero(number_of_points);

    // Run K-Means algorithm
    RunKMeans(cluster_data.data(), number_of_points, point_dim, K, n_iters, seed, plusplus, clusters.data(), cluster_labels.data());

    // Perform feature matching for each cluster
    std::vector<cv::KeyPoint> scene_keypoints_cluster;
    cv::Mat scene_descriptors_cluster(sampled_scene_descriptors.row(0));
    cv::Mat matches_image;
    bool first_drawmatches = true;
    for (int i = 0; i < K; i++) {
        // Identify number of points in each cluster to initialize size of keypoint/descriptor containers
        int num_points_in_cluster = 0;
        for (int j = 0; j < number_of_points; j++) {
            if (cluster_labels(j,0) == i) {
                num_points_in_cluster++;
            }
        }

        // Extract the cluster subset of keypoints and descriptors from the reference and scene
        scene_keypoints_cluster.resize(num_points_in_cluster);
        scene_descriptors_cluster.resize(num_points_in_cluster);
        int ind = 0;
        for (int j = 0; j < number_of_points; j++) {
            if (cluster_labels(j,0) == i) {
                scene_keypoints_cluster[ind] = sampled_scene_keypoints[j];
                sampled_scene_descriptors.row(j).copyTo(scene_descriptors_cluster.row(ind));
                ind++;
            }
        }
        
        // Matching descriptor vectors with a FLANN based matcher
        int descriptor_matcher_type = cv::DescriptorMatcher::FLANNBASED;
        cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create(descriptor_matcher_type);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        int desired_number_of_matches = 2;
        try {
            descriptor_matcher->knnMatch(object_reference_descriptors, scene_descriptors_cluster, knn_matches, desired_number_of_matches);
        } catch (cv::Exception e) {
            // Clear the keypoints vector
            scene_keypoints_cluster.clear();
            continue;
        }

        // Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.7f;
        std::vector<cv::DMatch> good_matches;
        for (int i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        // Draw matches
        if (draw_feature_matches) {
            const cv::Scalar match_color = cv::Scalar::all(-1);
            const cv::Scalar single_point_color = cv::Scalar::all(-1);
            const std::vector<char> matches_mask = std::vector<char>();
            int drawing_flag = 0;
            if (first_drawmatches) {
                // For first cluster, create new image
                drawing_flag = cv::DrawMatchesFlags::DEFAULT;
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
        for (int i = 0; i < good_matches.size(); i++)
        {
            // Get the keypoints from the good matches
            object_match_keypoints.push_back(object_reference_keypoints[good_matches[i].queryIdx].pt);
            scene_match_keypoints.push_back(scene_keypoints_cluster[good_matches[i].trainIdx].pt);
        }
        cv::Mat object_scene_transform;
        try {
            object_scene_transform = cv::findHomography(object_match_keypoints, scene_match_keypoints, cv::RANSAC);
        } catch (cv::Exception e) {
            // Clear the keypoints vector
            scene_keypoints_cluster.clear();
            continue;
        }

        // Get the object corners from the reference image
        std::vector<cv::Point2f> object_corners(4);
        object_corners[0] = cv::Point2f(0, 0);
        object_corners[1] = cv::Point2f((float)object_reference_image.cols, 0);
        object_corners[2] = cv::Point2f((float)object_reference_image.cols, (float)object_reference_image.rows);
        object_corners[3] = cv::Point2f(0, (float)object_reference_image.rows);
        std::vector<cv::Point2f> scene_corners(4);
        try {
            cv::perspectiveTransform(object_corners, scene_corners, object_scene_transform);
        } catch (cv::Exception e) {
            // Clear the keypoints vector
            scene_keypoints_cluster.clear();
            continue;
        }

        // Draw object boundaries in scene image
        cv::Scalar line_color = cv::Scalar(0, 255, 0);
        for (int i = 0; i < 4; i++)
        {
            if (draw_feature_matches) {
                cv::line(matches_image, scene_corners[i] + cv::Point2f((float)object_reference_image.cols, 0),
                     scene_corners[(i + 1) % 4] + cv::Point2f((float)object_reference_image.cols, 0), line_color, 4);
            } else {
                cv::line(matches_image, scene_corners[i], scene_corners[(i + 1) % 4], line_color, 4);
            }
            
        }

        // Clear the keypoints vector
        scene_keypoints_cluster.clear();
    }

    // Draw cluster centers
    for (int i = 0; i < K; i++) {
        cv::Point circle_center;
        if (draw_feature_matches) {
            circle_center = cv::Point(clusters(i, 0) + object_reference_image.cols, clusters(i, 1));
        } else {
            circle_center = cv::Point(clusters(i, 0), clusters(i, 1));
        }
        cv::circle(matches_image, circle_center, 50, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }

    // Finish timing the detection process
    //auto stop = chrono::high_resolution_clock::now();
    //auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    //ROS_INFO_STREAM << "DETECTION TIME: " << duration.count() / 1000.0 << "[ms]");
    source_img_ptr = matches_image;
    return true;
}
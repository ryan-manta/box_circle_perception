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

void detect_boxes(cv::Mat& source_img_ptr, int hessian_threshold) {
    // Load reference and sampled scene images
    cv::Mat object_reference_image = cv::imread("data/cropped_image.jpg");

    // Check that reference image has data
    if (object_reference_image.empty()){
        ROS_ERROR("Could not open or find object reference image for box detection");
        return;
    }

    //Detect the keypoints using SURF Detector, compute the descriptors
    /* TODO: CONSIDER DOING THIS ON CALIBRATION THEN SAVING */
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(hessian_threshold);
    std::vector<cv::KeyPoint> object_reference_keypoints, sampled_scene_keypoints;
    cv::Mat object_reference_descriptors, sampled_scene_descriptors;
    cv::InputOutputArray detector_mask = cv::noArray();
    detector->detectAndCompute(object_reference_image, detector_mask, object_reference_keypoints, object_reference_descriptors);
    detector->detectAndCompute(source_img_ptr, detector_mask, sampled_scene_keypoints, sampled_scene_descriptors);

    // Matching descriptor vectors with a FLANN based matcher
    /* TODO: USE desired_number_of_matches TO FIND MULTIPLE ITEMS, THEN USE HEURISTIC TO GET CORNER ITEM */
    int descriptor_matcher_type = cv::DescriptorMatcher::FLANNBASED;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create(descriptor_matcher_type);
    std::vector< std::vector<cv::DMatch> > knn_matches;
    int desired_number_of_matches = 2;
    descriptor_matcher->knnMatch(object_reference_descriptors, sampled_scene_descriptors, knn_matches, desired_number_of_matches);

    // Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<cv::DMatch> good_matches;
    for (int i=0; i<knn_matches.size(); i++){
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    // Localize the object
    std::vector<cv::Point2f> object_match_keypoints;
    std::vector<cv::Point2f> scene_match_keypoints;
    for(int i=0; i<good_matches.size(); i++)
    {
        // Get the keypoints from the good matches
        object_match_keypoints.push_back(object_reference_keypoints[good_matches[i].queryIdx].pt);
        scene_match_keypoints.push_back(sampled_scene_keypoints[good_matches[i].trainIdx].pt);
    }
    cv::Mat object_scene_transform = cv::findHomography(object_match_keypoints, scene_match_keypoints, cv::RANSAC);

    // Get the object corners from the reference image
    std::vector<cv::Point2f> object_corners(4);
    object_corners[0] = cv::Point2f(0, 0);
    object_corners[1] = cv::Point2f((float) object_reference_image.cols, 0);
    object_corners[2] = cv::Point2f((float) object_reference_image.cols, (float) object_reference_image.rows);
    object_corners[3] = cv::Point2f(0, (float) object_reference_image.rows);
    std::vector<cv::Point2f> scene_corners(4);
    cv::perspectiveTransform(object_corners, scene_corners, object_scene_transform);

    // Draw object boundaries in scene image
    cv::Scalar line_color = cv::Scalar(0, 255, 0);
    for (int i=0; i<4; i++) {
        cv::line(source_img_ptr,  scene_corners[i] + cv::Point2f((float)object_reference_image.cols, 0),
                scene_corners[(i+1) % 4] + cv::Point2f((float)object_reference_image.cols, 0), line_color, 4);
    }
}

void detect_boxes_showmatches(cv::Mat& source_img_ptr, cv::Mat& matches_image, int hessian_threshold)
{
    // Load reference and sampled scene images
    cv::Mat object_reference_image = cv::imread("data/cropped_image.jpg");

    // Check that reference image has data
    if (object_reference_image.empty()){
        ROS_ERROR("Could not open or find object reference image for box detection");
        return;
    }

    //Detect the keypoints using SURF Detector, compute the descriptors
    /* TODO: CONSIDER DOING THIS ON CALIBRATION THEN SAVING */
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(hessian_threshold);
    std::vector<cv::KeyPoint> object_reference_keypoints, sampled_scene_keypoints;
    cv::Mat object_reference_descriptors, sampled_scene_descriptors;
    cv::InputOutputArray detector_mask = cv::noArray();
    detector->detectAndCompute(object_reference_image, detector_mask, object_reference_keypoints, object_reference_descriptors);
    detector->detectAndCompute(source_img_ptr, detector_mask, sampled_scene_keypoints, sampled_scene_descriptors);

    // Matching descriptor vectors with a FLANN based matcher
    /* TODO: USE desired_number_of_matches TO FIND MULTIPLE ITEMS, THEN USE HEURISTIC TO GET CORNER ITEM */
    int descriptor_matcher_type = cv::DescriptorMatcher::FLANNBASED;
    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create(descriptor_matcher_type);
    std::vector< std::vector<cv::DMatch> > knn_matches;
    int desired_number_of_matches = 2;
    descriptor_matcher->knnMatch(object_reference_descriptors, sampled_scene_descriptors, knn_matches, desired_number_of_matches);

    // Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<cv::DMatch> good_matches;
    for (int i=0; i<knn_matches.size(); i++){
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    // Draw matches
    const cv::Scalar match_color = cv::Scalar::all(-1);
    const cv::Scalar single_point_color = cv::Scalar::all(-1);
    const std::vector<char> matches_mask = std::vector<char>();
    int drawing_flag = cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;
    cv::drawMatches(object_reference_image, object_reference_keypoints, source_img_ptr, sampled_scene_keypoints,
                good_matches, matches_image, match_color,
                single_point_color, matches_mask, drawing_flag);    

    // Localize the object
    std::vector<cv::Point2f> object_match_keypoints;
    std::vector<cv::Point2f> scene_match_keypoints;
    for(int i=0; i<good_matches.size(); i++)
    {
        // Get the keypoints from the good matches
        object_match_keypoints.push_back(object_reference_keypoints[good_matches[i].queryIdx].pt);
        scene_match_keypoints.push_back(sampled_scene_keypoints[good_matches[i].trainIdx].pt);
    }
    cv::Mat object_scene_transform = cv::findHomography(object_match_keypoints, scene_match_keypoints, cv::RANSAC);

    // Get the object corners from the reference image
    std::vector<cv::Point2f> object_corners(4);
    object_corners[0] = cv::Point2f(0, 0);
    object_corners[1] = cv::Point2f((float) object_reference_image.cols, 0);
    object_corners[2] = cv::Point2f((float) object_reference_image.cols, (float) object_reference_image.rows);
    object_corners[3] = cv::Point2f(0, (float) object_reference_image.rows);
    std::vector<cv::Point2f> scene_corners(4);
    cv::perspectiveTransform(object_corners, scene_corners, object_scene_transform);

    // Draw object boundaries in scene image
    cv::Scalar line_color = cv::Scalar(0, 255, 0);
    for (int i=0; i<4; i++) {
        cv::line(matches_image,  scene_corners[i] + cv::Point2f((float)object_reference_image.cols, 0),
                scene_corners[(i+1) % 4] + cv::Point2f((float)object_reference_image.cols, 0), line_color, 4);   
    }
}
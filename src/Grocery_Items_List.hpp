#include <iostream>
#include <fstream>
#include <string>

using namespace std;

struct grocery_item {
    string item;
    int	   type;
    string image;
    int	   hessian_threshold;
    int	   K;
    int	   parallel_angle_threshold;
    int	   min_parallelogram_edge_length;
    int	   right_angle_threshold;
    int	   min_circle_dist;
    int	   canny_edge_detector_thresh;
    int	   hough_accumulator_thresh;
    int	   circle_radius;
    int	   circle_radius_perc_tolerance;
};

void get_Grocery_Items(grocery_item items[], ifstream items_list, int lines);

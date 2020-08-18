#include "Grocery_Items_List.hpp"

void get_Grocery_Items(grocery_item items[], ifstream items_list, int lines) {
    string stoi_temp;

    for (int i = 0; i < lines; i++) {
        getline(items_list, items[i].item, ',');
        getline(items_list, stoi_temp, ',');
        items[i].type = stoi(stoi_temp);
        getline(items_list, items[i].image);
        getline(items_list, stoi_temp, ',');
        items[i].hessian_threshold = stoi(stoi_temp);
        getline(items_list, stoi_temp, ',');
        items[i].K = stoi(stoi_temp);
        getline(items_list, stoi_temp, ',');
        items[i].parallel_angle_threshold = stoi(stoi_temp);
        getline(items_list, stoi_temp, ',');
        items[i].min_parallelogram_edge_length = stoi(stoi_temp);
        getline(items_list, stoi_temp, ',');
        items[i].right_angle_threshold = stoi(stoi_temp);
        getline(items_list, stoi_temp, ',');
        items[i].min_circle_dist = stoi(stoi_temp);
        getline(items_list, stoi_temp, ',');
        items[i].canny_edge_detector_thresh = stoi(stoi_temp);
        getline(items_list, stoi_temp, ',');
        items[i].hough_accumulator_thresh = stoi(stoi_temp);
        getline(items_list, stoi_temp, ',');
        items[i].circle_radius = stoi(stoi_temp);
        getline(items_list, stoi_temp, ',');
        items[i].circle_radius_perc_tolerance = stoi(stoi_temp);
    }

    items_list.close();
}

# green_pick_ros

## Dependencies:
1. OpenCV 2.4.9 (Needs installation)
    - https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

2. PCL (Needs installation)
    - https://pointclouds.org/
    - sudo apt install libpcl-dev

2. Eigen 3 (Needs installation)
    - https://gitlab.com/libeigen/eigen.git

3. KMeansRex (Included and trimmed down from original)
    - https://github.com/tedlutkus/KMeansRex.git

4. Realsense-ros (Needs installation)
    - https://github.com/IntelRealSense/realsense-ros

## Currently Implemented:
### State Machine
- Handles timing of station events including perception actions
    and robot movement

### Perception Pipeline
- Handles conversion between ROS Image messages and OpenCV image objects
- Provides ROS service (/generate_pickpoint) that selects and runs the appropriate detection algorithm based on the desired item input
    - TODO: return the [x, y, z] vector of identified item

### Box Item Calibrator
- Provides GUI for extracting a reference image of a new item to be added to pickable box items
    - TODO: provide ability to name the object and add object to list of pickable items

### Algorithm Calibrator
- Provides GUI for calibrating the parameters of computer vision algorithms by adjusting a sliding bar with real-time feedback of the algorithm's performance displayed

### Detection Algorithms
1. Hough circle detection
2. SURF/FLANN/RANSAC box detection via feature mapping
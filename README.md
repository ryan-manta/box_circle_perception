# Box Circle Detection

## Dependencies:
1. OpenCV 4.4.0
    - Run following command to remove Ubuntu default installed OpenCV 3.2.0
     
     ```
     sudo apt-get autoremove opencv-doc opencv-data libopencv-dev libopencv3.2-java libopencv3.2-jni python-opencv libopencv-core3.2 libopencv-photo3.2 libopencv-contrib3.2 libopencv-imgproc3.2 libopencv-superres3.2 libopencv-stitching3.2 libopencv-ml3.2 libopencv-video3.2 libopencv-videostab3.2 libopencv-objdetect3.2 libopencv-calib3d3.2
     sudo apt-get install libopenexr-dev  
     ```
     
    - https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

2. PCL
    - https://pointclouds.org/
    - sudo apt install libpcl-dev

2. Eigen 3 (Included)
    - https://gitlab.com/libeigen/eigen.git

3. KMeansRex (Included and trimmed down from original)
    - https://github.com/tedlutkus/KMeansRex.git

4. Realsense-ros
    - https://github.com/IntelRealSense/realsense-ros
    
5. pylon_camera
    - http://wiki.ros.org/pylon_camera

## Currently Implemented:
### Perception Pipeline
- Handles conversion between ROS Image messages and OpenCV image objects
- Provides ROS service (/generate_pickpoint) that selects and runs either box or circle detection algorithm based on the item input

### Box Item Calibrator
- Provides GUI for extracting the reference image used in box detection

### Algorithm Calibrator
- Provides GUI for calibrating the parameters of box/circle detection algorithms by adjusting sliders

### Detection Algorithms
1. Hough circle transform for circle detection
2. Cluster-based SURF feature matching for box detection

## Intended Workflow
1. **Box Detection Only:** Obtain reference image of the desired box-like object to be picked using "Box Item Calibrator" ROS node
2. Calibrate algorithms for object set to be picked from using "Algorithm Calibrator" ROS node
3. Use perception pipeline with calibrated algorithm parameters to find pickpoints [x, y, z] of items by passing name of item to pipeline

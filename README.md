# plane_detector
Implementation of the fast plane detector as described in the paper "Fast  Plane  Detection  and  Polygonalization  in  noisy  3D  Range  Images", of Poppinga et al. (IROS 2008)  https://ieeexplore.ieee.org/abstract/document/4650729

## Dependencies

This module has the following dependencies:

- Eigen3
- Boost (array, thread)
- OpenCV
- ROS 

## Usage:

This library can be used standalone without using ROS. In this way the module accepts a OpenCV depth image and outputs the detected plane. To use it in this mode you should use the following methods for include/plane_detector/plane_detector.hpp file:

- PlaneDetector(double delta = 1.0, double epsilon = 1.0, double gamma = 10.0, int theta = 1000, double _std_dev = 0.03); The meaning of each parameter is available on the Popping paper. The std_dev is related to the accuracy of the sensor.

- int detectPlanes(const cv::Mat &depth); (Input: depth image. Output: number of planes).
  
- inline std::vector<DetectedPlane> getPlanes() const; (gets the actual information of the detected planes)

Also, a testing ROS node is provided in src/test_plane_detector.cpp. In this case, the node subscribes to a depth stream from a RGB-D sensor (tested with OpenNI2 compatible devices) and generates markers and outputs the detected planes via stdout. 

Future work should include the output of a specialized plane message for making it easier to use it in other modules. Any help is appreciated!!



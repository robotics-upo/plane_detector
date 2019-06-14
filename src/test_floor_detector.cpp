#include "plane_detector/floor_detector.hpp"
#include "ros/ros.h"
#include <string>
#include <vector>

using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

ros::Publisher marker_pub;

std::string link_name("/front_link");

bool initialized = false;
double scale = 1.0;

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "test_floor_detector");
  
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");
  
  FloorDetector fd(nh, pn);
  
  ros::spin();
}


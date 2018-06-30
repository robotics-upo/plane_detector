#include "plane_detector/wall_detector.hpp"
#include "ros/ros.h"
#include <string>
#include <vector>

using namespace std;
 
ros::Publisher marker_pub;

std::string link_name("/front_link");

bool initialized = false;
double scale = 1.0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_wall_detector");
  
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");
  
  WallDetector fd(nh, pn);
  
  ros::spin();
}



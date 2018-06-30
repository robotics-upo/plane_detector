#include "plane_detector/plane_detector.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include "ros/ros.h"
#include <limits>
#include <string>
#include <vector>
#include <fstream>

using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

void imgCallback(const sensor_msgs::ImageConstPtr &im);
void infoCallback(const sensor_msgs::CameraInfoConstPtr &info);

PlaneDetector *p_det = NULL;
ros::Publisher marker_pub;
ros::Publisher pointcloud_pub;

std::string link_name("/front_link");

bool initialized = false;
double scale = 1.0;

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "test_plane_detector");
  std::string camera("/front/depth_registered");
  double delta = 10.0, epsilon = 3.0 , gamma = 10.0;
  int theta = 10000;
  
  
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");
  
  if (argc > 1) {
    try {
      ifstream ifs(argv[1]);
      ifs >> delta >> epsilon >> gamma >> theta; 
    } catch (exception &e) {
      cerr << "Could not load parameter file";
    }
  }
  
  ROS_INFO("Parameters: %f, %f, %f, %d", delta, epsilon, gamma, theta);
//   ROS_INFO("Std dev of the sensor: %d", std_dev);
  
  pn.getParam("camera", camera);
  pn.getParam("link_name", link_name);
  pn.getParam("scale", scale);
  
  std::string depth_topic = camera + "/image_raw";
  std::string info_topic = camera + "/camera_info";

  ros::Subscriber im_info_sub = nh.subscribe(info_topic, 2, infoCallback);
  ros::Subscriber im_subs = nh.subscribe(depth_topic, 2, imgCallback);
  marker_pub = nh.advertise<visualization_msgs::Marker>("plane_marker", 2);
  pointcloud_pub = nh.advertise<sensor_msgs::PointCloud>("plane_pointcloud", 2);
  
  p_det = new PlaneDetector(nh, pn);
  
  ros::spin();
}


void imgCallback(const sensor_msgs::ImageConstPtr& im)
{
  if (initialized) {
    p_det->detectPlanes(*im);
    
    ROS_INFO("In imgCallback!!");
    
    printPlanes(p_det->getPlanes());
    
//     for (int i = 0; i < p_det->getPlanes().size(); i++) 
//     {
//       const DetectedPlane &p = p_det->getPlanes().at(i);
//       marker_pub.publish(p.getMarker(link_name, i, scale));
//     }
    p_det->publishMarkers(marker_pub, link_name);
    p_det->publishPointCloud(pointcloud_pub, link_name);
  }
  
  
}

void infoCallback(const sensor_msgs::CameraInfoConstPtr& info)
{
  if (info != NULL && !initialized) {
    p_det->setCameraParameters(info->K);
    initialized = true;
  }
}



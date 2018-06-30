#ifndef __WALL_DETECTOR_HPP
#define __WALL_DETECTOR_HPP

#include "plane_detector.hpp"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h> 
#include <plane_detector/WallInfo.h>

//! @brief Detects the floor plane and its neighbors and classifies them according to the height with respect to the floor.
//! @brief It assumes that the camera has been calibrated with respect to the robot.
class WallDetector:public PlaneDetector {
public:
  //! @brief ROS constructor
  WallDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  //! @brief NON-ROS constructor
  WallDetector(double delta, double epsilon, double gamma, int theta, const std::string &cam, const Eigen::Affine3d &T);
  
  ~WallDetector();
  
  //! @brief Detects the floor and generates an altitude map (local)
  void detectWalls(const sensor_msgs::Image &img);
  
protected:
  int _skip, _skip_cont;
  
  void getTransformFromTF();
  
  void infoCallback_1(const sensor_msgs::CameraInfoConstPtr &info);
  void imgCallback_1(const sensor_msgs::ImageConstPtr &img);
  
  std::string link_1, link_2; // Link1 --> robot. Link2 --> camera
  std::string cam_1;
  std::string img_topic_1, info_topic_1, marker_topic, wall_info_topic;
  tf::TransformListener tfListener;
  
  // Camera info subscribers and initialization
  ros::Subscriber info_sub_1, img_sub_1;
  ros::Publisher state_pub;
  ros::Publisher marker_pub;
  ros::Publisher wall_info_pub;
  
  // Transforms
  Eigen::Affine3d T;
  Eigen::Affine3d T_inv;
  
};

WallDetector::~WallDetector()
{
}


WallDetector::WallDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh): PlaneDetector(nh, pnh)
{
  // Set defaults
  link_1 = "/base_link";
  cam_1 = "/camera";
  pnh.getParam("camera", cam_1);
  link_2 = cam_1 + "_depth_optical_frame";
  
  pnh.getParam("link_1", link_1);
  pnh.getParam("link_2", link_2);
  
  
  if (!pnh.getParam("skip", _skip))
    _skip = 2;
  _skip_cont = 0;
  
  
  img_topic_1 = cam_1 + "/depth_registered/image_raw";
  info_topic_1 = cam_1 + "/depth_registered/camera_info";
  marker_topic = nh.resolveName("/wall_marker");
  wall_info_topic = nh.resolveName("/wall_info");
  
  getTransformFromTF();
  
  info_sub_1 = nh.subscribe(info_topic_1, 2, &WallDetector::infoCallback_1, this);
  img_sub_1 = nh.subscribe(img_topic_1, 2, &WallDetector::imgCallback_1, this);
  
  marker_pub = nh.advertise<visualization_msgs::Marker>(marker_topic, 2);
  wall_info_pub = nh.advertise<plane_detector::WallInfo>(wall_info_topic, 2);
}

WallDetector::WallDetector(double delta, double epsilon, double gamma, int theta, const std::string &cam, const Eigen::Affine3d& T): PlaneDetector(delta, epsilon, gamma, theta), T(T)
{
  // Calculate inverse transform
  T_inv = T.inverse();
  // Set defaults
  link_1 = "/base_link";
  cam_1 = "/camera";
  link_2 = cam_1 + "_depth_optical_frame";
  
  marker_topic = "/wall_marker";
}



void WallDetector::detectWalls(const sensor_msgs::Image &img)
{
  if (detectPlanes(img) < 2) {
    // Could not detect a wall plane --> exit
    ROS_INFO("Wall detector --> Not enough planes" );
    return;
  }
  // Check which of the planes is vertical --> it will give us the position of the walls nearby
  Eigen::Vector3d v_z, v_y;
  v_z(0) = 0.0;
  v_z(1) = 0.0;
  v_z(2) = 1.0;
  
  v_y(0) = 0.0;
  v_y(1) = 1.0;
  v_y(2) = 0.0;
  Plane floor_plane ;
  floor_plane.v = v_z;
  floor_plane.d = 0.0;
  std::vector<DetectedPlane> wall_planes;
  
  double angle_1, angle_2;
  for (unsigned int i = 0; i < _detected_planes.size() ; i++) {
    DetectedPlane p = _detected_planes.at(i);
//     ROS_INFO("Detected plane: %s", p.toString().c_str());
    
    p = p.affine(T_inv);
//     ROS_INFO("Transformed plane: %s", p.toString().c_str());
    if (fabs(p.v(1)) > 0.9 ) {
      // New wall plane detected
//       ROS_INFO("Wall detector: New wall detected: %s", p.toString().c_str() );
      wall_planes.push_back(p);
      Eigen::Vector3d cross = p.v.cross(v_z);
      
      double angle = -atan(cross(1)/cross(0));
//       ROS_INFO("New angle: %f", angle);
      
      if (wall_planes. size() == 1) 
	angle_1 = angle;
      else
	angle_2 = angle;
    }
    
  } 
  
  if (wall_planes.size() != 2) {
    ROS_INFO("Detected %lu wall planes --> not taking decisions", wall_planes.size());
    // Publish wrong info (distances cannot be less than zero)
    plane_detector::WallInfo w_info;
    w_info.d_left = -1.0;
    w_info.d_right = -1.0;
    w_info.angle = -1.0;
    wall_info_pub.publish(w_info);
    return;
  }
  
  // Print the distances  (TODO: orientation)
  DetectedPlane p_left = wall_planes.at(0);
  DetectedPlane p_right = wall_planes.at(1);
  if (p_left.v.dot(v_y) < 0) {
    // TODO: switch if ...
    DetectedPlane aux = p_left;
    p_left = p_right;
    p_right = aux;
  }
//   ROS_INFO("Angle_left = %f \t Angle_right = %f \t Angle = %f", angle_1, angle_2, (angle_1 + angle_2) * 0.5);
  
  // Publish markers
  marker_pub.publish(p_left.getMarker(link_1, 0, color.at(0)));
  marker_pub.publish(p_right.getMarker(link_1, 1, color.at(1)));
  
  plane_detector::WallInfo w_info;
  w_info.d_left = p_left.d;
  w_info.d_right = p_right.d;
  w_info.angle = (angle_1 + angle_2) * 0.5;
  wall_info_pub.publish(w_info);
}

void WallDetector::getTransformFromTF()
{
  ROS_INFO("Waiting for transform between: %s and %s", link_1.c_str(), link_2.c_str());
  tf::StampedTransform tf_;
  try {
    while (!tfListener.waitForTransform( link_1, link_2, ros::Time(0), ros::Duration(0.1))) {
      sleep(1);
    }
  
    bool ok = false;
  
    while (!ok && ros::ok()) {
      
      tfListener.lookupTransform(link_1, link_2, ros::Time(0), tf_);
      ok = true;
    } 
  } catch (tf2::ExtrapolationException &e) {
    ROS_ERROR("Problems with tfs");
    usleep(100000);
  }
  
  // Get rotation
  tf::Quaternion q = tf_.getRotation();
//   std::cout << "Quaternion: " << q.getW() << " " << q.getX() << " " << q.getY() << " " << q.getZ() << std::endl;
  Eigen::Quaterniond q_eigen;
  q_eigen.w() = q.getW();
  q_eigen.x() = q.getX();
  q_eigen.y() = q.getY();
  q_eigen.z() = q.getZ();
  T = Eigen::Affine3d::Identity();
  T.rotate(q_eigen);
  
  tf::Vector3 trans_tf = tf_.getOrigin();
  T.matrix()(0,3) = trans_tf.getX();
  T.matrix()(1,3) = trans_tf.getY();
  T.matrix()(2,3) = trans_tf.getZ();
  
  T_inv = T.inverse();
//   ROS_INFO("Transform OK");
  
  
  std::cout << T.matrix() << std::endl;
  std::cout << T_inv.matrix() << std::endl;
}


void WallDetector::infoCallback_1(const sensor_msgs::CameraInfoConstPtr& info)
{
  if (!isInitialized()) {
    setCameraParameters(info->K);
    ROS_INFO("WallDetector: Camera 1 info OK");
  }
  
}

void WallDetector::imgCallback_1(const sensor_msgs::ImageConstPtr& img)
{
  // Detecting planes with respect to the camera
  detectWalls(*img);
}


#endif

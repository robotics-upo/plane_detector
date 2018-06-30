#ifndef __FLOOR_DETECTOR_HPP
#define __FLOOR_DETECTOR_HPP

#include "plane_detector.hpp"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//! @brief Detects the floor plane and its neighbors and classifies them according to the height with respect to the floor.
//! @brief It assumes that the camera has been calibrated with respect to the robot.
class FloorDetector:public PlaneDetector {
public:
  //! @brief ROS constructor
  FloorDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  //! @brief NON-ROS constructor
  FloorDetector(double delta, double epsilon, double gamma, int theta, const std::string &cam, const Eigen::Affine3d &T);
  
  
  
  ~FloorDetector();
  
  //! @brief Detects the floor and generates an altitude map (local)
  void detectFloor(const sensor_msgs::Image &img);
  
protected:
  
  
  bool getTransformFromTF();
  
  void infoCallback_1(const sensor_msgs::CameraInfoConstPtr &info);
  void imgCallback_1(const sensor_msgs::ImageConstPtr &img);
  
  void calculateRollPitch(const Plane &p, double &roll, double &pitch) const;
  
  inline void prepare_pc() {
    pcd_mod = new sensor_msgs::PointCloud2Modifier(pc);
    // this call also resizes the data structure according to the given width, height and fields
    pcd_mod->setPointCloud2FieldsByString(2, "xyz", "rgb");
    pcd_mod->reserve(100000);
  }
  
  double floor_tolerance, angular_tolerance;
  std::string link_1, link_2; // Link1 --> robot. Link2 --> camera
  std::string cam_1;
  std::string img_topic_1, info_topic_1, pc_topic;
  tf::TransformListener tfListener;
  
  // Camera info subscribers and initialization
  ros::Subscriber info_sub_1, img_sub_1;
  ros::Publisher state_pub;
  ros::Publisher pointcloud_pub;
  
  // Point cloud related data
  sensor_msgs::PointCloud2 pc;
  sensor_msgs::PointCloud2Modifier *pcd_mod;
  
  // Transforms
  Eigen::Affine3d T;
  Eigen::Affine3d T_inv;
  
  // Flags
  bool publish_cloud;
};

FloorDetector::~FloorDetector()
{
  delete pcd_mod;
}


FloorDetector::FloorDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh): PlaneDetector(nh, pnh), pcd_mod(NULL),publish_cloud(true)
{
  // Set defaults
  floor_tolerance = 0.15;
  link_1 = "/base_link";
  cam_1 = "/camera";
  
  pnh.getParam("publish_cloud", publish_cloud);
  pnh.getParam("floor_tolerance", floor_tolerance);
  pnh.getParam("angular_tolerance", angular_tolerance);
  pnh.getParam("camera", cam_1);
  link_2 = cam_1 + "_depth_optical_frame";
  pnh.getParam("link_1", link_1);
  pnh.getParam("link_2", link_2);
  
  
  
  img_topic_1 = cam_1 + "/depth_registered/image_raw";
  info_topic_1 = cam_1 + "/depth_registered/camera_info";
  pc_topic = nh.resolveName("/out_cloud");
  
  
  while (!getTransformFromTF()) {
    sleep(1);
  }
  
  info_sub_1 = nh.subscribe(info_topic_1, 2, &FloorDetector::infoCallback_1, this);
  img_sub_1 = nh.subscribe(img_topic_1, 2, &FloorDetector::imgCallback_1, this);
  
  pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pc_topic, 2);
  
  // Prepare the point cloud
  prepare_pc();
  

}

FloorDetector::FloorDetector(double delta, double epsilon, double gamma, int theta, const std::string &cam, const Eigen::Affine3d& T): PlaneDetector(delta, epsilon, gamma, theta), T(T)
{
  // Calculate inverse transform
  T_inv = T.inverse();
  // Set defaults
  floor_tolerance = 0.15;
  link_1 = "/base_link";
  cam_1 = "/camera";
  link_2 = cam_1 + "_depth_optical_frame";
  
  pc_topic = "/out_cloud";
  
  publish_cloud = false;
  
  prepare_pc();
}



void FloorDetector::detectFloor(const sensor_msgs::Image &img)
{
  if (detectPlanes(img) == 0) {
    // Could not detect a floor plane --> exit
    ROS_INFO("Floor detector --> No planes" );
    
    
    return;
  }
  // Check which of the planes is horizontal and at a altitude --> it will give us the position of the floor
  Eigen::Vector3d v_z;
  v_z(0) = 0.0;
  v_z(1) = 0.0;
  v_z(2) = 1.0;
  std::vector<int> floor_planes;
  for (unsigned int i = 0; i < _detected_planes.size() ; i++) {
    DetectedPlane p = _detected_planes.at(i);
    ROS_INFO("Detected plane: %s", p.toString().c_str());
    
    p = p.affine(T_inv.matrix().block<3,3>(0,0), T_inv.matrix().block<3,1>(0,3));
    ROS_INFO("Transformed plane: %s. \tdot product = %f. \t Distance: %f", p.toString().c_str(), p.v.dot(v_z), p.d);
    if (fabs(p.v.dot(v_z)) > 0.9 && p.d < floor_tolerance) {
      // New floor plane detected
      ROS_INFO("Floor detector: New floor plane detected: %s", p.toString().c_str() );
      ROS_INFO("Original plane: %s", _detected_planes.at(i).toString().c_str() );
      floor_planes.push_back(i);
      double roll, pitch;
      calculateRollPitch(p, roll, pitch);
      ROS_INFO("Roll and pitch %f %f", roll, pitch);
    }
    
  }
  
  if (floor_planes.size() == 0) {
    ROS_INFO("No floor");
//     printPlanes(_detected_planes);
    
  } else {
  
    // Header
    pc.header = img.header;
    
    int size = 0;
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pc, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pc, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pc, "b");
    
    for (unsigned int i = 0; i < img.height * img.width; i++) {
  //     std::cout  << i << std::endl;
      Eigen::Vector3d p = get3DPoint(i);
  //     std::cout  << "After point" << std::endl;
      for (unsigned int j = 0; j < floor_planes.size(); j++) {
        if ( _detected_planes.at(floor_planes.at(j)).distance(p) < floor_tolerance ) {
          // Publish point if: TODO: check wether its a floor plane or its an unidentified plane
          size++;
          
        }
      }
    }
    
    pcd_mod->resize(size);
    
    for (unsigned int i = 0; i < img.height * img.width; i++) {
      Eigen::Vector3d p = get3DPoint(i);
      for (unsigned int j = 0; j < floor_planes.size(); j++) {
        if ( _detected_planes.at(floor_planes.at(j)).distance(p) < floor_tolerance ) {
          // Publish point if: TODO: check wether its a floor plane or its an unidentified plane
          *iter_x = p(0);
          *iter_y = p(1);
          *iter_z = p(2);
          int c = status_vec.at(i)%color.size();
          *iter_r = color[c](0);
          *iter_g = color[c](1);
          *iter_b = color[c](2);        
          ++iter_x;++iter_y;++iter_z;
          ++iter_r;++iter_g;++iter_b;
        }
      }
    }
    if (publish_cloud) {
      pointcloud_pub.publish(pc);
    } else {
      ROS_INFO("Cloud size: %d", size);
    }
  }
}

bool FloorDetector::getTransformFromTF()
{
  ROS_INFO("Waiting for transform between: %s and %s", link_1.c_str(), link_2.c_str());
  tf::StampedTransform tf_;
  while (!tfListener.waitForTransform(link_1, link_2, ros::Time(0), ros::Duration(1.0)) && ros::ok()) {
      sleep(1);
  }
    try {
      tfListener.lookupTransform(link_1, link_2, ros::Time(0), tf_);
    } catch (std::exception &e) {
      return false;
    }
  
  ROS_INFO("Got transform");
  
  
  
  // Get rotation
  tf::Quaternion q = tf_.getRotation();
  std::cout << "Quaternion: " << q.getW() << " " << q.getX() << " " << q.getY() << " " << q.getZ() << std::endl;
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
  
   std::cout << "Translation: " << trans_tf.getX() << " " << trans_tf.getY() << " " << trans_tf.getZ() << std::endl;
  
  T_inv = T.inverse();
  
  std::cout << T.matrix() << std::endl;
  std::cout << T_inv.matrix() << std::endl;
  return true;
}


void FloorDetector::infoCallback_1(const sensor_msgs::CameraInfoConstPtr& info)
{
  if (!isInitialized()) {
    setCameraParameters(info->K);
    ROS_INFO("FloorDetector: Camera 1 info OK");
  }
  
}

void FloorDetector::imgCallback_1(const sensor_msgs::ImageConstPtr& img)
{
  // Detecting planes with respect to the camera
  detectFloor(*img);
}

void FloorDetector::calculateRollPitch(const Plane& p, double& roll, double& pitch) const
{
  pitch = asin(p.v[1]);
  roll = acos(p.v[2]/sqrt(1 - pow(p.v[1], 2)));
}


#endif

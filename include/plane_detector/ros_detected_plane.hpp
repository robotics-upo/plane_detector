#ifndef _ROS_DETECTED_PLANE_HPP__
#define _ROS_DETECTED_PLANE_HPP__

#include "detected_plane.hpp"
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Jacobi>

class DetectedPlaneROS:public DetectedPlane {
public:
  DetectedPlaneROS();
  DetectedPlaneROS(const DetectedPlane &p);
  visualization_msgs::Marker getMarker(const std::string &frame_id, int id, double scale = 1.0, double lifetime = 0.1) const;
  visualization_msgs::Marker getMarker(const std::string &frame_id, int id, const Eigen::Vector3d &c, double scale = 1.0, double lifetime = 0.1) const;
};

visualization_msgs::Marker DetectedPlaneROS::getMarker(const std::string &frame_id, int id, double scale, double lifetime) const
{
  return getMarker(frame_id, id, Eigen::Vector3d(0,1,0), scale, lifetime);
  
}

DetectedPlaneROS::DetectedPlaneROS():DetectedPlane()
{

}


DetectedPlaneROS::DetectedPlaneROS(const DetectedPlane& p)
{
   // Detecting data
  r_g = p.r_g; // Center of gravity
  s_g = p.s_g; // Cumulative sum of points (r_g = s_k/n_points)
  m_k = p.m_k; // Matrix to estimate n and v (n = eigen vector related to minimum eigenval)
  p_k = p.m_k; // P_k = sum(v_k*v_k')
  S = p.S; // Scatter matrix (filled by plane detector)
  mse = p.mse; // Minimum square error of estimation
  n_points = p.n_points; // Number of points 
  cov = p.cov;
  weight = p.weight;
  this->d = p.d;
  this->v = p.v;
}


visualization_msgs::Marker DetectedPlaneROS::getMarker(const std::string &frame_id, int id, const Eigen::Vector3d &c, double scale, double lifetime) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "detected_planes";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = r_g(0);
  marker.pose.position.y = r_g(1);
  marker.pose.position.z = r_g(2);
  
  Eigen::Vector3d v_aux;
  v_aux(0) = 1.0; v_aux(1) = v_aux(2) = 0.0;
  
  Eigen::Vector3d axis_rot = v_aux.cross(v);
  
  double cos_angle = v.dot(v_aux);
  double sin_half = sqrt((1-cos_angle)/2);
  
//   std::cout << axis_rot.transpose() << "\t Sin_half: " << sin_half << "\t cos: " << cos_angle << std::endl;
  
  if (axis_rot.norm() > 5e-2) {
    axis_rot.normalize();
    marker.pose.orientation.x = axis_rot(0) * sin_half;
    marker.pose.orientation.y = axis_rot(1) * sin_half;
    marker.pose.orientation.z = axis_rot(2) * sin_half;
    marker.pose.orientation.w = sqrt(1 - sin_half*sin_half);
  } else {
    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
  }
  marker.scale.x = 0.01 * scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.a = 0.5; // Don't forget to set the alpha!
  marker.color.r = c(0);
  marker.color.g = c(1);
  marker.color.b = c(2);
  marker.lifetime = ros::Duration(lifetime);
  
  return marker;
}

#endif
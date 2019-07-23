#ifndef ROS_PLANE_DETECTOR__
#define ROS_PLANE_DETECTOR__

#include <vector>
#include <random>
#include <queue>
#include "detected_plane.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include <dynamic_reconfigure/server.h>
#include <plane_detector/DetectorConfig.h>
#include <plane_detector/plane_detector.hpp>

#include <plane_detector/ros_detected_plane.hpp>

#include <exception>

#define __MIN_RANGE__ 0.3
#define __MAX_RANGE__ 10.0

//! @class PlaneDetectorROS
//! @brief Implements a fast plane detector on RGB-D images as presented in Poppinga IROS 2008
class PlaneDetectorROS:public PlaneDetector {
public:
  //! Non ROS constructor
  PlaneDetectorROS(double delta = 1.0, double epsilon = 1.0, double gamma = 10.0, int theta = 1000, double _std_dev = 0.03);
  
  //! ROS constructor
  PlaneDetectorROS(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  //! @brief Detects the planes inside the image
  int detectPlanes(const sensor_msgs::Image &depth);
  
  void publishMarkers(ros::Publisher &pub, const std::string &frame_id, double lifetime = 0.1) const;
  
  void publishPointCloud(ros::Publisher& pub, const std::string& frame_id, double lifetime = 0.1);
  
protected:
  cv_bridge::CvImageConstPtr _cvbDepth;
  
  // Dynamic reconfigure stuff
  typedef dynamic_reconfigure::Server<plane_calibrator::DetectorConfig> ReconfigureServer;
  void parametersCallback(plane_calibrator::DetectorConfig &config, uint32_t level);  
  boost::shared_ptr<ReconfigureServer> _reconfigure_server;
  bool _config_init;
  ReconfigureServer::CallbackType _call_type;  
  plane_calibrator::DetectorConfig _config;
  void dynamicReconfigureUpdate();
  
  sensor_msgs::PointCloud coloured_cloud;
};

PlaneDetectorROS::PlaneDetectorROS(double delta, double epsilon, double gamma, int theta, double _std_dev): PlaneDetector(delta, epsilon, gamma, theta, _std_dev)
{

}


PlaneDetectorROS::PlaneDetectorROS(ros::NodeHandle& nh, ros::NodeHandle& pnh):PlaneDetector(),_reconfigure_server()
{
  
  pnh.getParam("std_dev", _std_dev);
  
  _reconfigure_server.reset(new ReconfigureServer(pnh));
  _call_type = boost::bind(&PlaneDetectorROS::parametersCallback, this, _1, _2);
  _reconfigure_server->setCallback(_call_type);
  
  while (!_config_init)
  {
    ROS_INFO("Waiting for dynamic reconfigure configuration.");
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
  ROS_DEBUG("Dynamic reconfigure configuration received.");
}

int PlaneDetectorROS::detectPlanes(const sensor_msgs::Image &depth)
{
  if (!_initialized) {
    // The camera parameters have to be set before detecting planes
    return 0;
  }
  try
  {
    _cvbDepth.reset();
    _cvbDepth = cv_bridge::toCvCopy(depth);      
  }
  catch(cv_bridge::Exception& e)
  {
    std::cerr << "[PlaneDetector] cv_bridge exception: " << e.what() << std::endl;
    return 0;
  }
  
  if(depth.encoding.c_str() == sensor_msgs::image_encodings::TYPE_32FC1)
    _float_image = true;
  else if(depth.encoding.c_str() == sensor_msgs::image_encodings::TYPE_16UC1)
    _float_image = false;
  else
  {
    std::cerr << "[Plane detector ROS] Unsupported depth image enconding. Supported encodings are 32FC1 and 16UC1";
    return 0;
  }
  
  return PlaneDetector::detectPlanes(_cvbDepth->image);
}

void PlaneDetectorROS::publishMarkers(ros::Publisher &pub, const std::string &frame_id, double lifetime) const
{
  for (unsigned int i = 0; i < _detected_planes.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "detected_planes";
    marker.id = i;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    for (int j = 0; j < 9; j++) 
    {
      Eigen::Vector3d v = get3DPoint(getRandomPixel(_detected_ids.at(i)));
      geometry_msgs::Point p;
      p.x = v(0);
      p.y = v(1);
      p.z = v(2);
      marker.points.push_back(p);
    }
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 0.5; // Don't forget to set the alpha!
    marker.color.r = _color[i%_color.size()](1);
    marker.color.g = _color[i%_color.size()](0);
    marker.color.b = _color[i%_color.size()](2);
    marker.lifetime = ros::Duration(lifetime);
    
//     pub.publish(marker);
    
    DetectedPlaneROS p(_detected_planes[i]);
    pub.publish(p.getMarker(frame_id, i, 1.0, lifetime));
  }
}

void PlaneDetectorROS::publishPointCloud(ros::Publisher &pub, const std::string &frame_id, double lifetime)
{
  
  sensor_msgs::ChannelFloat32 channel_point;
  channel_point.values.resize(1);
  coloured_cloud.header.frame_id = frame_id;
  coloured_cloud.header.stamp = ros::Time();
  channel_point.name = "rgb"; 
  unsigned int limit = _width*_height;
  coloured_cloud.channels.reserve(limit);
  coloured_cloud.points.reserve(limit);
  if (_downsample)
    limit = limit /4;
//   coloured_cloud.points.resize(limit);      
//   coloured_cloud.channels.resize(limit);      
  geometry_msgs::Point32 p;
  size_t cont = 0;
  for (unsigned int j = 0; j <  limit; j++)
  {
    try {
      Eigen::Vector3d v = get3DPoint(j);
      
      
      p.x = v(0);
      p.y = v(1);
      p.z = v(2);
      coloured_cloud.points.push_back(p);

  //       ROS_INFO("%d iteracion",j);

      uint colors[3]; 
      switch(_status_vec[j])
      {
	case 0: colors[0] = 0; colors[1] = 255; colors[2] = 255; break;
	case 1: colors[0]  = 255; colors[1] = 0; colors[2] = 255; break;
	case 2: colors[0]  = 255; colors[1] = 255; colors[2] = 0; break;
	case 3: colors[0]  = 0; colors[1] = 0; colors[2] = 255; break;
	case 4: colors[0]  = 255; colors[1] = 0; colors[2] = 0; break;
	case 5: colors[0]  = 0; colors[1] = 255; colors[2] = 0; break;
	case 6: colors[0]  = 125; colors[1] = 255; colors[2] = 0; break;
	case 7: colors[0]  = 0; colors[1] = 125; colors[2] = 255; break;
	case 8: colors[0]  = 255; colors[1] = 125; colors[2] = 125; break;
	case 9: colors[0]  = 0; colors[1] = 125; colors[2] = 0; break;
	default: colors[0]  = 255; colors[1] = 255; colors[2] = 255; break;	
      }
	    
      channel_point.values[0] = colors[0]<< 16 | colors[1]<< 8 | colors[2];
      
      coloured_cloud.channels.push_back(channel_point);
      cont ++;
//       coloured_cloud.channels[j] = channel_point;
    } catch (UnknownDepthException &e) {
//       ROS_INFO("PublishPointCloud --> UnknownDepth");
    }
    
      
      
  }
  coloured_cloud.channels.resize(cont);
  coloured_cloud.points.resize(cont);
   
   ROS_INFO("Published clouds. Points: %d", (int)coloured_cloud.points.size());
  pub.publish(coloured_cloud);
}




void PlaneDetectorROS::dynamicReconfigureUpdate()
{
//   plane_calibrator::DetectorConfig config;
  _config.delta = _delta;
  _config.epsilon = _epsilon;
  _config.gamma = _gamma;
  _config.theta = _theta;
  _config.downsample = _downsample;
}

void PlaneDetectorROS::parametersCallback(plane_calibrator::DetectorConfig& config, uint32_t level)
{
  
  _delta = config.delta;
  _epsilon = config.epsilon;
  _gamma = config.gamma;
  _theta = config.theta;
  _downsample = config.downsample;
  _config_init = true;
  ROS_INFO("In parameter callback. Delta = %f. Epsilon = %f. Gamma = %f. Theta = %d", _delta, _epsilon, _gamma, _theta);
}


#endif

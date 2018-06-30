#ifndef PLANE_DETECTOR__
#define PLANE_DETECTOR__

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

enum PixelStatus {
  UNPROCESSED = -3, IN_R_PRIMA, IN_QUEUE
};

//! @class PlaneDetector
//! @brief Implements a fast plane detector on RGB-D images as presented in Poppinga IROS 2008
class PlaneDetector {
public:
  //! Recommended constructor
  PlaneDetector(double delta = 1.0, double epsilon = 1.0, double gamma = 10.0, int theta = 1000, double std_dev = 0.03);
  
  //! ROS constructor
  PlaneDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  
  //! @brief Detects the planes inside the image
  int detectPlanes(const sensor_msgs::Image &depth);
  
  inline std::vector<DetectedPlane> getPlanes() const {
    return _detected_planes;
  }
  
  void resetStatus();
  
  inline int getPos(int i, int j) const {
    return i + j * width;
  }
  
  void setCameraParameters(const boost::array< double, 9 >& k_);
  
  void publishMarkers(ros::Publisher &pub, const std::string &frame_id, double lifetime = 0.1) const;
  
  void publishPointCloud(ros::Publisher &pub, const std::string &frame_id, double lifetime = 0.1) const;
   
  
  
  inline bool isInitialized() {return _initialized;}
  
protected:
  double _delta, _epsilon, _gamma; // Dynamically reconfigurable
  int _theta;
  std::vector <DetectedPlane> _detected_planes; // Vector de IDs
  std::vector <int> _detected_ids; // Contiene todos los IDs que son considerados planos
  std::vector<int> status_vec; // Relaciona los puntos de la imagen con una region
  std::queue<int> q;
  cv_bridge::CvImageConstPtr cvbDepth;
  int _available_pixels;
  bool _initialized;
  bool _downsample;
  double std_dev;

  std::vector<int> curr_region; // Saves the region in coordinates --> i + j * height
  DetectedPlane curr_plane;
  
  // Internal stuff for update Matrices
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  DetectedPlane p;
  
  // Attributes of the image
  int width, height;
  bool float_image;
  
  // Camera parameters
  Eigen::Matrix3d K;
  Eigen::Vector3d D;
  float kx, cx, ky , cy; // Taken from camera parameters
  
  std::vector<Eigen::Vector3d> color;
  
  int getRandomPixel(int region = (int)UNPROCESSED) const;
  //! @brief Gets the unprocessed nearest neighbor of the image
  int getNearestNeighbor(int index) const;
  //! @brief Gets the amount of available pixels in image
  int availablePixels() const;
  
  // Dynamic reconfigure stuff
  typedef dynamic_reconfigure::Server<plane_calibrator::DetectorConfig> ReconfigureServer;
  void parametersCallback(plane_calibrator::DetectorConfig &config, uint32_t level);  
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  bool config_init_;
  ReconfigureServer::CallbackType call_type;  
  plane_calibrator::DetectorConfig config;
  void dynamicReconfigureUpdate();
  
  // For random numbers
  static std::default_random_engine generator;
  
  //! @brief Adds pixels to region and adds its nearest neighbors to the queue (modifying status_vec)
  void addPixelToRegion(int index, int curr_region_id);
  
  Eigen::Vector3d get3DPoint(int i, int j) const;
  Eigen::Vector3d get3DPoint(int index) const;
  //! @brief Gets the depth of a depth image
  //! @return THe depth value or -1.0 if the data is not valid
  double getDepth(int i, int j) const;
  
  //! @brief updates s_g, m_k, p_k and MSE
  //! @retval true The conditions of step 8 of the matrix (see [1]) are met --> the matrices are updated
  bool  updateMatrices(const Eigen::Vector3d &v);
  
  //! @brief Initialize the color matrix
  void initializeColors();
};

std::default_random_engine PlaneDetector::generator;

PlaneDetector::PlaneDetector(double delta, double epsilon, double gamma, int theta, double std_dev):
_delta(delta), _epsilon(epsilon), _gamma(gamma), _theta(theta),_initialized(false), _downsample(true),std_dev(std_dev)
{
  initializeColors();
}

void PlaneDetector::initializeColors() {
  // Different colors for planes
  Eigen::Vector3d v;
  v(0) = 0.0; v(1) = 1.0; v(2) = 0.0;
  color.push_back(v);
  v(0) = 1.0; v(1) = 0.0; v(2) = 0.0;
  color.push_back(v);
  v(0) = 0.0; v(1) = 0.0; v(2) = 1.0;
  color.push_back(v);
  v(0) = 0.0; v(1) = 1.0; v(2) = 1.0;
  color.push_back(v);
  v(0) = 1.0; v(1) = 0.0; v(2) = 1.0;
  color.push_back(v);
  v(0) = 1.0; v(1) = 1.0; v(2) = 0.0;
  color.push_back(v);
  
}
  
PlaneDetector::PlaneDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh):_initialized(false), std_dev(0.03), reconfigure_server_(),config_init_(false)
{
  
  pnh.getParam("std_dev", std_dev);
  
  reconfigure_server_.reset(new ReconfigureServer(pnh));
  call_type = boost::bind(&PlaneDetector::parametersCallback, this, _1, _2);
  reconfigure_server_->setCallback(call_type);
  
  while (!config_init_)
  {
    ROS_INFO("Waiting for dynamic reconfigure configuration.");
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
  ROS_DEBUG("Dynamic reconfigure configuration received.");
  
  initializeColors();
}

int PlaneDetector::detectPlanes(const sensor_msgs::Image &depth)
{
  if (!_initialized) {
    // The camera parameters have to be set before detecting planes
    return 0;
  }
  // First: get parameters of the image (MUST BE AT THE FIRST)
  width = depth.width;
  height = depth.height;
  if (_downsample) {
    width /= 2;
    height /= 2;
  }
  
  try
  {
    cvbDepth.reset();
    cvbDepth = cv_bridge::toCvCopy(depth);      
  }
  catch(cv_bridge::Exception& e)
  {
    std::cerr << "[PlaneDetector] cv_bridge exception: " << e.what() << std::endl;
    return 0;
  }
  
  if(depth.encoding.c_str() == sensor_msgs::image_encodings::TYPE_32FC1)
    float_image = true;
  else if(depth.encoding.c_str() == sensor_msgs::image_encodings::TYPE_16UC1)
    float_image = false;
  else
  {
    std::cerr << "[Plane detector] Unsupported depth image enconding. Supported encodings are 32FC1 and 16UC1";
    return 0;
  }
  
  // All internal status to zero or proper values
  resetStatus();
  q.empty();
  int curr_region_id = 0;
  
  
  _available_pixels = availablePixels();
  while (_available_pixels > _theta * 1.1) 
  {
//     std::cout << "Available pixels: " << _available_pixels << std::endl;
    curr_region.clear();
    // Initialization of the algorithm--> random point and nearest neighbor
    int candidate = getRandomPixel();
    int nearest = getNearestNeighbor(candidate);
    
    if (nearest > 0) 
    {
      status_vec[nearest] = curr_region_id;
      addPixelToRegion(candidate, curr_region_id);
      addPixelToRegion(nearest, curr_region_id);
      
      // Initialize matrices and vectors
      Eigen::Vector3d r_1 = get3DPoint(candidate);
      Eigen::Vector3d r_2 = get3DPoint(nearest);
      curr_plane.s_g = r_1 + r_2;
      curr_plane.m_k = (r_1 - curr_plane.s_g * 0.5)*(r_1 - curr_plane.s_g * 0.5).transpose();
      curr_plane.m_k += (r_2 - curr_plane.s_g * 0.5)*(r_2 - curr_plane.s_g * 0.5).transpose();
      curr_plane.p_k = r_1 * r_1.transpose() + r_2*r_2.transpose();
      curr_plane.n_points = 2;
      
      while (!q.empty()) 
      {
        int new_point = q.front();
        q.pop();
        Eigen::Vector3d v = get3DPoint(new_point);
        if (updateMatrices(v)) {
          addPixelToRegion(new_point, curr_region_id);
        } else {
          _available_pixels--;
        }
      }
      
      // The queue has been emptied --> clear possible QUEUE status and add the region to the detected planes if condition of step 12 (Algorithm 1)
      if (curr_region.size() > _theta) {
        curr_plane.makeDPositive();
        curr_plane.calculateCovariance(std_dev);
//         std::cout << "Detected plane: " << curr_plane.toString() << std::endl;
        _detected_planes.push_back(curr_plane);
        _detected_ids.push_back(curr_region_id);
      }
      curr_region_id++; // The next plane will have a new identificator
    } else {
      // No nearest neighbor available --> discard (to R_PRIMA)
      status_vec[candidate] = (int)IN_R_PRIMA;
      _available_pixels--;
    }
  }
  
  return _detected_planes.size();
}

void PlaneDetector::resetStatus()
{
  if (status_vec.size() != width*height) {
    status_vec.resize(width*height);
  }
  for (size_t i = 0; i < width; i++) {
    for (size_t j = 0; j < height; j++) {
      if (getDepth(i, j) > 0.0) {
        status_vec[getPos(i,j)] = (int)UNPROCESSED;
      } else {
        status_vec[getPos(i,j)] = (int)IN_R_PRIMA;
      }
    }
  }
  
  p.init();
  curr_plane.init();
  
  
  _detected_planes.clear();
  _detected_ids.clear();
}

int PlaneDetector::getRandomPixel(int region) const
{
  std::uniform_int_distribution<int> distribution(0, height * width - 1);
  int ret_val;
  
  do {
    ret_val = distribution(generator);
  } while (status_vec[ret_val] != region);
  
  return ret_val;
}

//! Gets the unprocessed nearest neighbor of a pixel in a window of size 9
int PlaneDetector::getNearestNeighbor(int index) const
{
  int near = -1;
  int aux;
  float min_dist = 1e10;
  
  Eigen::Vector3d v = get3DPoint(index);
  Eigen::Vector3d v_;
  for (int i = -1; i < 2; i++) {
    for (int j = -1; j < 2; j++) {
      if (i != 0 || j != 0) {
        aux = index + i + j * width;
        
        if (aux > 0 && aux < height * width) { // Check bounds
          if (status_vec[aux] == (int)UNPROCESSED) {
            v_ = get3DPoint(aux);
            double dist = (v - v_).norm();
            if (dist < min_dist) {
              near = aux;
              min_dist = dist;
            }
          }
        }
      }
    }
  }
  return near;
}

int PlaneDetector::availablePixels() const
{
  bool ret_val = false;
  int cont = 0;
  
  //TODO: Uncomment
  for (int i = 0; i < width * height /*&& cont < _theta*/; i++) {
    if (status_vec[i] == (int)UNPROCESSED) 
      cont++;
  }
  
//   std::cout << "availablePixels() --> cont = " << cont << std::endl;
  
  return cont;
}

void PlaneDetector::setCameraParameters(const boost::array< double, 9 >& k_)
{
  for (int i = 0; i < 3;i++) 
  {
    for (int j = 0; j < 3; j++) 
    {
      K(i,j) = k_[i*3 + j];
    }
  }
  
  kx = 1.0/K(0,0);
  cx = K(0,2);
  ky = 1.0/K(1,1);
  cy = K(1,2);
  _initialized = true;
}

double PlaneDetector::getDepth(int i, int j) const
{
  double ret_val = -1.0;
  if (_downsample) {
    i *= 2;
    j *= 2;
  }
//   std::cout << "Data: " << cvbDepth->image.at<float>(i,j) << " ";
  if(float_image)
    ret_val = cvbDepth->image.at<float>(j, i); /// CAUTION!!!! the indices in cv::Mat are row, cols (elsewhere cols, rows)
  else
    ret_val = cvbDepth->image.at<u_int16_t>(j, i)*0.001;
  if(ret_val < 0.3 || ret_val > 10.0)
  {
    ret_val = -1.0;
  }
  return ret_val;
}


Eigen::Vector3d PlaneDetector::get3DPoint(int i, int j) const
{
  Eigen::Vector3d pt;
  
  pt[2] = getDepth(i, j);
  if (_downsample) {
    i *= 2;
    j *= 2;
  }
  if(pt[2] > 0.3 && pt[2] < 10.0)
  {
    pt[0] = (float)(i - cx) * pt[2] * kx;
    pt[1] = (float)(j - cy) * pt[2] * ky;
  }

  return pt;
}

Eigen::Vector3d PlaneDetector::get3DPoint(int index) const
{
  return get3DPoint(index%width, index/width);
}
  
void PlaneDetector::addPixelToRegion(int index, int curr_region_id)
{
  status_vec[index] = curr_region_id;
  curr_region.push_back(index);
  Eigen::Vector3d v = get3DPoint(index);
  _available_pixels--;
  
  int neighbor = getNearestNeighbor(index);
  while (neighbor >= 0) {
    Eigen::Vector3d v_2 = get3DPoint(neighbor);
    if ( (v-v_2).norm() < _delta) { // First check --> the neighbor is sufficiently near to the point
      status_vec[neighbor] = (int)IN_QUEUE;
      q.push(neighbor);
      neighbor = getNearestNeighbor(index);
    } else {
      neighbor = -1;
    }
  }
}

bool PlaneDetector::updateMatrices(const Eigen::Vector3d& v)
{
  size_t k = curr_region.size();
  double div_1 = 1.0 / (double)(k + 1);
  double div = 1.0 / (double)(k);
  
  p.s_g = curr_plane.s_g + v;
  p.m_k = curr_plane.m_k + v*v.transpose() - (p.s_g * div_1)*p.s_g.transpose() + (curr_plane.s_g * div) * curr_plane.s_g.transpose();
  p.p_k = curr_plane.p_k + v*v.transpose();
  
  // Calculate d and n (n is the eigenvector related to the lowest eigenvalue)
  
  es.compute(p.m_k);
  double min_eigenval = 1e10;
  int min_i;
  for (int i = 0; i < 3; i++) {
    double curr_eigen = fabs(es.eigenvalues()(i));
    if (curr_eigen < min_eigenval) {
      min_eigenval = curr_eigen;
      min_i = i;
    }
    
  }
  p.v = es.eigenvectors().col(min_i);
  p.d = div_1 * p.v.dot(p.s_g);
  
  // Update the MSE (Eq 3)
  p.mse = div_1 * p.v.transpose() * p.p_k * p.v - 2 * p.v.dot(p.s_g) * p.d * div_1 + p.d * p.d;
  p.n_points = k + 1;
  
  
  // Check if the new plane meets the constraint of algorithm 1 step 8. If so, update the values of the matrices of the class
  if (p.mse < _epsilon && p.distance(v) < _gamma) 
  {
    // Meets the constraints --> Actualize the plane
    p.r_g = p.s_g * div_1;
    curr_plane = p;
    
    return true;
  }
  return false;
}


void PlaneDetector::publishMarkers(ros::Publisher &pub, const std::string &frame_id, double lifetime) const
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
    marker.color.r = color[i%color.size()](1);
    marker.color.g = color[i%color.size()](0);
    marker.color.b = color[i%color.size()](2);
    marker.lifetime = ros::Duration(lifetime);
    
//     pub.publish(marker);
    pub.publish(_detected_planes[i].getMarker(frame_id, i, 1.0, lifetime));
  }
}


void PlaneDetector::publishPointCloud(ros::Publisher &pub, const std::string &frame_id, double lifetime) const
{
  sensor_msgs::PointCloud coloured_cloud;
  sensor_msgs::ChannelFloat32 channel_point;
  channel_point.values.resize(1);
  coloured_cloud.header.frame_id = frame_id;
  coloured_cloud.header.stamp = ros::Time();
  channel_point.name = "rgb"; 
  coloured_cloud.points.resize(width*height);      
  coloured_cloud.channels.resize(width*height);      
  
//  ROS_INFO("Primera fase");
  for (unsigned int j = 0; j <  width*height; j++)
  {
    Eigen::Vector3d v = get3DPoint(j);
    geometry_msgs::Point p;
    coloured_cloud.points[j].x = v(0);
    coloured_cloud.points[j].y = v(1);
    coloured_cloud.points[j].z = v(2);

 //       ROS_INFO("%d iteracion",j);

    uint colors[3]; 
    switch(status_vec[j])
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
      
    coloured_cloud.channels[j] = channel_point;    
      
      
  }
   
  pub.publish(coloured_cloud);
}




void PlaneDetector::dynamicReconfigureUpdate()
{
  plane_calibrator::DetectorConfig config;
  config.delta = _delta;
  config.epsilon = _epsilon;
  config.gamma = _gamma;
  config.theta = _theta;
  config.downsample = _downsample;
}

void PlaneDetector::parametersCallback(plane_calibrator::DetectorConfig& config, uint32_t level)
{
  
  _delta = config.delta;
  _epsilon = config.epsilon;
  _gamma = config.gamma;
  _theta = config.theta;
  _downsample = config.downsample;
  config_init_ = true;
  ROS_INFO("In parameter callback. Delta = %f. Epsilon = %f. Gamma = %f. Theta = %d", _delta, _epsilon, _gamma, _theta);
}


#endif

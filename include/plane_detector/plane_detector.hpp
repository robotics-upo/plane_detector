#ifndef PLANE_DETECTOR__
#define PLANE_DETECTOR__

#include <vector>
#include <random>
#include <queue>
#include "detected_plane.hpp"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <exception>
#include <opencv2/opencv.hpp>
#include <iostream>

#define __MIN_RANGE__ 0.3
#define __MAX_RANGE__ 10.0

struct UnknownDepthException:public std::exception {
  const char *what() const throw()
  {
    return "Unknown Depth Exception";
  }
  
};

enum PixelStatus {
  UNPROCESSED = -3, IN_R_PRIMA, IN_QUEUE
};

//! @class PlaneDetector
//! @brief Implements a fast plane detector on RGB-D images as presented in Poppinga IROS 2008
class PlaneDetector {
public:
  //! Recommended constructor
  PlaneDetector(double delta = 1.0, double epsilon = 1.0, double gamma = 10.0, int theta = 1000, double _std_dev = 0.03);
  
  //! @brief Detects the planes inside the image
  int detectPlanes(const cv::Mat &depth);
  
  inline std::vector<DetectedPlane> getPlanes() const {
    return _detected_planes;
  }
  
  void resetStatus();
  
  inline int getPos(int i, int j) const {
    return i + j * _width;
  }
  
  void setCameraParameters(const boost::array< double, 9 >& k_);
  
  inline bool isInitialized() {return _initialized;}
  
  inline bool setFloatImage(bool new_value) {_float_image = new_value;}
  
protected:
  double _delta, _epsilon, _gamma; // Dynamically reconfigurable
  int _theta;
  std::vector <DetectedPlane> _detected_planes; // Vector de IDs
  std::vector <int> _detected_ids; // Contiene todos los IDs que son considerados planos
  std::vector<int> _status_vec; // Relaciona los puntos de la imagen con una region
  std::queue<int> _q;
  int _available_pixels;
  bool _initialized;
  bool _downsample;
  double _std_dev;
  cv::Mat _image;

  std::vector<int> _curr_region; // Saves the region in coordinates --> i + j * _height
  DetectedPlane _curr_plane;
  
  // Internal stuff for update Matrices
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> _es;
  DetectedPlane _p;
  
  // Attributes of the image
  int _width, _height;
  bool _float_image;
  
  // Camera parameters
  Eigen::Matrix3d _K;
  float _kx, _cx, _ky , _cy; // Taken from camera parameters
  
  std::vector<Eigen::Vector3d> _color;
  
  int getRandomPixel(int region = (int)UNPROCESSED) const;
  //! @brief Gets the unprocessed nearest neighbor of the image
  int getNearestNeighbor(int index) const;
  //! @brief Gets the amount of available pixels in image
  int availablePixels() const;
  
  // For random numbers
  static std::default_random_engine generator;
  
  //! @brief Adds pixels to region and adds its nearest neighbors to the queue (modifying status_vec)
  void addPixelToRegion(int index, int _curr_region_id);
  
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

PlaneDetector::PlaneDetector(double delta, double epsilon, double gamma, int theta, double _std_dev):
_delta(delta), _epsilon(epsilon), _gamma(gamma), _theta(theta),_initialized(false), _downsample(true),_std_dev(_std_dev)
{
  initializeColors();
}

void PlaneDetector::initializeColors() {
  // Different colors for planes
  Eigen::Vector3d v;
  v(0) = 0.0; v(1) = 1.0; v(2) = 0.0;
  _color.push_back(v);
  v(0) = 1.0; v(1) = 0.0; v(2) = 0.0;
  _color.push_back(v);
  v(0) = 0.0; v(1) = 0.0; v(2) = 1.0;
  _color.push_back(v);
  v(0) = 0.0; v(1) = 1.0; v(2) = 1.0;
  _color.push_back(v);
  v(0) = 1.0; v(1) = 0.0; v(2) = 1.0;
  _color.push_back(v);
  v(0) = 1.0; v(1) = 1.0; v(2) = 0.0;
  _color.push_back(v);
  
}
  
int PlaneDetector::detectPlanes(const cv::Mat& image)
{
  if (!_initialized) {
    // The camera parameters have to be set before detecting planes
    return 0;
  }
  // First: get parameters of the image (MUST BE AT THE FIRST)
  _width = image.cols;
  _height = image.rows;
  this->_image = image; // TODO: será lento?
  
  if (_downsample) {
    _width /= 2;
    _height /= 2;
  }
  
  // All internal status to zero or proper values
  resetStatus();
  _q.empty();
  int _curr_region_id = 0;
  
  _available_pixels = availablePixels();
  while (_available_pixels > _theta * 1.1) 
  {
//     std::cout << "Available pixels: " << _available_pixels << std::endl;
    _curr_region.clear();
    // Initialization of the algorithm--> random point and nearest neighbor
    int candidate = getRandomPixel();
    int nearest = getNearestNeighbor(candidate);
    
    if (nearest > 0) 
    {
      _status_vec[nearest] = _curr_region_id;
      addPixelToRegion(candidate, _curr_region_id);
      addPixelToRegion(nearest, _curr_region_id);
      
      // Initialize matrices and vectors
      Eigen::Vector3d r_1 = get3DPoint(candidate);
      Eigen::Vector3d r_2 = get3DPoint(nearest);
      _curr_plane.s_g = r_1 + r_2;
      _curr_plane.m_k = (r_1 - _curr_plane.s_g * 0.5)*(r_1 - _curr_plane.s_g * 0.5).transpose();
      _curr_plane.m_k += (r_2 - _curr_plane.s_g * 0.5)*(r_2 - _curr_plane.s_g * 0.5).transpose();
      _curr_plane.p_k = r_1 * r_1.transpose() + r_2*r_2.transpose();
      _curr_plane.n_points = 2;
      
      while (!_q.empty()) 
      {
        int new_point = _q.front();
        _q.pop();
        Eigen::Vector3d v = get3DPoint(new_point);
        if (updateMatrices(v)) {
          addPixelToRegion(new_point, _curr_region_id);
        } else {
          _available_pixels--;
        }
      }
      
      // The queue has been emptied --> clear possible QUEUE status and add the region to the detected planes if condition of step 12 (Algorithm 1)
      if (_curr_region.size() > _theta) {
        _curr_plane.makeDPositive();
        _curr_plane.calculateCovariance(_std_dev);
//         std::cout << "Detected plane: " << _curr_plane.toString() << std::endl;
        _detected_planes.push_back(_curr_plane);
        _detected_ids.push_back(_curr_region_id);
      }
      _curr_region_id++; // The next plane will have a new identificator
    } else {
      // No nearest neighbor available --> discard (to R_PRIMA)
      _status_vec[candidate] = (int)IN_R_PRIMA;
      _available_pixels--;
    }
  }
  
  return _detected_planes.size();
}

void PlaneDetector::resetStatus()
{
  if (_status_vec.size() != _width*_height) {
    _status_vec.resize(_width*_height);
  }
  for (size_t i = 0; i < _width; i++) {
    for (size_t j = 0; j < _height; j++) {
      if (getDepth(i, j) > 0.0) {
        _status_vec[getPos(i,j)] = (int)UNPROCESSED;
      } else {
        _status_vec[getPos(i,j)] = (int)IN_R_PRIMA;
      }
    }
  }
  
  _p.init();
  _curr_plane.init();
  
  
  _detected_planes.clear();
  _detected_ids.clear();
}

int PlaneDetector::getRandomPixel(int region) const
{
  std::uniform_int_distribution<int> distribution(0, _height * _width - 1);
  int ret_val;
  
  do {
    ret_val = distribution(generator);
  } while (_status_vec[ret_val] != region);
  
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
        aux = index + i + j * _width;
        
        if (aux > 0 && aux < _height * _width) { // Check bounds
          if (_status_vec[aux] == (int)UNPROCESSED) {
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
  for (int i = 0; i < _width * _height /*&& cont < _theta*/; i++) {
    if (_status_vec[i] == (int)UNPROCESSED) 
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
      _K(i,j) = k_[i*3 + j];
    }
  }
  
  _kx = 1.0/_K(0,0);
  _cx = _K(0,2);
  _ky = 1.0/_K(1,1);
  _cy = _K(1,2);
  _initialized = true;
}

double PlaneDetector::getDepth(int i, int j) const 
{
  double ret_val = -1.0;
  if (_downsample) {
    i *= 2;
    j *= 2;
  }
  
  if (i >= _image.cols || j >= _image.rows)
    return -1.0;
//   std::cout << "Data: " << cvbDepth->image.at<float>(i,j) << " ";
  if(_float_image)
    ret_val = _image.at<float>(j, i); /// CAUTION!!!! the indices in cv::Mat are row, cols (elsewhere cols, rows)
  else
    ret_val = _image.at<u_int16_t>(j, i)*0.001;
  if(ret_val < __MIN_RANGE__ || ret_val > __MAX_RANGE__)
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
  if(pt[2] > __MIN_RANGE__ && pt[2] < __MAX_RANGE__)
  {
    pt[0] = (float)(i - _cx) * pt[2] * _kx;
    pt[1] = (float)(j - _cy) * pt[2] * _ky;
  } else {
    throw UnknownDepthException();
  }

  return pt;
}

Eigen::Vector3d PlaneDetector::get3DPoint(int index) const
{
  return get3DPoint(index%_width, index/_width);
}
  
void PlaneDetector::addPixelToRegion(int index, int _curr_region_id)
{
  _status_vec[index] = _curr_region_id;
  _curr_region.push_back(index);
  Eigen::Vector3d v = get3DPoint(index);
  _available_pixels--;
  
  int neighbor = getNearestNeighbor(index);
  while (neighbor >= 0) {
    Eigen::Vector3d v_2 = get3DPoint(neighbor);
    if ( (v-v_2).norm() < _delta) { // First check --> the neighbor is sufficiently near to the point
      _status_vec[neighbor] = (int)IN_QUEUE;
      _q.push(neighbor);
      neighbor = getNearestNeighbor(index);
    } else {
      neighbor = -1;
    }
  }
}

bool PlaneDetector::updateMatrices(const Eigen::Vector3d& v)
{
  size_t k = _curr_region.size();
  double div_1 = 1.0 / (double)(k + 1);
  double div = 1.0 / (double)(k);
  
  _p.s_g = _curr_plane.s_g + v;
  _p.m_k = _curr_plane.m_k + v*v.transpose() - (_p.s_g * div_1)*_p.s_g.transpose() + (_curr_plane.s_g * div) * _curr_plane.s_g.transpose();
  _p.p_k = _curr_plane.p_k + v*v.transpose();
  
  // Calculate d and n (n is the eigenvector related to the lowest eigenvalue)
  
  _es.compute(_p.m_k);
  double min_eigenval = 1e10;
  int min_i;
  for (int i = 0; i < 3; i++) {
    double curr_eigen = fabs(_es.eigenvalues()(i));
    if (curr_eigen < min_eigenval) {
      min_eigenval = curr_eigen;
      min_i = i;
    }
    
  }
  _p.v = _es.eigenvectors().col(min_i);
  _p.d = div_1 * _p.v.dot(_p.s_g);
  
  // Update the MSE (Eq 3)
  _p.mse = div_1 * _p.v.transpose() * _p.p_k * _p.v - 2 * _p.v.dot(_p.s_g) * _p.d * div_1 + _p.d * _p.d;
  _p.n_points = k + 1;
  
  
  // Check if the new plane meets the constraint of algorithm 1 step 8. If so, update the values of the matrices of the class
  if (_p.mse < _epsilon && _p.distance(v) < _gamma) 
  {
    // Meets the constraints --> Actualize the plane
    _p.r_g = _p.s_g * div_1;
    _curr_plane = _p;
    
    return true;
  }
  return false;
}

#endif

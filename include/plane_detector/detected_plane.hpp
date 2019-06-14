#ifndef DETECTED_PLANE_HPP__
#define DETECTED_PLANE_HPP__

#include "plane.hpp"
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Jacobi>
#include <iostream>

#include <vector>

class DetectedPlane:public Plane {
public:
  // Detecting data
  Eigen::Vector3d r_g; // Center of gravity
  // Estimating the plane
  Eigen::Vector3d s_g; // Cumulative sum of points (r_g = s_k/n_points)
  Eigen::Matrix3d m_k; // Matrix to estimate n and v (n = eigen vector related to minimum eigenval)
  Eigen::Matrix3d p_k; // P_k = sum(v_k*v_k')
  Eigen::Matrix3d S; // Scatter matrix (filled by plane detector)
  double mse; // Minimum square error of estimation
  int n_points; // Number of points 
  Eigen::Matrix4d cov;
  double weight;
  
  DetectedPlane affine_inv(const Eigen::Matrix3d& rot_, const Eigen::Vector3d& trans_) const;
  DetectedPlane affine_inv2(const Eigen::Matrix3d& rot_, const Eigen::Vector3d& trans_) const;
  
  DetectedPlane affine(const Eigen::Matrix3d &rot, const Eigen::Vector3d &trans) const;
  
  DetectedPlane affine(const Eigen::Affine3d &T) const
  {
    
    return affine(T.matrix().block<3,3>(0,0), T.matrix().block<3,1>(0,3));
  }
  
  DetectedPlane():Plane() {
    init();
  }
  
  inline void init() {
    n_points = 0;
    mse = 0.0;
    s_g(0) = s_g(1) = s_g(2) = 0.0;
    r_g(0) = r_g(1) = r_g(2) = 0.0;
    p_k(0,0) = p_k(0,1) = p_k(0,2) = 0.0;
    p_k(1,0) = p_k(1,1) = p_k(1,2) = 0.0;
    p_k(2,0) = p_k(2,1) = p_k(2,2) = 0.0;
  }
  
  virtual std::string toString(bool verbose = false) const;
  
  void calculateCovariance(double std_dev = 0.02, bool pathak_way = true);
  
protected:
  //! @brief Composes the covariance of the plane according to Fdez-Jimenez et al (IROS 2014)
  //! @param std_dev Standard deviation of a depth measure
  void calculateCovariance_jimenez(double std_dev = 0.02);
  //! @brief Composes the covariance of the plane according to Pathak et al (Intelligent Service Robotics 2010)
  //! @param std_dev Standard deviation of a depth measure
  void calculateCovariance_pathak(double std_dev = 0.02);
};

std::string DetectedPlane::toString(bool verbose) const
{
  std::ostringstream os;
  
  os << Plane::toString() << "\t";
  if (verbose) {
    os << "MSE = " << mse << "\t r_g = " << r_g.transpose();
    os << "\nCov = \n" << cov << "\n";
  }
  
  return os.str();
}

void printPlanes(const std::vector<DetectedPlane> &p) {
  std::cout << "Detected planes: " << p.size() << std::endl;
  for (unsigned int i = 0; i < p.size(); i++) {
    std::cout << "Plane " << i << ": " << p[i].toString(true) << std::endl;
  }
}

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV);
  double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
  return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

void DetectedPlane::calculateCovariance(double std_dev, bool pathak_way)
{
  if (pathak_way)
    calculateCovariance_pathak(std_dev);
  else
    calculateCovariance_jimenez(std_dev);
  weight = fabs(weight);
}


void DetectedPlane::calculateCovariance_jimenez(double std_dev)
{
  Eigen::Matrix4d hess;
  for (int i = 0; i < 3; i++) {
    hess(i, 3) = -s_g(i);
    hess(3, i) = -s_g(i);
    for (int j = 0; j < 3; j++) {
      hess(i, j) = p_k(i, j);
    }
  }
  hess (3, 3) = n_points;
//   std::cout << "N_points = " << n_points << std::endl;
  hess *= pow(std_dev, -2);
  
//   Eigen::Vector4d e;
//   e << v , d;
//   std::cout << hess << "Hess" << std::endl;
//   std::cout << "Hess * e = " << hess*e << std::endl;
  
  cov = pseudoInverse(hess, 1e-6);
  
  weight = 1/cov.determinant();
}

void DetectedPlane::calculateCovariance_pathak(double std_dev)
{
  Eigen::Matrix4d hess = Eigen::Matrix4d::Zero();
  double mu = pow(std_dev, -2.0) * n_points;
  
  Eigen::Matrix3d Hnn_div_mu;
  Hnn_div_mu = -r_g*r_g.transpose() - m_k + (  ( v.transpose() * m_k * v)(0,0)) * Eigen::Matrix3d::Identity();
  
  for (int i = 0; i < 3; i++) {
    hess(i, 3) = r_g(i);
    hess(3, i) = r_g(i);
    for (int j = 0; j < 3; j++) {
      hess(i, j) = Hnn_div_mu(i, j);
    }
  }
  hess (3, 3) = -1;
  hess *= mu;
  
//   std::cout << hess << "Hess" << std::endl;
//   Eigen::Vector4d e;
//   e << v , d;
//   std::cout << "Hess * e = " << hess*e << std::endl;
  
  cov = -pseudoInverse(hess);
  weight = 1/cov.determinant();
}

// NOTE: If the transform translates coordinates r2 into the transform 1 r1 (r1 = rot*r2 + trans) 
// then this function gives n2 and d2 (in coordinate transform 2) from a plane in transform 1
DetectedPlane DetectedPlane::affine_inv(const Eigen::Matrix3d& rot_, const Eigen::Vector3d& trans_) const
{
  DetectedPlane ret;
  
  ret.v = rot_.transpose() * v;
  ret.d = d - v.transpose()*trans_;
  
  ret.makeDPositive();
  
  return ret;
}

DetectedPlane DetectedPlane::affine_inv2(const Eigen::Matrix3d &rot, const Eigen::Vector3d& trans) const
{
  DetectedPlane pf;
  Eigen::Matrix4d U = Eigen::Matrix4d::Zero();
  
  Eigen::Vector4d e_t;
  
  
  for (int i = 0; i < 3; i++) {
    U(3, i) = -trans(i);
    e_t(i) = v(i);
    for (int j = 0; j < 3;j++) {
      U(i,j) = rot(j,i);
    }
  }
  U(3,3) = 1.0;
  e_t(3) = d;
  
  e_t = U*e_t;
  for (int i = 0; i < 3;i++) {
    pf.v(i) = e_t(i);
  }
  pf.d = e_t(3);
  pf.makeDPositive();
  
  return pf;
}


// NOTE: If the transform translates coordinates r2 into the transform 1 r1 (r1 = rot*r2 + trans) 
// then this function gives n2 and d2 (in coordinate transform 2) from a plane in transform 1
DetectedPlane DetectedPlane::affine(const Eigen::Matrix3d& rot_, const Eigen::Vector3d& trans_) const
{
  DetectedPlane ret;
  
  ret.v = rot_*v;
  ret.d = d + ret.v.transpose()*trans_;
  
  ret.makeDPositive();
  
  return ret;
}


#endif
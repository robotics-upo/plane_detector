#ifndef PLANE_HPP__
#define PLANE_HPP__

#include <eigen3/Eigen/Geometry>
#include <sstream>

// Describes the plane as all the points (p) that fulfills v*p = d
class Plane {
public:
  // Basic data
  Eigen::Vector3d v;
  double d;
  
  //! @brief Default constructor
  Plane() {
    v(0) = v(1) = v(2) = d = 0.0;
  }
  
  //! @brief Recommended constructor
  Plane(const Eigen::Vector3d v_, double d_):v(v_),d(d_) {
    
  }
  
  //! @brief Returns the distance from the point v_ to the plane
  //! @return The distance from v_ to the plane
  double distance(const Eigen::Vector3d v_) const;
  
  virtual std::string toString() const;
  
  void makeDPositive();
  
};

double Plane::distance(const Eigen::Vector3d v_) const
{
  return fabs(v_.dot(v) - d); 
}

std::string Plane::toString() const
{
  std::ostringstream os;
  
  os << "n = " << v.transpose() << "\t d = " << d;
  
  return os.str();
}

void Plane::makeDPositive()
{
  if (d < 0.0) {
    d *= -1.0;
    v *= -1.0;
    
  }
}




#endif
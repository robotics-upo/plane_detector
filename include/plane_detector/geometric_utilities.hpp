////////////////////////////////////////////////////////////////////////////////////
//  Geometric utilities for multicamera calibration
//
//  Copyright (C) 2016  David Alejo, Fernando Caballero, Luis Merino 
//  Universidad Pablo de Olavide
//  Seville, Spain.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
////////////////////////////////////////////////////////////////////////////////////

#ifndef GEOMETRIC_UTILS__
#define GEOMETRIC_UTILS__

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/concept_check.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifndef  CALI_VEC_SIZE 
#define CALI_VEC_SIZE 7
#endif

typedef boost::variate_generator<boost::mt19937, boost::normal_distribution<> > NormalDist;
static NormalDist norm_dist(boost::mt19937(time(0)), boost::normal_distribution<>(0, 1.0));


// --------------------------- General utilities ----------------------------------
double f_rand(double fMin = 0.0, double fMax = 1.0)
{
  static bool init = false;
  if (!init) {
    init = true;
    srand(time(NULL));
  }
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

std::vector<double> eigen2stdvec(const Eigen::Vector3d &v) {
  std::vector<double> ret;
  ret.resize(3);
  for (int i = 0; i < 3; i++) {
    ret[i] = v[i];
  }
  return ret;
}


// ----------------- Geometric utilities ------------------------------


//! Gets a transformation from a parameter
Eigen::Affine3d getTransform(const double *p) 
{
  // Get the transformation matrix T from p vector
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  Eigen::Quaterniond R(p[0], p[1], p[2], p[3]);
  T.rotate(R);
  T.pretranslate(Eigen::Vector3d(p[4], p[5], p[6]));
    
  return T;
}

void decomposeTransform(const Eigen::Affine3d &T, Eigen::Quaterniond &q, Eigen::Vector3d &v) {
  Eigen::Quaterniond d(T.rotation( ));
  q = d;
  v[0] = T(0,3);
  v[1] = T(1,3);
  v[2] = T(2,3);
}

bool quat2rotvec(const Eigen::Quaterniond &q, Eigen::Vector3d &v) {
  double norm = sqrt(1 - q.w());
  if (norm < 1e-4) { 
    // Too small rotation
    v[0] = v[1] = v[2] = 0.0;
    return false;
  }
  norm = 1 / norm;
  v[0] = q.x() * norm;
  v[1] = q.y() * norm;
  v[2] = q.z() * norm;
  
  return true;
}

Eigen::Quaterniond euler2quat(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d(1.0, 0.0, 0.0));
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d(0.0, 1.0, 0.0));
  Eigen::Quaterniond q =  yawAngle * pitchAngle * rollAngle; // 3-2-1
 
  return q;
}

// Returns the 3-2-1 body angles in [roll, pitch, yaw]
Eigen::Vector3d quat2euler(const Eigen::Quaterniond &q) {
  Eigen::Vector3d ret;
  double w = q.w(), x = q.x(), y = q.y(), z = q.z();
  
  ret[0] = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
  ret[1] = asin(0.9999999* (2*(w*y - z*x)));
//   cout << "x = " << asin(1.0) << endl;
  ret[2] = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
  
  return ret;
}

#endif
#include "plane_detector/detected_plane.hpp"
#include "plane_detector/ros_detected_plane.hpp"
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;
int main(int argc, char **argv) 
{
  if (argc < 3) {
    cerr << "Use: " << argv[0] << " <plane_file> <tf_file>\n";
    return -1;
  }
  
  DetectedPlane p;
  Eigen::Matrix3d rot;
  Eigen::Vector3d v;
  try {
    ifstream ifs2(argv[2]);
    // Get the guess TF from file
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        ifs2 >> rot(i,j);
      }
    }
    ifs2 >> v(0) >> v(1) >> v(2); 
    cout << "Rot:" << rot << endl;
    cout << "Trans: " << v.transpose() << endl;
    ifstream ifs(argv[1]);
    while (ifs.good() && !ifs.eof()) {
      ifs >> p.v(0) >> p.v(1) >> p.v(2) >> p.d; 
      cout << "Plane: " << p.toString() << endl;
      
      DetectedPlane rotated = p.affine(rot, v);
      cout << "Rotated plane: " << rotated.toString() << endl;
      cout << "Inverse Rotated plane: " << p.affine_inv(rot, v).toString() << endl;
      cout << "Inverse Rotated plane 2: " << p.affine_inv2(rot, v).toString() << endl;
      
      
      cout << "Rotated plane and inverse rotated (should be equal to the original: " << rotated.affine_inv(rot, v).toString() << endl;
      cout << "Rotated plane and inverse rotated (should be equal to the original: " << rotated.affine_inv2(rot, v).toString() << endl;

      DetectedPlaneROS r_p (p);
      cout << "DetectedPlaneROS" << r_p.toString() << endl;
      
    }
  } catch (exception &e) {
    
  }
  

  return 0;
}


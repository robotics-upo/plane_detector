#include "plane_detector/detected_plane.hpp"
#include <iostream>
#include <fstream>

using namespace std;
int main(int argc, char **argv) 
{
  if (argc < 3) {
    cerr << "Use: " << argv[0] << " <plane_file> <tf_file> [r_g file]\n";
    return -1;
  }
  
  DetectedPlane p;
  Eigen::Matrix3d rot;
  Eigen::Vector3d v;
  try {
    ifstream ifs(argv[1]);
    ifs >> p.v(0) >> p.v(1) >> p.v(2) >> p.d; 
    ifstream ifs2(argv[2]);
    // Get the guess TF from file
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        ifs2 >> rot(i,j);
      }
    }
    ifs2 >> v(0) >> v(1) >> v(2); 
  } catch (exception &e) {
    cerr << "Could not load parameter file";
  }
  cout << "Rot:" << rot << endl;
  cout << "Trans: " << v.transpose() << endl;
  
  
  if (argc > 3) {
    ifstream ifs(argv[3]);
    ifs >> p.r_g(0) >> p.r_g(1) >> p.r_g(2); 
    cout << "R_g = " << p.r_g.transpose() << endl;
  }
  
  
  cout << "Plane: " << p.toString() << endl;
  cout << "Rotated plane: " << p.affine(rot, v).toString() << endl;

  return 0;
}


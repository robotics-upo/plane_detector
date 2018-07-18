#include "plane_detector/plane_detector_ros.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "ros/ros.h"
#include <boost/tokenizer.hpp>

ros::Publisher marker_pub;

std::string link_name("/front_link");

bool initialized = false;
double scale = 1.0;

using namespace std;

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "test_plane_marker");
  
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");

  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("test_plane_marker", 1);
  
  std::string link("/base_link");
  pn.getParam("/link_name", link);
  
  DetectedPlaneROS p;
  if (argc > 4) {
    p.v(0) = stof(argv[1]);
    p.v(1) = stof(argv[2]);
    p.v(2) = stof(argv[3]);
    p.d = stof(argv[4]);
    if (argc > 7) {
      p.r_g(0) = stof(argv[5]);
      p.r_g(1) = stof(argv[6]);
      p.r_g(2) = stof(argv[7]);
    }
    
    cout << "Plane: " << p.toString() << endl;
    pub.publish(p.getMarker(link, 0, 1.0, 100.0));
  }
  
  bool end_ = false;
  char *c_s = new char[255];
  boost::char_separator<char> sep(" ");
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  while(!end_) {
    ros::spinOnce();
    cout << "Enter new plane (no data to exit) -->";
    cin.getline(c_s, 255);
    string s(c_s);
    tokenizer tok(s, sep);
    int cont = 0;
    for(tokenizer::iterator it = tok.begin(); it != tok.end(); ++it, cont++) 
    {
      istringstream is(*it);
      if (cont < 3) 
      {
        is >> p.v(cont);
      } else if (cont == 3) 
      {
        is >> p.d;
      } else if (cont < 7) 
      {
        is >> p.r_g(cont - 4);
      }
    }
    cout << "Cont = " << cont << endl;
    if (cont < 4) 
    {
      end_ = true;
    } else {
      pub.publish(p.getMarker(link, 0, 1.0, 100.0));
    }
  }
  
  return 0;
}



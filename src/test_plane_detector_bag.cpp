#include "plane_detector/plane_detector_ros.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif
#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include "ros/ros.h"
#include <limits>
#include <string>
#include <vector>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;

sensor_msgs::Image::Ptr decodeCompressedDepthImage(const sensor_msgs::CompressedImage& message);

int main(int argc, char **argv) 
{
  
  rosbag::Bag bag;
  std::string camera("/front/depth_registered");
  double delta = 10.0, epsilon = 3.0 , gamma = 10.0;
  int theta = 10000;
  
  int max_cont = 100;
  
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <bag file> [<camera_name>] [parameter_file] [max_images]\n";
    return -1;
  }
  if (argc > 2) {
    camera = string(argv[2]);
  }
  if (argc > 3) {
    try {
      ifstream ifs(argv[3]);
      ifs >> delta >> epsilon >> gamma >> theta; 
    } catch (exception &e) {
      cerr << "Could not load parameter file";
      
    }
  }
  
  if (argc > 4) {
    max_cont = stoi(argv[4]);
  }
  
  std::string compressed_topic = camera + "/image_raw/compressedDepth";
  std::string depth_topic = camera + "/image_raw";
  std::string info_topic = camera + "/camera_info";
  
  PlaneDetectorROS p(delta, epsilon, gamma, theta);
                                                   
  try {
    bag.open(string(argv[1]), rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(depth_topic));
    topics.push_back(std::string(compressed_topic));
    topics.push_back(std::string(info_topic));
    

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    bool initialized = false;

    int cont = 0;
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::CameraInfo::Ptr info = m.instantiate<sensor_msgs::CameraInfo>();
        if (info != NULL && !initialized) {
          p.setCameraParameters(info->K);
          initialized = true;
        }
        sensor_msgs::Image::Ptr im2 = m.instantiate<sensor_msgs::Image>();
        if (im2 != NULL && initialized) {
          cout << "Image " << cont << "\t";
          p.detectPlanes(*im2);
          printPlanes(p.getPlanes());
          cont++;
        }
      
        sensor_msgs::CompressedImage::Ptr im = m.instantiate<sensor_msgs::CompressedImage>();
        if (im != NULL && initialized) {
          cout << "Image = " << cont << "\t";
          sensor_msgs::Image::Ptr conv = decodeCompressedDepthImage(*im);
          p.detectPlanes(*conv);
          printPlanes(p.getPlanes());
          cont++;
        }
        
        if (cont > max_cont )
          break;
    }

    bag.close();
  } catch (exception &e) {
    cerr << "Exception while manipulating the bag  " << argv[1] << endl;
    cerr << "Content " << e.what() << endl;
    return -2;
  }

  
  return 0;
}

sensor_msgs::Image::Ptr decodeCompressedDepthImage(const sensor_msgs::CompressedImage& message)
{

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message.header;

  // Assign image encoding
  std::string image_encoding = message.format.substr(0, message.format.find(';'));
  cv_ptr->encoding = image_encoding;

  // Decode message data
  if (message.data.size() > sizeof(compressed_depth_image_transport::ConfigHeader))
  {

    // Read compression type from stream
    compressed_depth_image_transport::ConfigHeader compressionConfig;
    memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));

    // Get compressed image data
    const std::vector<uint8_t> imageData(message.data.begin() + sizeof(compressionConfig), message.data.end());

    // Depth map decoding
    float depthQuantA, depthQuantB;

    // Read quantization parameters
    depthQuantA = compressionConfig.depthParam[0];
    depthQuantB = compressionConfig.depthParam[1];

    if (enc::bitDepth(image_encoding) == 32)
    {
      cv::Mat decompressed;
      try
      {
        // Decode image data
        decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.what());
        return sensor_msgs::Image::Ptr();
      }

      size_t rows = decompressed.rows;
      size_t cols = decompressed.cols;

      if ((rows > 0) && (cols > 0))
      {
        cv_ptr->image = Mat(rows, cols, CV_32FC1);

        // Depth conversion
        MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
                            itDepthImg_end = cv_ptr->image.end<float>();
        MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                          itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
        {
          // check for NaN & max depth
          if (*itInvDepthImg)
          {
            *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
          }
          else
          {
            *itDepthImg = std::numeric_limits<float>::quiet_NaN();
          }
        }

        // Publish message to user callback
        return cv_ptr->toImageMsg();
      }
    }
    else
    {
      // Decode raw image
      try
      {
        
        cv_ptr->image = cv::imdecode(imageData, IMREAD_UNCHANGED); //CV_LOAD_IMAGE_UNCHANGED);
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR("%s", e.what());
        return sensor_msgs::Image::Ptr();
      }

      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;

      if ((rows > 0) && (cols > 0))
      {
        // Publish message to user callback
        return cv_ptr->toImageMsg();
      }
    }
  }
  return sensor_msgs::Image::Ptr();
}


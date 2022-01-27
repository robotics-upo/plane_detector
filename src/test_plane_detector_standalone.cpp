#include "plane_detector/plane_detector.hpp"
#include <limits>
#include <string>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

double scale = 1.0;

//! Util for loading depth image. Tested with depth images from TUM dataset https://vision.in.tum.de/data/datasets/rgbd-dataset
cv::Mat loadDepthImage(const char *filename);

int main(int argc, char **argv) 
{
    if (argc < 2) {
        printf("Usage: %s <depth image> [<delta> <epsilon> <gamma> <theta>]\n", argv[0]);
        return -1;
    }

    cv::Mat m = loadDepthImage(argv[1]);
    double delta = 0.2, epsilon = 0.001 , gamma = 0.1; // Default parameters
    int theta = 5000;
    // Get params from argv
    if (argc > 2) 
        delta = atof(argv[2]);
    if (argc > 3) 
        epsilon = atof(argv[3]);
    if (argc > 4) 
        gamma = atof(argv[4]);
    if (argc > 5) 
        theta = atoi(argv[5]);

    PlaneDetector p(delta, epsilon, gamma, theta);
    p.setCameraParameters(525.0f, 319.5f, 239.5f); // From TUM Dataset: focal_length = 525, cx = 319.5, cy = 239.5

    p.detectPlanes(m);
    printPlanes(p.getPlanes());

    return 0;
}

cv::Mat loadDepthImage(const char *filename) {
    cv::Mat img = cv::imread(filename, IMREAD_UNCHANGED);
    cout << "Image read. Type: " << img.type() << endl;
    cv::Mat ret, aux;
    const float scaleFactor = 1.0 / 5000.0;
    img.convertTo(ret, CV_32FC1, scaleFactor);

    // cv::Mat falseColorsMap;
    // applyColorMap(img, falseColorsMap, cv::COLORMAP_AUTUMN);
    ret.convertTo(aux, CV_8UC1, 50.0);
    cv::Mat falseColorsMap;
    applyColorMap(aux, falseColorsMap, cv::COLORMAP_AUTUMN);
    cv::imshow("Depth image", falseColorsMap);
    cv::waitKey();

    double min, max;
    cv::minMaxIdx(ret, &min, &max);
    cout << "Min: " << min << "\t Max: " << max << endl;


    return ret;
}

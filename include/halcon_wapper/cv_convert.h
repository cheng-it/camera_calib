#ifndef CV_CONVERT_H
#define CV_CONVERT_H

#include "HalconCpp.h"
#include "HDevThread.h"
#include <X11/Xlib.h>
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#define MATH_PI 3.1415926

using namespace HalconCpp;
using namespace std;
using namespace cv;

namespace halcon_wrapper
{
 
void hImageToCVImage(HObject &ho_Image, cv::Mat &cv_Image);

void cvImageToHImage(const cv::Mat &cv_Image, HObject &ho_Image);

} //namespace halcon_wapper

#endif
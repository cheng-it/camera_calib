#include "HalconCpp.h"
#include "HDevThread.h"
#include <X11/Xlib.h>
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include "halcon_wapper/cv_convert.h"
#include "halcon_wapper/calibrate.h"

using namespace HalconCpp;
using namespace std;
using namespace cv;
using namespace halcon_wapper;

void printCorners(const cv::vector<cv::Point2f> corners)
{
  cout << "print all corners." << endl;
  for (int i = 0; i < corners.size(); i++)
	cout << corners[i] << endl;
}

int main(int argc, char *argv[])
{
  using namespace halcon_wapper;
  
  const string imageFile = "/home/jackymond/Data/image/fan2_lasor_visual/left_camera/img30.jpg";
  
  const string plateDescriptionFile = "/home/jackymond/Software/halcon/calib/caltab_30mm.descr";
  HTuple hv_PlateDescription;
  hv_PlateDescription = plateDescriptionFile.c_str();
  
  HTuple hv_CamParam;
  hv_CamParam.Clear();
  hv_CamParam[0] = "area_scan_division";
  hv_CamParam[1] = 0.00414847;
  hv_CamParam[2] = -33485.5;
  hv_CamParam[3] = 8.31651e-06;
  hv_CamParam[4] = 8.3e-06;
  hv_CamParam[5] = 357.822;
  hv_CamParam[6] = 242.925;
  hv_CamParam[7] = 752;
  hv_CamParam[8] = 480;
  
  cv:Mat frame = imread(imageFile, CV_LOAD_IMAGE_GRAYSCALE);
  if (frame.empty())
  {
	printf("the image is empty.");
	return -1;
  }
  std::vector<cv::Point2f> centers;
  cv::Mat R, t;
  
  Mat frameUndistorted;
  cv_UndistortImage(frame, frameUndistorted, hv_CamParam);
  imshow("Undistort", frameUndistorted);
  char key1 = (char)waitKey();
  destroyWindow("Undistort");
  
  cv_FindMarisAndPose(frame, hv_CamParam, hv_PlateDescription, centers, R, t);
  cout << "R: " << R << endl;
  cout << "t: " << t << endl;
  printCorners(centers);
  
  Size patternSize(7, 7);
  drawChessboardCorners(frame, patternSize, Mat(centers), true);
  imshow("FindCircleGrid", frame);
  char key = (char)waitKey();
  destroyWindow("FindCircleGrid");
  return 0;
}


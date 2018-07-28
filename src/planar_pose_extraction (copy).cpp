///////////////////////////////////////////////////////////////////////////////
// File generated by HDevelop for HALCON/C++ Version 13.0.1
///////////////////////////////////////////////////////////////////////////////
#  include "HalconCpp.h"
#  include "HDevThread.h"
#  if defined(__linux__) && !defined(__arm__) && !defined(NO_EXPORT_APP_MAIN)
#    include <X11/Xlib.h>
#  endif
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#define MATH_PI 3.1415926

using namespace HalconCpp;
using namespace std;
using namespace cv;


cv::Mat hImageToCVImage(HObject &hObj)
{
	HTuple hCh = HTuple();
	ConvertImageType(hObj, &hObj, "byte");
	CountChannels(hObj, &hCh);
	HTuple hW;
	HTuple hH;
	HTuple hType;
	int width, height;
	
	Mat cvMat;
	if(hCh[0].I()==1)
	{
		HTuple ptr;
		GetImagePointer1(hObj, &ptr, &hType, &hW, &hH);
		width = (Hlong)hW;
		height = (Hlong)hH;
		cvMat.create(height, width, CV_8UC1);
		memcpy(cvMat.data, (uchar*)ptr[0].I(), width*height*sizeof(uchar));
		
	}
	if(hCh[0].I()==3)
	{
		width = (Hlong)width;
		height = (Hlong)height;
		
		HTuple ptrR, ptrG, ptrB;
		GetImagePointer3(hObj, &ptrR, &ptrG, &ptrB, &hType, &hW, &hH);
		width = (Hlong)hW;
		height = (Hlong)hH;
		uchar* pR = (uchar*)ptrR[0].I();
		uchar* pG = (uchar*)ptrG[0].I();
		uchar* pB = (uchar*)ptrB[0].I();
		
		Mat rgb[3];
		for (int i; i<3; i++) rgb[i].create(height, width, CV_8UC1);
		memcpy(rgb[2].data, pR, width*height*sizeof(uchar));
		memcpy(rgb[1].data, pG, width*height*sizeof(uchar));
		memcpy(rgb[0].data, pB, width*height*sizeof(uchar));
		merge(rgb, 3, cvMat);
	}
	return cvMat;
}

void cvImageToHImage(const cv::Mat &cvImage, HObject &hvImage)
{
	if (cvImage.empty())
	  return;
	int height =cvImage.rows;
	int width =cvImage.cols;
	
	if (cvImage.channels() == 3)
	{
	  Mat chs[3];
	  split(cvImage, chs);
	  GenImage3(&hvImage, "byte", width, height, (Hlong)chs[0].data, (Hlong)chs[1].data, (Hlong)chs[2].data);
	}
	else if (cvImage.channels() == 1)
	  GenImage1(&hvImage, "byte", width, height, (Hlong)cvImage.data);
	else
	  return;
}

// Main procedure 
void cvfindMarisAndPose(cv::Mat &frame,
  std::vector<cv::Point2f> &corners, cv::Mat &R, cv::Mat &t,
  const string &plateDescription)
{
  // Local iconic variables
  HObject  ho_Image, ho_CalPlate;

  // Local control variables
  HTuple  hv_ImageFile, hv_PlateDescription, hv_StartCamParam;
  HTuple  hv_RCoord, hv_CCoord, hv_StartPose;

  hv_ImageFile = "/home/jackymond/Data/image/fan2_lasor_visual/left_camera/img30.jpg";
  hv_PlateDescription = "/home/jackymond/Software/halcon/calib/caltab_30mm.descr";
  hv_PlateDescription = plateDescription.c_str();
  
  cvImageToHImage(frame, ho_Image);
  //ReadImage(&ho_Image, hv_ImageFile);

  //Find calibration pattern.
  FindCaltab(ho_Image, &ho_CalPlate, hv_PlateDescription, 3, 112, 5);
  //Find calibration marks and start pose.
  hv_StartCamParam.Clear();
  hv_StartCamParam[0] = "area_scan_division";
  hv_StartCamParam[1] = 0.00414847;
  hv_StartCamParam[2] = -33485.5;
  hv_StartCamParam[3] = 8.31651e-06;
  hv_StartCamParam[4] = 8.3e-06;
  hv_StartCamParam[5] = 357.822;
  hv_StartCamParam[6] = 242.925;
  hv_StartCamParam[7] = 752;
  hv_StartCamParam[8] = 480;

  FindMarksAndPose(ho_Image, ho_CalPlate, hv_PlateDescription, hv_StartCamParam, 
      128, 10, 18, 0.9, 15.0, 100.0, &hv_RCoord, &hv_CCoord, &hv_StartPose);

  corners.clear();
  for (int i=0; i<49; i++)
    corners.push_back(cv::Point2f(hv_CCoord[i], hv_RCoord[i]));

  Mat r = (Mat_<double>(3, 1) << hv_StartPose[3]/180, hv_StartPose[4]/180, hv_StartPose[5]/180);
  cv::Rodrigues(r, R);
  t = (Mat_<double>(3, 1) << hv_StartPose[0], hv_StartPose[1], hv_StartPose[2]);

  // 校正畸变
//   HTuple hv_CarParamVirtualFixed = hv_StartCamParam;
//   HObject ho_Map, ho_ImageMapped;
//   ChangeRadialDistortionCamPar("adaptive", hv_StartCamParam, 0, &hv_CarParamVirtualFixed);
//   GenRadialDistortionMap(&ho_Map, hv_StartCamParam, hv_CarParamVirtualFixed, "bilinear");
//   MapImage(ho_Image, ho_Map, &ho_ImageMapped);
//   frame = hImageToCVImage(ho_ImageMapped);
}

void undistortImage()
{
  
}

void printCorners(const cv::vector<cv::Point2f> corners)
{
  cout << "print all corners." << endl;
  for (int i = 0; i < corners.size(); i++)
	cout << corners[i] << endl;
}

int main(int argc, char *argv[])
{
  #if defined(_WIN32)
	SetSystem("use_window_thread", "true");
  #elif defined(__linux__)
	XInitThreads();
  #endif

  // Default settings used in HDevelop (can be omitted) 
  int ret=0;
  SetSystem("width", 512);
  SetSystem("height", 512);
  
  string imageFile = "/home/jackymond/Data/image/fan2_lasor_visual/left_camera/img29.jpg";
  
  cv:Mat frame = imread(imageFile, CV_LOAD_IMAGE_GRAYSCALE);
  if (frame.empty())
  {
	printf("the image is empty.");
	return -1;
  }
  std::vector<cv::Point2f> centers;
  cv::Mat R, t;
  string plateDescriptionFile = "/home/jackymond/Software/halcon/calib/caltab_100mm.descr";
  cvfindMarisAndPose(frame, centers, R, t, plateDescriptionFile);
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



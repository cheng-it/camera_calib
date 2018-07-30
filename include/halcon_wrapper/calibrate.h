#ifndef CALIBRATE_H
#define CALIBRATE_H

#include "HalconCpp.h"
#include "HDevThread.h"
#include <X11/Xlib.h>
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace HalconCpp;
using namespace std;
using namespace cv;

namespace halcon_wrapper
{
  //////////////////////////////
  // monocular calibrate
  // undistort
  //////////////////////////////
  void hc_calibrateMonoCular(const string &imageFolder, const string &plateDescriptionFile, const string &camParamSaveDir);
  void cv_calibrateMonoCular(const string &imageFolder,
		const string &plateDescriptionFile, const string &camParamSaveDir,
		std::vector<double> startCamParam,
		cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
  
  void cvtCamParamToCVMat(const HTuple &hv_CamParam, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
  void hc_saveCamParamToCVYAML(const HTuple &hv_CamParam, std::string output);
  void hc_readCamParam(HTuple &hv_CamParam, const string &camParamFile);

  void hc_UndistortImage(HObject &ho_Image, HObject &ho_ImageMapped, const HTuple &hv_CamParam);
  void cv_UndistortImage(const cv::Mat &cv_Image, cv::Mat &cv_ImageMapped, const HTuple &hv_CamParam);

  //////////////////////////////
  // planar pose extraction
  //////////////////////////////
  void hc_FindMarksAndPose(HObject &ho_Image, 
		const HTuple &hv_CamParam, const HTuple &hv_PlateDescription,
		HTuple &hv_RCoord, HTuple &hv_CCoord, HTuple &hv_Pose);
  void cv_FindMarksAndPose(const std::string &imageFile,
		const HTuple &hv_CamParam, const HTuple &hv_PlateDescription,
		std::vector<cv::Point2f> &centers, cv::Mat &R, cv::Mat &t);
  void cv_FindMarksAndPose(const cv::Mat &cv_Image,
		const HTuple &hv_CamParam, const HTuple &hv_PlateDescription,
		std::vector<cv::Point2f> &centers, cv::Mat &R, cv::Mat &t);
} //namespace halcon_wapper

#endif
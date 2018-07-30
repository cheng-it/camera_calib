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
using namespace halcon_wrapper;

int runCalibrate()
{
  const string imageFolder = "/home/jackymond/Data/image/fan2_lasor_visual/left_camera";
  const string plateDescriptionFile = "/home/jackymond/Software/halcon/calib/caltab_30mm.descr";
  const string camParamSaveDir = "/home/jackymond";
  hc_calibrateMonoCular(imageFolder, plateDescriptionFile, camParamSaveDir);
}

int runUndistort()
{
  const string imageFile = "/home/jackymond/Data/image/fan2_lasor_visual/left_camera/img30.jpg";
  const string camParamFile = "/home/jackymond/camera_parameters.dat";
  HTuple hv_CamParam;
  hc_readCamParam(hv_CamParam, camParamFile);
  
  cv:Mat frame, frameUndistorted;
  frame = imread(imageFile, CV_LOAD_IMAGE_GRAYSCALE);
  if (frame.empty())
  {
	printf("the image is empty.");
	return -1;
  }
  cv_UndistortImage(frame, frameUndistorted, hv_CamParam);
  imshow("Undistort", frameUndistorted);
  char key1 = (char)waitKey();
  destroyWindow("Undistort");  
}

int main(int argc, char *argv[])
{
  runCalibrate();
  //runUndistort();
  return 0;
}



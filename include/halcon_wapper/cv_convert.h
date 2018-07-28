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

namespace halcon_wapper
{
 
void hImageToCVImage(HObject &hObj, cv::Mat &cvMat)
{
	HTuple hv_Ch = HTuple();
	ConvertImageType(hObj, &hObj, "byte");
	CountChannels(hObj, &hv_Ch);
	HTuple hv_W;
	HTuple hv_H;
	HTuple hType;
	int width, height;
	
	if(hv_Ch[0].I()==1)
	{
		HTuple ptr;
		GetImagePointer1(hObj, &ptr, &hType, &hv_W, &hv_H);
		width = (Hlong)hv_W;
		height = (Hlong)hv_H;
		cvMat.create(height, width, CV_8UC1);
		memcpy(cvMat.data, reinterpret_cast<uchar *>(ptr[0].I()), width*height*sizeof(uchar));
		
	}
	if(hv_Ch[0].I()==3)
	{
		width = (Hlong)width;
		height = (Hlong)height;
		
		HTuple ptrR, ptrG, ptrB;
		GetImagePointer3(hObj, &ptrR, &ptrG, &ptrB, &hType, &hv_W, &hv_H);
		width = (Hlong)hv_W;
		height = (Hlong)hv_H;
		uchar* pR = reinterpret_cast<uchar *>(ptrR[0].I());
		uchar* pG = reinterpret_cast<uchar *>(ptrG[0].I());
		uchar* pB = reinterpret_cast<uchar *>(ptrB[0].I());
		
		Mat rgb[3];
		for (int i; i<3; i++) rgb[i].create(height, width, CV_8UC1);
		memcpy(rgb[2].data, pR, width*height*sizeof(uchar));
		memcpy(rgb[1].data, pG, width*height*sizeof(uchar));
		memcpy(rgb[0].data, pB, width*height*sizeof(uchar));
		merge(rgb, 3, cvMat);
	}
}

void cvImageToHImage(const cv::Mat &cv_Image, HObject &ho_Image)
{
	if (cv_Image.empty())
	  return;
	int height = cv_Image.rows;
	int width = cv_Image.cols;
	
	if (cv_Image.channels() == 3)
	{
	  Mat chs[3];
	  split(cv_Image, chs);
	  GenImage3(&ho_Image, "byte", width, height, (Hlong)chs[0].data, (Hlong)chs[1].data, (Hlong)chs[2].data);
	}
	else if (cv_Image.channels() == 1)
	  GenImage1(&ho_Image, "byte", width, height, (Hlong)cv_Image.data);
	else
	  return;
}

} //namespace halcon_wapper

#endif
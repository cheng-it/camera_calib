#include "halcon_wapper/cv_convert.h"
#include "halcon_wapper/calibrate.h"

namespace halcon_wrapper
{
	//////////////////////////////
	// monocular calibrate
	// undistort
	//////////////////////////////
	void hc_calibrateMonoCular(const string &imageFolder, const string &plateDescriptionFile, const string &camParamSaveDir)
	{
		// 变量声明
		// Local iconic variables
		HObject  ho_Image, ho_GrayImage, ho_Map, ho_ImageMapped;
		// Local control variables
		HTuple  hv_ImageFiles, hv_TmpCtrl_ReferenceIndex;
		HTuple  hv_TmpCtrl_PlateDescription;
		HTuple  hv_TmpCtrl_FindCalObjParNames, hv_TmpCtrl_FindCalObjParValues;
		HTuple  hv_CalibHandle, hv_Index, hv_TmpCtrl_Errors;
		HTuple  hv_StartParameters, hv_CameraParameters, hv_CameraPose, hv_CarParamVirtualFixed;

		/* 相机标定 */
		ListFiles(imageFolder.c_str(), (HTuple("files").Append("follow_links")), 
			&hv_ImageFiles);
		TupleRegexpSelect(hv_ImageFiles, (HTuple("\\.(jpg|jpeg|png)$").Append("ignore_case")), 
			&hv_ImageFiles);
		
		hv_TmpCtrl_ReferenceIndex = 0;
		hv_TmpCtrl_PlateDescription = plateDescriptionFile.c_str();
	//     hv_StartParameters.Clear();
	//     hv_StartParameters[0] = "area_scan_division";
	//     hv_StartParameters[1] = 0.008;
	//     hv_StartParameters[2] = 0;
	//     hv_StartParameters[3] = 8.3e-06;
	//     hv_StartParameters[4] = 8.3e-06;
	//     hv_StartParameters[5] = 376;
	//     hv_StartParameters[6] = 240;
	//     hv_StartParameters[7] = 752;
	//     hv_StartParameters[8] = 480;
		hv_StartParameters.Clear();
		hv_StartParameters[0] = "area_scan_polynomial";
		hv_StartParameters[1] = 0.008;
		hv_StartParameters[2] = 0;
		hv_StartParameters[3] = 0;
		hv_StartParameters[4] = 0;
		hv_StartParameters[5] = 0;
		hv_StartParameters[6] = 0;
		hv_StartParameters[7] = 8.3e-06;
		hv_StartParameters[8] = 8.3e-06;
		hv_StartParameters[9] = 376;
		hv_StartParameters[10] = 240;
		hv_StartParameters[11] = 752;
		hv_StartParameters[12] = 480;
		hv_TmpCtrl_FindCalObjParNames.Clear();
		hv_TmpCtrl_FindCalObjParNames[0] = "gap_tolerance";
		hv_TmpCtrl_FindCalObjParNames[1] = "alpha";
		hv_TmpCtrl_FindCalObjParNames[2] = "skip_find_caltab";
		hv_TmpCtrl_FindCalObjParValues.Clear();
		hv_TmpCtrl_FindCalObjParValues[0] = 1;
		hv_TmpCtrl_FindCalObjParValues[1] = 1;
		hv_TmpCtrl_FindCalObjParValues[2] = "false";
		// Create calibration model for managing calibration data
		CreateCalibData("calibration_object", 1, 1, &hv_CalibHandle);
		SetCalibDataCamParam(hv_CalibHandle, 0, HTuple(), hv_StartParameters);
		SetCalibDataCalibObject(hv_CalibHandle, 0, hv_TmpCtrl_PlateDescription);
		// Collect mark positions and estimated poses for all plates
		{
			HTuple end_val14 = (hv_ImageFiles.TupleLength())-1;
			HTuple step_val14 = 1;
			for (hv_Index=0; hv_Index.Continue(end_val14, step_val14); hv_Index += step_val14)
			{
				ReadImage(&ho_Image, HTuple(hv_ImageFiles[hv_Index]));
				Rgb1ToGray(ho_Image, &ho_GrayImage);
				FindCalibObject(ho_GrayImage, hv_CalibHandle, 0, 0, hv_Index, hv_TmpCtrl_FindCalObjParNames, 
					hv_TmpCtrl_FindCalObjParValues);
			}
		}
		// Perform the actual calibration
		CalibrateCameras(hv_CalibHandle, &hv_TmpCtrl_Errors);
		GetCalibData(hv_CalibHandle, "camera", 0, "params", &hv_CameraParameters);
		GetCalibData(hv_CalibHandle, "calib_obj_pose", HTuple(0).TupleConcat(hv_TmpCtrl_ReferenceIndex), 
			"pose", &hv_CameraPose);
		// Adjust origin for plate thickness
		SetOriginPose(hv_CameraPose, 0.0, 0.0, 0.001, &hv_CameraPose);
		// Clear calibration model when done
		ClearCalibData(hv_CalibHandle);
		

		/* 保存参数 */
		try
		{
		WriteCamPar(hv_CameraParameters, (camParamSaveDir + "/camera_parameters.dat").c_str());
		//WritePose(hv_CameraPose, "/home/jackymond/Workspace2/halcon/campos.dat");
		}
		catch(Exception e)
		{
		cout << e.what() << endl;
		}
		
		hc_saveCamParamToCVYAML(hv_CameraParameters, "camera_parameters.yaml");
	}

	void cv_calibrateMonoCular(const string &imageFolder,
			const string &plateDescriptionFile, const string &camParamSaveDir,
			std::vector<double> startCamParam, 
			cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
	{	
		// 变量声明
		// Local iconic variables
		HObject  ho_Image, ho_GrayImage, ho_Map, ho_ImageMapped;
		// Local control variables
		HTuple  hv_ImageFiles, hv_TmpCtrl_ReferenceIndex;
		HTuple  hv_TmpCtrl_PlateDescription;
		HTuple  hv_TmpCtrl_FindCalObjParNames, hv_TmpCtrl_FindCalObjParValues;
		HTuple  hv_CalibHandle, hv_Index, hv_TmpCtrl_Errors;
		HTuple  hv_StartParameters, hv_CameraParameters, hv_CameraPose;

		/* 相机标定 */
		ListFiles(imageFolder.c_str(), (HTuple("files").Append("follow_links")), 
			&hv_ImageFiles);
		TupleRegexpSelect(hv_ImageFiles, (HTuple("\\.(jpg|jpeg|png)$").Append("ignore_case")), 
			&hv_ImageFiles);
		
		hv_TmpCtrl_ReferenceIndex = 0;
		hv_TmpCtrl_PlateDescription = plateDescriptionFile.c_str();
		hv_StartParameters.Clear();
		if (startCamParam.size() == 12)
		{
			hv_StartParameters[0] = "area_scan_polynomial";
			for (int i=0; i<startCamParam.size(); i++)
				hv_StartParameters[i+1] = startCamParam[i];
		}
		else if (startCamParam.size() == 8)
		{
			hv_StartParameters[0] = "area_scan_division";
			for (int i=0; i<startCamParam.size(); i++)
				hv_StartParameters[i+1] = startCamParam[i];
		}
		else
			return;
		
		hv_TmpCtrl_FindCalObjParNames.Clear();
		hv_TmpCtrl_FindCalObjParNames[0] = "gap_tolerance";
		hv_TmpCtrl_FindCalObjParNames[1] = "alpha";
		hv_TmpCtrl_FindCalObjParNames[2] = "skip_find_caltab";
		hv_TmpCtrl_FindCalObjParValues.Clear();
		hv_TmpCtrl_FindCalObjParValues[0] = 1;
		hv_TmpCtrl_FindCalObjParValues[1] = 1;
		hv_TmpCtrl_FindCalObjParValues[2] = "false";
		// Create calibration model for managing calibration data
		CreateCalibData("calibration_object", 1, 1, &hv_CalibHandle);
		SetCalibDataCamParam(hv_CalibHandle, 0, HTuple(), hv_StartParameters);
		SetCalibDataCalibObject(hv_CalibHandle, 0, hv_TmpCtrl_PlateDescription);
		// Collect mark positions and estimated poses for all plates
		{
		HTuple end_val14 = (hv_ImageFiles.TupleLength())-1;
		HTuple step_val14 = 1;
		for (hv_Index=0; hv_Index.Continue(end_val14, step_val14); hv_Index += step_val14)
		{
			ReadImage(&ho_Image, HTuple(hv_ImageFiles[hv_Index]));
			Rgb1ToGray(ho_Image, &ho_GrayImage);
			FindCalibObject(ho_GrayImage, hv_CalibHandle, 0, 0, hv_Index, hv_TmpCtrl_FindCalObjParNames, 
				hv_TmpCtrl_FindCalObjParValues);
		}
		}
		// Perform the actual calibration
		CalibrateCameras(hv_CalibHandle, &hv_TmpCtrl_Errors);
		GetCalibData(hv_CalibHandle, "camera", 0, "params", &hv_CameraParameters);
		GetCalibData(hv_CalibHandle, "calib_obj_pose", HTuple(0).TupleConcat(hv_TmpCtrl_ReferenceIndex), 
			"pose", &hv_CameraPose);
		// Adjust origin for plate thickness
		SetOriginPose(hv_CameraPose, 0.0, 0.0, 0.001, &hv_CameraPose);
		// Clear calibration model when done
		ClearCalibData(hv_CalibHandle);
		
		cvtCamParamToCVMat(hv_CameraParameters, cameraMatrix, distCoeffs);
	}
	
	void cvtCamParamToCVMat(const HTuple &hv_CamParam, cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
	{
		cameraMatrix = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
		cameraMatrix.at<double>(0, 0) = hv_CamParam[1].D() * 1e5;
		cameraMatrix.at<double>(1, 1) = hv_CamParam[1].D() * 1e5;
		cameraMatrix.at<double>(0, 2) = hv_CamParam[9].D();
		cameraMatrix.at<double>(1, 2) = hv_CamParam[10].D();
		cameraMatrix.at<double>(2, 2) = 1;
		distCoeffs = (Mat_<double>(5, 1) << hv_CamParam[2].D(), hv_CamParam[3].D(), hv_CamParam[4].D(), hv_CamParam[5].D(), hv_CamParam[6].D());
	}

	void hc_readCamParam(HTuple &hv_CamParam, const string &camParamFile)
	{
		HTuple hv_CamParamFile = camParamFile.c_str();
		/* 读取参数 */
		try
		{
			ReadCamPar(hv_CamParamFile, &hv_CamParam);
		}
		catch(Exception e)
		{
			cout << e.what() << endl;
		}  
	}

	void hc_saveCamParamToCVYAML(const HTuple &hv_CamParam, std::string output)
	{
		if (std::string(hv_CamParam[0].S().Text()) == "area_scan_polynomial" && hv_CamParam.Length() == 13)
		{
			Mat M = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
			M.at<double>(0, 0) = hv_CamParam[1].D() * 1e5;
			M.at<double>(1, 1) = hv_CamParam[1].D() * 1e5;
			M.at<double>(0, 2) = hv_CamParam[9].D();
			M.at<double>(1, 2) = hv_CamParam[10].D();
			M.at<double>(2, 2) = 1;
			Mat D = (Mat_<double>(5, 1) << hv_CamParam[2].D(), hv_CamParam[3].D(), hv_CamParam[4].D(), hv_CamParam[5].D(), hv_CamParam[6].D());
			// Write into intrinsic.yaml
			cv::FileStorage fs;
			fs = cv::FileStorage(output.c_str(), cv::FileStorage::WRITE);
			if (fs.isOpened())
			{
				fs << "M" << M;
				fs << "D" << D;
				fs.release();
				cout << "save camera parameters as" << output << endl;;
			}
		}
		else
		cout << "camera model is not area_scan_polynomial!\n";
	}
	
	void hc_UndistortImage(HObject &ho_Image, HObject &ho_ImageMapped, const HTuple &hv_CamParam)
	{
		//校正畸变
		HTuple hv_CarParamVirtualFixed;
		HObject ho_Map;
		ChangeRadialDistortionCamPar("adaptive", hv_CamParam, 0, &hv_CarParamVirtualFixed);
		GenRadialDistortionMap(&ho_Map, hv_CamParam, hv_CarParamVirtualFixed, "bilinear");
		MapImage(ho_Image, ho_Map, &ho_ImageMapped);
		}

		void cv_UndistortImage(const cv::Mat &cv_Image, cv::Mat &cv_ImageMapped, const HTuple &hv_CamParam)
		{
		HObject ho_Image;
		cvImageToHImage(cv_Image, ho_Image);

		//校正畸变
		HTuple hv_CarParamVirtualFixed;
		HObject ho_Map, ho_ImageMapped;
		ChangeRadialDistortionCamPar("adaptive", hv_CamParam, ((((HTuple(0).Append(0)).Append(0)).Append(0)).Append(0)), &hv_CarParamVirtualFixed);
		GenRadialDistortionMap(&ho_Map, hv_CamParam, hv_CarParamVirtualFixed, "bilinear");
		MapImage(ho_Image, ho_Map, &ho_ImageMapped);

		hImageToCVImage(ho_ImageMapped, cv_ImageMapped);
	}


	//////////////////////////////
	// planar pose extraction
	//////////////////////////////
	void hc_FindMarksAndPose(HObject &ho_Image,
						const HTuple &hv_CamParam, const HTuple &hv_PlateDescription,
						HTuple &hv_RCoord, HTuple &hv_CCoord, HTuple &hv_Pose)
	{
		HObject ho_CalPlate;
		//Find calibration pattern.
		FindCaltab(ho_Image, &ho_CalPlate, hv_PlateDescription, 3, 112, 5);

		//Find calibration marks and start pose.
		FindMarksAndPose(ho_Image, ho_CalPlate, hv_PlateDescription, hv_CamParam, 
			128, 10, 18, 0.9, 15.0, 100.0, &hv_RCoord, &hv_CCoord, &hv_Pose);
		}

		void cv_FindMarksAndPose(const std::string &imageFile,
		const HTuple &hv_CamParam, const HTuple &hv_PlateDescription,
		std::vector<cv::Point2f> &centers, cv::Mat &R, cv::Mat &t)
		{
		HObject ho_Image, ho_CalPlate;
		HTuple hv_ImageFile;
		HTuple hv_RCoord, hv_CCoord, hv_Pose;

		// Read image
		hv_ImageFile = imageFile.c_str();
		ReadImage(&ho_Image, hv_ImageFile);

		//Find calibration pattern.
		FindCaltab(ho_Image, &ho_CalPlate, hv_PlateDescription, 3, 112, 5);

		//Find calibration marks and start pose.
		FindMarksAndPose(ho_Image, ho_CalPlate, hv_PlateDescription, hv_CamParam, 
			128, 10, 18, 0.9, 15.0, 100.0, &hv_RCoord, &hv_CCoord, &hv_Pose);

		centers.clear();
		for (int i=0; i<49; i++)
			centers.push_back(cv::Point2f(hv_CCoord[i], hv_RCoord[i]));

		Mat r = (Mat_<double>(3, 1) << hv_Pose[3]/180, hv_Pose[4]/180, hv_Pose[5]/180);
		cv::Rodrigues(r, R);
		t = (Mat_<double>(3, 1) << hv_Pose[0], hv_Pose[1], hv_Pose[2]);
	}


	void cv_FindMarksAndPose(const cv::Mat &cv_Image,
		const HTuple &hv_CamParam, const HTuple &hv_PlateDescription,
		std::vector<cv::Point2f> &centers, cv::Mat &R, cv::Mat &t)
	{
		HObject ho_Image;
		HTuple hv_RCoord, hv_CCoord, hv_Pose;
		cvImageToHImage(cv_Image, ho_Image);

		hc_FindMarksAndPose(ho_Image, hv_CamParam, hv_PlateDescription, hv_RCoord, hv_CCoord, hv_Pose);

		centers.clear();
		for (int i=0; i<49; i++)
			centers.push_back(cv::Point2f(hv_CCoord[i], hv_RCoord[i]));

		Mat r = (Mat_<double>(3, 1) << hv_Pose[3]/180, hv_Pose[4]/180, hv_Pose[5]/180);
		cv::Rodrigues(r, R);
		t = (Mat_<double>(3, 1) << hv_Pose[0], hv_Pose[1], hv_Pose[2]);
	}
} //namespace halcon_wapper

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>
#include<LSM/src/LSM.h>
#include"Line3d.h"
#include"boost/filesystem.hpp"
#include"boost/program_options.hpp"
#include"boost/program_options/errors.hpp"
#include"LineMapping.h"

using namespace std;
using namespace boost::program_options;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
	vector<double> &vTimestamps);

int ComputePose(ORB_SLAM2::System &SLAM, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, string &imgPath, int iteration);

void Undistort(string &strSettingPath, vector<string> &vstrImageFilenames, string &imgDir) {

	// If already undistorted, skip. 
	string undistortPath = imgDir + "/undistort";
	boost::filesystem::path dir(undistortPath);
	if (boost::filesystem::create_directory(dir))
	{
		boost::filesystem::path dir_img(undistortPath + "/images");
		boost::filesystem::create_directory(dir_img);
		std::cerr << "Directory Created: " << undistortPath << std::endl;
	}
	else {
		cout << "Already Undistortion done." << endl;
		return;
	}

	// Load camera parameters from settings file
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
	float fx = fSettings["Camera.fx"];
	float fy = fSettings["Camera.fy"];
	float cx = fSettings["Camera.cx"];
	float cy = fSettings["Camera.cy"];

	cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
	K.at<float>(0, 0) = fx;
	K.at<float>(1, 1) = fy;
	K.at<float>(0, 2) = cx;
	K.at<float>(1, 2) = cy;

	cv::Mat DistCoef(4, 1, CV_32F);
	DistCoef.at<float>(0) = fSettings["Camera.k1"];
	DistCoef.at<float>(1) = fSettings["Camera.k2"];
	DistCoef.at<float>(2) = fSettings["Camera.p1"];
	DistCoef.at<float>(3) = fSettings["Camera.p2"];
	const float k3 = fSettings["Camera.k3"];
	if (k3 != 0)
	{
		DistCoef.resize(5);
		DistCoef.at<float>(4) = k3;
	}

	int nImages = vstrImageFilenames.size();

	for (int i = 0; i < nImages; i++) {
		// Read image from file
		Mat im = cv::imread(imgDir + "/" + vstrImageFilenames[i], CV_LOAD_IMAGE_UNCHANGED);
		Mat undistort = im.clone();
		cv::undistort(im, undistort, K, DistCoef);

		string outputName = undistortPath + "/" + vstrImageFilenames[i];
		cv::imwrite(outputName, undistort);
	}

	// copy Keyframe infos. to newly created directory. 
	string keyframeInfo = imgDir + "KFinfo.txt";
	string copiedPath = undistortPath + "KFinfo.txt";
	boost::filesystem::copy_file(keyframeInfo, copiedPath, boost::filesystem::copy_option::overwrite_if_exists);

	cout << "Undistortion done." << endl;
}

Mat ConvertQuatFromMat(const float Qw, const float Qx, const float Qy, const float Qz);

int PoseFromColmap(ORB_SLAM2::System &SLAM, vector<string> &vstrImageFilenames, string &sSfMDir) {

	//int nImages = vstrImageFilenames.size();
	Map *_pMap = SLAM.GetMap();

	cout << endl << "-------" << endl;
	cout << "Loading camera poses from SfM (Colmap) ..." << endl;

	// check if colmap directory exists
	boost::filesystem::path sfm(sSfMDir);
	if (!boost::filesystem::exists(sfm))
	{
		std::cerr << "Colmap result direcotry is not correct!" << std::endl;
		return -1;
	}

	// check if all colmap results exist
	boost::filesystem::path sfm_cameras(sSfMDir + "/cameras.txt");
	boost::filesystem::path sfm_images(sSfMDir + "/images.txt");
	boost::filesystem::path sfm_points3D(sSfMDir + "/points3D.txt");
	if (!boost::filesystem::exists(sfm_cameras) || !boost::filesystem::exists(sfm_images) ||
		!boost::filesystem::exists(sfm_points3D))
	{
		std::cerr << "At least one of the colmap result files does not exist in sfm folder: " << sfm << std::endl;
		return -2;
	}

	// Read cameras.txt
	// Here we assume only one common camera.
	// Code from Line3D++.
	std::ifstream cameraFile;
	cameraFile.open(sfm_cameras.c_str());
	std::string camerasLine;

	Mat matK, matDistCoef;

	while (std::getline(cameraFile, camerasLine))
	{
		// check first character for a comment (#)
		if (camerasLine.substr(0, 1).compare("#") != 0)
		{
			std::stringstream cameras_stream(camerasLine);

			unsigned int camID, width, height;
			std::string model;

			// parse essential data
			cameras_stream >> camID >> model >> width >> height;

			float fx, fy, cx, cy, k1, k2, k3, p1, p2;

			// check camera model
			if (model.compare("SIMPLE_PINHOLE") == 0)
			{
				// f,cx,cy
				cameras_stream >> fx >> cx >> cy;
				fy = fx;
				k1 = 0; k2 = 0; k3 = 0;
				p1 = 0; p2 = 0;
			}
			else if (model.compare("PINHOLE") == 0)
			{
				// fx,fy,cx,cy
				cameras_stream >> fx >> fy >> cx >> cy;
				k1 = 0; k2 = 0; k3 = 0;
				p1 = 0; p2 = 0;
			}
			else if (model.compare("SIMPLE_RADIAL") == 0)
			{
				// f,cx,cy,k
				cameras_stream >> fx >> cx >> cy >> k1;
				fy = fx;
				k2 = 0; k3 = 0;
				p1 = 0; p2 = 0;
			}
			else if (model.compare("RADIAL") == 0)
			{
				// f,cx,cy,k1,k2
				cameras_stream >> fx >> cx >> cy >> k1 >> k2;
				fy = fx;
				k3 = 0;
				p1 = 0; p2 = 0;
			}
			else if (model.compare("OPENCV") == 0)
			{
				// fx,fy,cx,cy,k1,k2,p1,p2
				cameras_stream >> fx >> fy >> cx >> cy >> k1 >> k2 >> p1 >> p2;
				k3 = 0;
			}
			else if (model.compare("FULL_OPENCV") == 0)
			{
				// fx,fy,cx,cy,k1,k2,p1,p2,k3[,k4,k5,k6]
				cameras_stream >> fx >> fy >> cx >> cy >> k1 >> k2 >> p1 >> p2 >> k3;
			}
			else
			{
				std::cerr << "camera model " << model << " unknown!" << std::endl;
				std::cerr << "please specify its parameters in the main_colmap.cpp in order to proceed..." << std::endl;
				return -3;
			}

			matK = (Mat_<float>(3, 3, CV_32F) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
			matDistCoef = (Mat_<float>(5, 1, CV_32F) << k1, k2, p1, p2, k3);

			break;
		}
	}
	cameraFile.close();

	// Read images.txt
	std::ifstream imageFile;
	imageFile.open(sfm_images.c_str());
	std::string images_line;

	std::map<unsigned int, Eigen::Matrix3d> cams_R;
	std::map<unsigned int, Eigen::Vector3d> cams_t;
	std::map<unsigned int, Eigen::Vector3d> cams_C;
	std::map<unsigned int, unsigned int> img2cam;
	std::map<unsigned int, std::string> cams_images;
	std::map<unsigned int, std::list<unsigned int> > cams_worldpoints;
	std::map<unsigned int, Eigen::Vector3d> wps_coords;
	std::vector<unsigned int> img_seq;
	unsigned int imgID, camID;
	Frame tmpF;
	int nFrames = 0;

	bool first_line = true;
	while (std::getline(imageFile, images_line))
	{
		// check first character for a comment (#)
		if (images_line.substr(0, 1).compare("#") != 0)
		{
			std::stringstream images_stream(images_line);
			if (first_line)
			{
				// image data
				float qw, qx, qy, qz, tx, ty, tz;
				std::string img_name;

				images_stream >> imgID >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> camID >> img_name;

				stringstream ss(img_name);
				string frameID;
				getline(ss, frameID, '.');
				int nFrameID = atoi(frameID.c_str()) - 1;

				// convert rotation
				Mat Rcw = ConvertQuatFromMat(qw, qx, qy, qz);
				Mat tcw = (Mat_<float>(3, 1, CV_32F) << tx, ty, tz);
				Mat Ocw = -Rcw.t() * tcw;

				tmpF.mK = matK;
				tmpF.mDistCoef = matDistCoef;
				tmpF.mnId = nFrameID;
				tmpF.fx = matK.at<float>(0, 0);
				tmpF.fy = matK.at<float>(1, 1);
				tmpF.cx = matK.at<float>(0, 2);
				tmpF.cy = matK.at<float>(1, 2);
				tmpF.invfx = 1 / tmpF.fx;
				tmpF.invfy = 1 / tmpF.fy;

				// Set Frame Poses
				Mat Tcw = Mat::eye(4, 4, CV_32F);
				Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
				tcw.copyTo(Tcw.rowRange(0, 3).col(3));
				tmpF.SetPose(Tcw);
				
				// For serializing. 
				tmpF.mDescriptors = Mat::zeros(1, 1, CV_32F);
				
				// For serializing. 
				KeyFrame *pKF = new KeyFrame(tmpF, _pMap, SLAM.GetKeyFrameDatabase());
				_pMap->AddKeyFrame(pKF);
				pKF->mTcwGBA = Mat::zeros(4,4,CV_32F);
				pKF->mTcwBefGBA = Mat::zeros(4, 4, CV_32F);
				pKF->mTcp = Mat::zeros(4, 4, CV_32F);
				first_line = false;
				nFrames++;
			}
			else
			{
				// 2D points
				double x, y;
				std::string wpID;
				std::list<unsigned int> wps;
				bool process = true;

				while (process)
				{
					wpID = "";
					images_stream >> x >> y >> wpID;

					if (wpID.length() > 0)
					{
						int wp = atoi(wpID.c_str());

						if (wp >= 0)
						{
							wps.push_back(wp);
							wps_coords[wp] = Eigen::Vector3d(0, 0, 0);
						}
					}
					else
					{
						// end reached
						process = false;
					}
				}
				cams_worldpoints[imgID] = wps;
				first_line = true;
			}
		}
	}
	imageFile.close();

	//std::cout << "found " << cams_R.size() << " images and " << wps_coords.size() << " worldpoints in [images.txt]" << std::endl;
	cout << "Total " << nFrames << " images have retrieved" << endl;


	//	// Main loop
	//	cv::Mat im;
	//	while (iteration > 0) {
	//		// Total number of images : iteration * (forward + backward)
	//		// Backward process leads to runtime-error : to be fixed.
	//
	//		for (int ni = 0; ni < total_nimages; ni++)
	//		{
	//			int frameNum = 0;
	//
	//			if ((ni / nImages) % 2 == 0) { frameNum = (ni % nImages); }
	//			else { frameNum = (nImages - 1) - (ni % nImages); }
	//
	//			cout << frameNum << " processing.." << endl;
	//			// Read image from file
	//			im = cv::imread(imgDir + "/" + vstrImageFilenames[frameNum], CV_LOAD_IMAGE_UNCHANGED);
	//			double tframe = vTimestamps[frameNum];
	//
	//			if (im.empty())
	//			{
	//				cerr << endl << "Failed to load image at: "
	//					<< imgDir << "/" << vstrImageFilenames[frameNum] << endl;
	//				return 1;
	//			}
	//
	//#ifdef COMPILEDWITHC11
	//			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	//#else
	//			std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
	//#endif
	//
	//			// Pass the image to the SLAM system
	//			SLAM.TrackMonocular(im, tframe);
	//
	//#ifdef COMPILEDWITHC11
	//			std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	//#else
	//			std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
	//#endif
	//
	//			double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
	//
	//			vTimesTrack[ni] = ttrack;
	//
	//			//// Wait to load the next frame
	//			//double T = 0;
	//			//if (ni < total_nimages - 1)
	//			//	T = vTimestamps[ni + 1] - tframe;
	//			//else if (ni > 0)
	//			//	T = tframe - vTimestamps[ni - 1];
	//
	//			// Wait to load the next frame
	//			double T = 0;
	//
	//			if ((ni / nImages) % 2 == 0) {
	//				if (ni < total_nimages - 1)
	//					T = vTimestamps[frameNum + 1] - tframe;
	//				else if (ni > 0)
	//					T = tframe - vTimestamps[frameNum - 1];
	//			}
	//			else {
	//				frameNum = (nImages - 1) - (ni % nImages);
	//				if (frameNum > 0) {
	//					T = tframe - vTimestamps[frameNum - 1];
	//				}
	//				else if (frameNum < nImages - 1)
	//					T = tframe - vTimestamps[0];
	//			}
	//
	//			if (ttrack < T)
	//				std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T - ttrack)*1e6)));
	//		}
	//
	//		iteration--;
	//	}
	// Stop all threads
	//SLAM.Shutdown(false);

	//// Tracking time statistics
	//sort(vTimesTrack.begin(), vTimesTrack.end());
	//float totaltime = 0;
	//for (int ni = 0; ni<nImages; ni++)
	//{
	//	totaltime += vTimesTrack[ni];
	//}
	//cout << "-------" << endl << endl;
	//cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	//cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	return 0;

}

Mat ConvertQuatFromMat(const float Qw, const float Qx, const float Qy, const float Qz) {
	// Duplicated from Line3D++.

	float n = Qw*Qw + Qx*Qx + Qy*Qy + Qz*Qz;

	float s;
	float epsilon = 1e-6;
	if (fabs(n) < epsilon)
		s = 0;
	else
		s = 2.0 / n;

	float wx = s*Qw*Qx; float wy = s*Qw*Qy; float wz = s*Qw*Qz;
	float xx = s*Qx*Qx; float xy = s*Qx*Qy; float xz = s*Qx*Qz;
	float yy = s*Qy*Qy; float yz = s*Qy*Qz; float zz = s*Qz*Qz;

	Mat R = (Mat_<float>(3, 3, CV_32F) << 1.0 - (yy + zz), xy - wz, xz + wy,
		xy + wz, 1.0 - (xx + zz), yz - wx,
		xz - wy, yz + wx, 1.0 - (xx + yy));
	return R.clone();
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
	ifstream f;
	f.open(strFile.c_str());
	cout << "Loading images ... ";

	// skip first three lines
	string s0;
	getline(f, s0);
	getline(f, s0);
	getline(f, s0);

	while (!f.eof())
	{
		string s;
		getline(f, s);
		if (!s.empty())
		{
			stringstream ss;
			ss << s;
			double t;
			string sRGB;
			ss >> t;
			vTimestamps.push_back(t);
			ss >> sRGB;
			vstrImageFilenames.push_back(sRGB);
		}
	}
	cout << "Done!!" << endl;

}

int ComputePose(ORB_SLAM2::System &SLAM, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, string &imgDir, int iteration) {

	int nImages = vstrImageFilenames.size();
	int total_nimages = 1 * iteration * nImages;

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(total_nimages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl;
	cout << "Total Iterations : " << iteration << endl << endl;

	// Main loop
	cv::Mat im;
	while (iteration > 0) {
		// Total number of images : iteration * (forward + backward)
		// Backward process leads to runtime-error : to be fixed.

		for (int ni = 0; ni < total_nimages; ni++)
		{
			if (ni > 200)
				continue;
			int frameNum = 0;

			if ((ni / nImages) % 2 == 0) { frameNum = (ni % nImages); }
			else { frameNum = (nImages - 1) - (ni % nImages); }

			cout << frameNum << " processing.." << endl;
			// Read image from file
			im = cv::imread(imgDir + "/" + vstrImageFilenames[frameNum], CV_LOAD_IMAGE_UNCHANGED);
			double tframe = vTimestamps[frameNum];

			if (im.empty())
			{
				cerr << endl << "Failed to load image at: "
					<< imgDir << "/" << vstrImageFilenames[frameNum] << endl;
				return 1;
			}

#ifdef COMPILEDWITHC11
			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
			std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

			// Pass the image to the SLAM system
			SLAM.TrackMonocular(im, tframe);

#ifdef COMPILEDWITHC11
			std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
			std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

			double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

			vTimesTrack[ni] = ttrack;

			//// Wait to load the next frame
			//double T = 0;
			//if (ni < total_nimages - 1)
			//	T = vTimestamps[ni + 1] - tframe;
			//else if (ni > 0)
			//	T = tframe - vTimestamps[ni - 1];

			// Wait to load the next frame
			double T = 0;

			if ((ni / nImages) % 2 == 0) {
				if (ni < total_nimages - 1)
					T = vTimestamps[frameNum + 1] - tframe;
				else if (ni > 0)
					T = tframe - vTimestamps[frameNum - 1];
			}
			else {
				frameNum = (nImages - 1) - (ni % nImages);
				if (frameNum > 0) {
					T = tframe - vTimestamps[frameNum - 1];
				}
				else if (frameNum < nImages - 1)
					T = tframe - vTimestamps[0];
			}

			if (ttrack < T)
				std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T - ttrack)*1e6)));
		}

		iteration--;
	}
	// Stop all threads
	SLAM.Shutdown(false);

	//// Tracking time statistics
	//sort(vTimesTrack.begin(), vTimesTrack.end());
	//float totaltime = 0;
	//for (int ni = 0; ni<nImages; ni++)
	//{
	//	totaltime += vTimesTrack[ni];
	//}
	//cout << "-------" << endl << endl;
	//cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	//cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	return 0;

}

int main(int argc, char **argv)
{
	options_description desc("Allowed options");
	desc.add_options()
		("help,h", "produce a help screen")
		("vocab_path,v", value<string>(), "Path to ORBvoc.bin")
		("parameter_path,p", value<string>(), "Path to parameter file(.yaml format)")
		("img_path,i", value<string>(), "Path to image folder(that includes 'images' folder)")
		("colmap,c", value<string>(), "Path to colmap result for precalculated camera poses.")
		("save_map,s", "Use save/load map function")
		("line_match,m", "Do Line Match")
		("RANSAC,r", "Use RANSAC in line registration")
		("LSD,l", "Use LSD");

	variables_map vm;
	store(parse_command_line(argc, argv, desc), vm);
	if (vm.count("help")) {
		cout << "Display all options" << endl;
		cout << desc << endl;
	}

	if (vm.count("vocab_path")) { cout << "**Vocab_path is set to " << vm["vocab_path"].as<string>() << endl; }
	else {
		cout << "Please specify vocab_path.. " << endl;
		return 0;
	}
	if (vm.count("parameter_path")) { cout << "**Parameter_path is set to " << vm["parameter_path"].as<string>() << endl; }
	else {
		cout << "Please specify parameter_path.. " << endl;
		return 0;
	}
	if (vm.count("img_path")) { cout << "**Img_path is set to " << vm["img_path"].as<string>() << endl; }
	else {
		cout << "Please specify img_path.. " << endl;
		return 0;
	}
	if (vm.count("colmap")) { cout << "Camera pose will be loaded from colmap : " << vm["colmap"].as<string>() << endl << endl; }
	if (vm.count("line_match")) { cout << "Match lines " << endl; }
	if (vm.count("RANSAC")) { cout << "Use RANSAC in initialization for line registration " << endl; }
	if (vm.count("LSD")) { cout << "Uses LSD for line extraction." << endl << endl; }

	
	// System Variables.
	vector<string> vstrImageFilenames;
	vector<double> vTimestamps;
	string imgDir, vocabDir, paramDir, colmapDir;
	string strFile, writeKFinfo, lineDir;
	bool bSaveMap = false;
	bool bPoseFromColmap = false;
	bool bSuccessLoadMap = false;

	// Option for line registration.
	bool bLineMatch = false;
	bool bLineRANSAC = false;
	bool bUseLSD = false;

	int iteration = 1;	// iteration in SLAM mode.
	
	// Set modes for current operation. OffLine : Map generation / OnLine : Localization.
	enum mode { OffLine, OnLine };
	mode currentMode = OffLine;

	// Load variables.
	imgDir = vm["img_path"].as<string>();
	vocabDir = vm["vocab_path"].as<string>();
	paramDir = vm["parameter_path"].as<string>();
	if (vm.count("colmap")) {
		bPoseFromColmap = true;
		colmapDir = vm["colmap"].as<string>();
	}

	bLineMatch = vm.count("line_match");
	bLineRANSAC = vm.count("RANSAC");
	bUseLSD = vm.count("LSD");

	// Load image  file names & time stamps.
	strFile = imgDir + "/images.txt";
	LoadImages(strFile, vstrImageFilenames, vTimestamps);

	// Write KF information into txt file (for further processing in python.)
	writeKFinfo = imgDir + "/undistort/KFinfo.txt";
	lineDir = imgDir + "/results";
	std::cout << "11111" << std::endl;
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(vocabDir, paramDir, ORB_SLAM2::System::MONOCULAR, true, bSaveMap);
	std::cout << "22222" << std::endl;
	if (SLAM.GetReuseMap())
		bSuccessLoadMap = true;
	
	// Generate Global Map using ORB-SLAM. 
	if (!bSuccessLoadMap) {
		if (bPoseFromColmap)
			PoseFromColmap(SLAM, vstrImageFilenames, colmapDir);
		else
			ComputePose(SLAM, vstrImageFilenames, vTimestamps, imgDir, iteration);
	}

	if (currentMode == OffLine) {
		LineMapping LR = LineMapping();
		LR.Undistort(paramDir, vstrImageFilenames, imgDir);
		LR.SetOptions(bLineMatch, bLineRANSAC, bUseLSD);
		LR.LineRegistration(SLAM, vstrImageFilenames, writeKFinfo, imgDir + "/undistort");

		// Stop all threads
		SLAM.Shutdown(true);
	}
	else {
		// Localization using lines.
		int nImages = vstrImageFilenames.size();

		// Vector for tracking time statistics
		vector<float> vTimesTrack;
		vTimesTrack.resize(iteration * nImages);

		cout << endl << "-------" << endl;
		cout << "Start processing sequence ..." << endl;
		cout << "Images in the sequence: " << nImages << endl;
		cout << "Total Iterations : " << iteration << endl << endl;

		// Perform Localization for given query image.  
		// Main loop
		cv::Mat im, lines;
		CIO io;
		for (int ni = 0; ni<nImages; ni++)
		{
			// Read image from file
			im = cv::imread(vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);

			// Load extracted lines. 
			string strLineName = imgDir + "/results/" + to_string(ni + 1) + "_lines.txt";
			char* lineName = &strLineName[0];
			io.loadData(lineName, lines);

			double tframe = vTimestamps[ni];

			if (im.empty())
			{
				cerr << endl << "Failed to load image at: "
					<< vstrImageFilenames[ni] << endl;
				return 1;
			}

#ifdef COMPILEDWITHC11
			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
			std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

			// Pass the image to the SLAM system
			SLAM.TrackMonocularLines(im, lines, tframe);

#ifdef COMPILEDWITHC11
			std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
			std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

			double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

			vTimesTrack[ni] = ttrack;

			// Wait to load the next frame
			double T = 0;
			if (ni<nImages - 1)
				T = vTimestamps[ni + 1] - tframe;
			else if (ni>0)
				T = tframe - vTimestamps[ni - 1];

			if (ttrack<T)
				std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T - ttrack)*1e6)));
		}

	}

	return 0;
}

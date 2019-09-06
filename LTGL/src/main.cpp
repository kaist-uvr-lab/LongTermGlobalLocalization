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
#include"LineMapping.h"

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
	vector<double> &vTimestamps);

int MapGeneration(ORB_SLAM2::System &SLAM, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, string &imgPath, int iteration);

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

int main(int argc, char **argv)
{
	if (argc < 5)
	{
		cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence [1|0](save map?)" << endl;
		return 1;
	}

	// Retrieve paths to images
	vector<string> vstrImageFilenames;
	vector<double> vTimestamps;
	string imgDir = string(argv[3]);
	string strFile = imgDir + "/images.txt";
	LoadImages(strFile, vstrImageFilenames, vTimestamps);

	string writeKFinfo = imgDir + "/undistort/KFinfo.txt";
	string lineDir = imgDir + "/results";

	enum mode {OffLine, OnLine};
	mode currentMode = OffLine;

	int iteration = 1;
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, (bool)atoi(argv[4]));

	// Generate Global Map using ORB-SLAM. 

	bool isMapGeneration = atoi(argv[5]);
	if ((isMapGeneration)) {
		MapGeneration(SLAM, vstrImageFilenames, vTimestamps, imgDir, iteration);
	}

	// If we need undistorted images, perform undistortion. 
	//Undistort(string(argv[2]), vstrImageFilenames, imgDir);
	
	//Undistort(string(argv[2]), vstrImageFilenames, imgDir);

	if (currentMode == OffLine) {
		LineMapping LR = LineMapping();
		LR.Undistort(string(argv[2]), vstrImageFilenames, imgDir);
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


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
	ifstream f;
	f.open(strFile.c_str());
	cout << "Loading images ..." << endl;

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
}

int MapGeneration(ORB_SLAM2::System &SLAM, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, string &imgDir, int iteration) {

	int nImages = vstrImageFilenames.size();

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(iteration * nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl;
	cout << "Total Iterations : " << iteration << endl << endl;

	// Main loop
	cv::Mat im;
	while (iteration > 0) {
		// Total number of images : iteration * (forward + backward)
		// Backward process leads to runtime-error : to be fixed.
		int total_nimages = 1 * iteration * nImages;

		for (int ni = 0; ni < total_nimages; ni++)
		{
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

			// Wait to load the next frame
			double T = 0;
			if (ni < total_nimages - 1)
				T = vTimestamps[ni + 1] - tframe;
			else if (ni > 0)
				T = tframe - vTimestamps[ni - 1];

			if (ttrack < T)
				std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T - ttrack)*1e6)));
		}

		iteration--;
	}
	// Stop all threads
	SLAM.Shutdown(false);

	// Tracking time statistics
	sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for (int ni = 0; ni<nImages; ni++)
	{
		totaltime += vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	return 0;

}

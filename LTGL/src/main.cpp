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

int main(int argc, char **argv)
{
	if (argc != 5)
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

	string writeKFinfo = imgDir + "/KFinfo.txt";
	string lineDir = imgDir + "/results";


	int iteration = 1;
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, (bool)atoi(argv[4]));

	// Generate Global Map using ORB-SLAM. 
	//mapGeneration(SLAM, vstrImageFilenames, vTimestamps, imgDir, iteration);

	// If we need undistorted images, perform undistortion. 
	//Undistort(string(argv[2]), vstrImageFilenames, imgDir);
	
	LineMapping LR = LineMapping();
	LR.Undistort(string(argv[2]), vstrImageFilenames, imgDir);
	LR.LineRegistration(SLAM, vstrImageFilenames, writeKFinfo, imgDir+"/undistort");

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
	SLAM.Shutdown();

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

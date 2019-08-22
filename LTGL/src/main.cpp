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

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
	vector<double> &vTimestamps);

int mapGeneration(ORB_SLAM2::System &SLAM, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, string &imgPath, int iteration);

int lineRegistration(ORB_SLAM2::System &SLAM, vector<string> &vstrLineFilenames, string &writeKFinfo, string &imgDir);

void Undistort(string &strSettingPath, vector<string> &vstrImageFilenames, string &imgDir);

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
	Undistort(string(argv[2]), vstrImageFilenames, imgDir);

	lineRegistration(SLAM, vstrImageFilenames, writeKFinfo, imgDir+"/undistort");

	return 0;
}

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

	cout << "Undistortion done."<<endl;

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

void saveKFinfo(vector<int> vKFindices, string writePath) {
	// Save KF infos into txt. 
	ofstream writeFile(writePath.data());
	if (writeFile.is_open()) {
		for (int i = 0; i < vKFindices.size(); i++) {
			writeFile << to_string(vKFindices[i]+1) << ".png\n";
		}
		writeFile.close();
	}
}

int mapGeneration(ORB_SLAM2::System &SLAM, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, string &imgDir, int iteration) {

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

Mat skewSymMat(float x, float y, float z) {
	Mat Ax = (Mat_<float>(3, 3, CV_32F) << 0, -z, y, z, 0, -x, -y, x ,0);
	return Ax;
}


float magMat(Mat &m) {
	float mag = 0;

	for (int i = 0; i < m.rows; i++) {
		for (int j = 0; j < m.cols; j++) {
			mag += m.at<float>(i, j) * m.at<float>(i, j);
		}
	}
	return sqrt(mag);
}

float pointToLineDist(Mat &pt, Mat &line1, Mat &line2) {

	Mat dir_vec = line1.rowRange(0, 2) - line2.rowRange(0, 2);
	Mat normal_vec = (Mat_<float>(2, 1, CV_32F) << -dir_vec.at<float>(1), dir_vec.at<float>(0));
	normal_vec = normal_vec / magMat(normal_vec);
	Mat vec = pt.rowRange(0, 2) - line2.rowRange(0, 2);
	return abs(normal_vec.dot(vec));
}

int lineRegistration(ORB_SLAM2::System &SLAM, vector<string> &vstrImageFilenames, string &writeKFinfo, string &imgDir) {

	ORB_SLAM2::Map* _mpMap = SLAM.GetMap();
	vector<ORB_SLAM2::KeyFrame*> vpKFS = _mpMap->GetAllKeyFrames();
	vector<int> vKFindices;
	vKFindices.reserve(vpKFS.size());

	// Prepare for line matching. 	
	LSM* lineMatching = new LSM(true);

	//// Get all of the ids of keyframes. 
	for (vector<ORB_SLAM2::KeyFrame*>::iterator vit = vpKFS.begin(), vend = vpKFS.end(); vit != vend; vit++) {
		vKFindices.push_back((*vit)->mnFrameId);
		sort(vKFindices.begin(), vKFindices.end(), less<int>());
		saveKFinfo(vKFindices, writeKFinfo);
	}

	////// Get all of the ids of keyframes. 
	for (vector<ORB_SLAM2::KeyFrame*>::iterator vit = vpKFS.begin(), vend = vpKFS.end(); vit != vend; vit++) {
		// Start with first keyframe. 
		ORB_SLAM2::KeyFrame* pCurrentKF = *vit;
		Mat K = pCurrentKF->mK;
		Mat invK = K.inv();

		//if (pCurrentKF->mnFrameId != 1439)
		//	continue;
		// Perform triangulation only for co-visible keyframes. 
		vector<ORB_SLAM2::KeyFrame*> vCovisibleKFs = pCurrentKF->GetBestCovisibilityKeyFrames(10);

		CIO io;
		Mat lines1, lines2;
		string strImgName1 = imgDir + "/" + vstrImageFilenames[pCurrentKF->mnFrameId];
		string strLineName1 = imgDir + "/results/" + to_string(pCurrentKF->mnFrameId + 1) + "_lines.txt";
		char* imgName1 = &strImgName1[0];
		char* lineName1 = &strLineName1[0];
		io.loadData(lineName1, lines1);
		pCurrentKF->SetExtracted2DLines(lines1);

		for (vector<ORB_SLAM2::KeyFrame*>::iterator vTmpit = vCovisibleKFs.begin(), vTmpend2 = vCovisibleKFs.end(); vTmpit != vTmpend2; vTmpit++) {
			ORB_SLAM2::KeyFrame* pTmpKF = *vTmpit;

			// Perform Line Matching First. 
			string strImgName2 = imgDir + "/" + vstrImageFilenames[pTmpKF->mnFrameId];
			string strLineName2 = imgDir + "/results/" + to_string(pTmpKF->mnFrameId + 1) + "_lines.txt";
			char* imgName2 = &strImgName2[0];
			char* lineName2 = &strLineName2[0];
			io.loadData(lineName2, lines2);
			pTmpKF->SetExtracted2DLines(lines2);

			cout << imgName1 << endl;
			cout << imgName2 << endl;

			// **********To DO ******************** //
			// Later, F Matrix should be changed into the one calculated in SLAM. 
			// For now, F Matrix is calculated through L-J-L points. 
			lineMatching->setImgLines(imgName1, imgName2, lineName1, lineName2);

			// matchedLines has a form of  (ps1.x, ps1.y, pe1.x, pe1.y, ps2.x, ps2.y, pe2.x, pe2.y)xN rows.
			pair<Mat*, Mat*> mLines = lineMatching->lsm(lines1, lines2);
			Mat* matchedLines = mLines.first;
			Mat* matchedLineIndices = mLines.second;

			int nMatchedLines = matchedLines->rows;

			// Triangulation for each pairs. 
			// First compare camera centers to avoid pure rotation case. 

			// First get Plucker Coordinates of triangulated lines.  
			Mat Ocw1 = pCurrentKF->GetCameraCenter();
			Mat Ocw2 = pTmpKF->GetCameraCenter();
			Mat Rcw1 = pCurrentKF->GetRotation();
			Mat Rcw2 = pTmpKF->GetRotation();
			Mat Tcw1 = pCurrentKF->GetPose();
			Mat Tcw2 = pTmpKF->GetPose();
			Mat Twc1 = Tcw1.inv();
			Mat Twc2 = Tcw2.inv();
			
			for (int i = 0; i < nMatchedLines; i++) {
				Mat matchedPts = matchedLines->row(i);
				//Eigen::Vector3d normPtS1(matchedPts.at<float>(0), matchedPts.at<float>(1), 1);
				//Eigen::Vector3d normPtE1(matchedPts.at<float>(2), matchedPts.at<float>(3), 1);
				//Eigen::Vector3d normPtS2(matchedPts.at<float>(4), matchedPts.at<float>(5), 1);
				//Eigen::Vector3d normPtE2(matchedPts.at<float>(6), matchedPts.at<float>(7), 1);

				Mat ptS1 = (Mat_<float>(3, 1, CV_32F) << matchedPts.at<float>(0), matchedPts.at<float>(1), 1);
				Mat ptE1 = (Mat_<float>(3, 1, CV_32F) << matchedPts.at<float>(2), matchedPts.at<float>(3), 1);
				Mat ptS2 = (Mat_<float>(3, 1, CV_32F) << matchedPts.at<float>(4), matchedPts.at<float>(5), 1);
				Mat ptE2 = (Mat_<float>(3, 1, CV_32F) << matchedPts.at<float>(6), matchedPts.at<float>(7), 1);

				// Get normalized coordinates
				Mat normPtS1 = invK * ptS1;
				Mat normPtE1 = invK * ptE1;
				Mat normPtS2 = invK * ptS2;
				Mat normPtE2 = invK * ptE2;

				// Get plane p1 = [px, py, pz, pw] in world coordinates
				Mat plane1 = Mat::zeros(4, 1, CV_32F);
				Mat normalC1 = Mat::zeros(3, 1, CV_32F);   // nomral in C1 coordinate
				normalC1 = skewSymMat(normPtS1.at<float>(0), normPtS1.at<float>(1), 1) * normPtE1;
				Mat normalW1 = Rcw1.t() * normalC1;
				normalW1.rowRange(0, 3).copyTo(plane1.rowRange(0, 3));
				plane1.at<float>(3) = -normalC1.dot(Ocw1);

				// Get plane p2 = [px', py', pz', pw'] in world coordinates
				Mat plane2 = Mat::zeros(4, 1, CV_32F);
				Mat normalC2 = Mat::zeros(3, 1, CV_32F);   // nomral in C2 coordinate
				normalC2 = skewSymMat(normPtS2.at<float>(0), normPtS2.at<float>(1), 1) * normPtE2;
				Mat normalW2 = Rcw2.t() * normalC2;
				normalW2.rowRange(0, 3).copyTo(plane2.rowRange(0, 3));
				plane2.at<float>(3) = -normalC2.dot(Ocw2);

				// Triangulate only if we have enough parallax.
				float test1 = (Rcw1.t() * normPtS1).dot(normalW2);
				float depthTest1 = pCurrentKF->ComputeSceneMedianDepth(2);
				float angle = acos(normalW1.dot(normalW2) / (magMat(normalW1)*magMat(normalW2))) * 180 / 3.141592;
				if (abs(angle) < 0.5)
					continue;

				// Get Plucker Coordinate L in world coordinates from Dual Plucker Coordinates 
				Mat dual_L = plane1 * plane2.t() - plane2 * plane1.t();
				Mat d_vector = (Mat_<float>(3, 1, CV_32F) << -dual_L.at<float>(1, 2), dual_L.at<float>(0, 2), -dual_L.at<float>(0, 1));
				Mat n_vector = dual_L.col(3).rowRange(0, 3);
				Mat triangulated_line = Mat::zeros(6, 1, CV_32F);
				n_vector.copyTo(triangulated_line.rowRange(0, 3));
				d_vector.copyTo(triangulated_line.rowRange(3, 6));

				float depthS1 = ((Ocw2 - Ocw1).dot(normalW2)) / ((Rcw1.t() * normPtS1).dot(normalW2));
				float depthE1 = ((Ocw2 - Ocw1).dot(normalW2)) / ((Rcw1.t() * normPtE1).dot(normalW2));
				float depthS2 = ((Ocw1 - Ocw2).dot(normalW1)) / ((Rcw2.t() * normPtS2).dot(normalW1));
				float depthE2 = ((Ocw1 - Ocw2).dot(normalW1)) / ((Rcw2.t() * normPtE2).dot(normalW1));
		
				// Depth should be greater than zero. 
				if (depthS1 < 0 || depthE1 < 0 || depthS2 < 0 || depthE2 < 0)
					continue;

				Mat homoPtsS1 = (Mat_<float>(4, 1, CV_32F) << normPtS1.at<float>(0), normPtS1.at<float>(1), normPtS1.at<float>(2), 1 / depthS1);
				Mat point3DS1 = depthS1 * Twc1 * homoPtsS1;
				Mat homoPtsE1 = (Mat_<float>(4, 1, CV_32F) << normPtE1.at<float>(0), normPtE1.at<float>(1), normPtE1.at<float>(2), 1 / depthE1);
				Mat point3DE1 = depthE1 * Twc1 * homoPtsE1;

				Mat homoPtsS2 = (Mat_<float>(4, 1, CV_32F) << normPtS2.at<float>(0), normPtS2.at<float>(1), normPtS2.at<float>(2), 1 / depthS2);
				Mat point3DS2 = depthS2 * Twc2 * homoPtsS2;
				Mat homoPtsE2 = (Mat_<float>(4, 1, CV_32F) << normPtE2.at<float>(0), normPtE2.at<float>(1), normPtE2.at<float>(2), 1 / depthE2);
				Mat point3DE2 = depthE2 * Twc2 * homoPtsE2;

				// Check reprojection error. 
				Mat projectedS2C1 = K * Tcw1.rowRange(0, 3) * point3DS2;
				Mat projectedE2C1 = K * Tcw1.rowRange(0, 3) * point3DE2;
				Mat projectedS1C2 = K * Tcw2.rowRange(0, 3) * point3DS1;
				Mat projectedE1C2 = K * Tcw2.rowRange(0, 3) * point3DE1;

				projectedS2C1 = projectedS2C1 / projectedS2C1.at<float>(2);
				projectedE2C1 = projectedE2C1 / projectedE2C1.at<float>(2);
				projectedS1C2 = projectedS1C2 / projectedS1C2.at<float>(2);
				projectedE1C2 = projectedE1C2 / projectedE1C2.at<float>(2);

				// It should be less than 1. Must be almost identical. 
				float dist1 = pointToLineDist(projectedS2C1, ptS1, ptE1);
				float dist2 = pointToLineDist(projectedE2C1, ptS1, ptE1);
				float dist3 = pointToLineDist(projectedS1C2, ptS2, ptE2);
				float dist4 = pointToLineDist(projectedE1C2, ptS2, ptE2);
				
				Mat matArray[] = { point3DS1 , point3DE1, point3DS2, point3DE2 };
				Mat endPts;
				cv::vconcat(matArray, 4, endPts);
				
				ORB_SLAM2::Line3d *newLine = new ORB_SLAM2::Line3d(triangulated_line, endPts, _mpMap);
				newLine->AddObservation(pCurrentKF, matchedLineIndices->at<int>(i, 0));
				newLine->AddObservation(pTmpKF, matchedLineIndices->at<int>(i, 1));

				pCurrentKF->AddLine3D(newLine, matchedLineIndices->at<int>(i, 0));
				pTmpKF->AddLine3D(newLine, matchedLineIndices->at<int>(i, 1));

				_mpMap->AddLine3d(newLine);

			}
		}
	}


	return 0;
}

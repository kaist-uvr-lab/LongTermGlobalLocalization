#pragma once

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include"boost/filesystem.hpp"

using namespace ORB_SLAM2;

class LineMapping {
public:

	
	void test();

	// For cross product. 
	cv::Mat SkewSymMat(float x, float y, float z);
	cv::Mat SkewSymMat(cv::Mat &m);

	// For image undistortion. 
	void Undistort(string &strSettingPath, vector<string> &vstrImageFilenames, string &imgDir);

	// Calculate Magnitude of given Matrix.
	float MagMat(cv::Mat &m);

	// Calcuate point to line distance. 
	float PointToLineDist(cv::Mat &pt, cv::Mat &line1, cv::Mat &line2);

	// Save Keyframe information from given indices.
	void SaveKFinfo(vector<int> vKFindices, string writePath);

	// Compute F Matrix from T and K.
	cv::Mat ComputeFMatrix(const cv::Mat &_T, const cv::Mat &_K);

	// Two View Triangulation.
	int TwoViewTriangulation(pair<cv::Mat*, cv::Mat*> _pairLines, const cv::Mat &_K, const cv::Mat &_invK, KeyFrame *_pKF1, KeyFrame *_pKF2, Map *_pMap);

	// 3D Line Registration. 
	int LineRegistration(ORB_SLAM2::System &SLAM, vector<string> &vstrLineFilenames, string &writeKFinfo, string &imgDir);

private:

};

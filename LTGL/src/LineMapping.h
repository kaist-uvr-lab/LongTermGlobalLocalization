#pragma once

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include"boost/filesystem.hpp"

using namespace ORB_SLAM2;

class LineMapping {
public:

	// For cross product. 
	cv::Mat SkewSymMat(float x, float y, float z);
	cv::Mat SkewSymMat(cv::Mat &m);

	// For image undistortion. 
	void Undistort(string &strSettingPath, vector<string> &vstrImageFilenames, string &imgDir);

	// Find borders after undistortion. 
	void ComputeImageBounds(const cv::Mat &K, const cv::Mat &distCoef, cv::Mat &im);

	// Filter out lines on the boundaries.
	void FilterBoundaryLines(cv::Mat &lines);

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

	// Add only Observations.
	int CollectObservations(pair<cv::Mat*, cv::Mat*> _pairLines, const cv::Mat &_K, const cv::Mat &_invK, KeyFrame *_pKF1, KeyFrame *_pKF2, Map *_pMap);

	// Initialize the line parameter via RANSAC.
	void InitializeLine3dRANSAC(vector<KeyFrame*> _vKFs, Map *_mpMap);

	// Initialize via two-view triangulation.
	bool InitializeLineParam(KeyFrame *_pKF1, KeyFrame *_pKF2, const cv::Mat &line2d1, const cv::Mat &line2d2, cv::Mat &tmpPlucker, Map *_pMap);

	// Compute Score for given line model
	pair<float, int> ComputeModelScore(const cv::Mat &tmpPlucker, const cv::Mat &K, map<KeyFrame*, size_t> allObservations, map<KeyFrame*, bool> &inlierIndex, const float th = sqrt(5.994) * 2);

	// 3D Line Registration. 
	int LineRegistration(ORB_SLAM2::System &SLAM, vector<string> &vstrLineFilenames, string &writeKFinfo, string &imgDir);

	// Test with visualization
	void TestVisualization(vector<ORB_SLAM2::KeyFrame*> vpKFS, string &imgDir, vector<string> &vstrImageFilenames);

	// Set line registration option;
	void SetOptions(bool isLineRegisterInitDone, bool isLineRANSACInitDone, bool isLSD);
private:
	float mnMinX, mnMinY, mnMaxX, mnMaxY;

	// You may change it.
	bool mbIsLineRegisterInit = false;
	bool mbIsLineRANSACInit = false;
	bool mbIsLSD = false;
};

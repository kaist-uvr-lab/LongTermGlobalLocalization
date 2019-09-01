
#include<System.h>

#include"LineMapping.h"
#include"Line3d.h"
#include<LSM/src/LSM.h>
#include<opencv2/core/core.hpp>
#include "LineOptimizer.h"

void LineMapping::test() {
	int a = 1;
}

cv::Mat LineMapping::SkewSymMat(float x, float y, float z){
	Mat Ax = (Mat_<float>(3, 3, CV_32F) << 0, -z, y, z, 0, -x, -y, x, 0);
	return Ax.clone();
}

cv::Mat LineMapping::SkewSymMat(cv::Mat &m) {
	// when m is a vector composed of 3 values. 
	return SkewSymMat(m.at<float>(0), m.at<float>(1), m.at<float>(2));
}

void LineMapping::Undistort(string &strSettingPath, vector<string> &vstrImageFilenames, string &imgDir) {

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

float LineMapping::MagMat(cv::Mat &m) {
	float mag = 0;

	for (int i = 0; i < m.rows; i++) {
		for (int j = 0; j < m.cols; j++) {
			mag += m.at<float>(i, j) * m.at<float>(i, j);
		}
	}
	return sqrt(mag);
}

float LineMapping::PointToLineDist(cv::Mat &pt, cv::Mat &line1, cv::Mat &line2) {
	cv::Mat dir_vec = line1.rowRange(0, 2) - line2.rowRange(0, 2);
	cv::Mat normal_vec = (cv::Mat_<float>(2, 1, CV_32F) << -dir_vec.at<float>(1), dir_vec.at<float>(0));
	normal_vec = normal_vec / MagMat(normal_vec);
	cv::Mat vec = pt.rowRange(0, 2) - line2.rowRange(0, 2);
	return abs(normal_vec.dot(vec));
}

void LineMapping::SaveKFinfo(vector<int> vKFindices, string writePath) {
	// Save KF infos into txt. 
	ofstream writeFile(writePath.data());
	if (writeFile.is_open()) {
		for (int i = 0; i < vKFindices.size(); i++) {
			writeFile << to_string(vKFindices[i] + 1) << ".png\n";
		}
		writeFile.close();
	}
}

cv::Mat LineMapping::ComputeFMatrix(const cv::Mat &_T, const cv::Mat &_K){
	//Compute F Matrix from given Relative Camera Pose.
	//R,t should be relative matrix.(C1 -> C2)
	cv::Mat R = _T.rowRange(0, 3).colRange(0, 3);
	cv::Mat t = _T.rowRange(0, 3).col(3);
	cv::Mat Kinv = _K.inv();
	cv::Mat F = Kinv.t() * SkewSymMat(t) * R * Kinv;
	return F.clone();
}


int LineMapping::TwoViewTriangulation(pair<Mat*, Mat*> _pairLines, const Mat &_K, const Mat &_invK, KeyFrame *_pKF1, KeyFrame *_pKF2, Map *_pMap) {
	
	int nCreatedLines = 0;
	Mat K = _K;
	Mat invK = _invK;

	Mat* matchedLines = _pairLines.first;
	Mat* matchedLineIndices = _pairLines.second;

	int nMatchedLines = matchedLines->rows;

	if (nMatchedLines == 0) {
		cout << "Pass triangulation" << endl;
	}

	// First get Plucker Coordinates of triangulated lines.  
	Mat Ocw1 = _pKF1->GetCameraCenter();
	Mat Ocw2 = _pKF2->GetCameraCenter();
	Mat Rcw1 = _pKF1->GetRotation();
	Mat Rcw2 = _pKF2->GetRotation();
	Mat Tcw1 = _pKF1->GetPose();
	Mat Tcw2 = _pKF2->GetPose();
	Mat Twc1 = Tcw1.inv();
	Mat Twc2 = Tcw2.inv();

	for (int i = 0; i < nMatchedLines; i++) {
		Mat matchedPts = matchedLines->row(i);

		// If 3D line has already registered, pass it. 
		Line3d *pLine3d1 = _pKF1->Get3DLine(matchedLineIndices->at<int>(i, 0));
		Line3d *pLine3d2 = _pKF2->Get3DLine(matchedLineIndices->at<int>(i, 1));

		// Instead of Fuse process, we simply add observation if the line is already registered. 
		if (pLine3d1) {
			if (pLine3d2) {
				//both are already registered. 
				continue;
			}
			else {
				// Line1 is already registered so only add observation for Line2. 
				pLine3d1->AddObservation(_pKF2, matchedLineIndices->at<int>(i, 1));
				_pKF2->AddLine3D(pLine3d1, matchedLineIndices->at<int>(i, 1));
				continue;
			}
		}
		else {
			if (pLine3d2) {
				// Line2 is already registered so only add observation for Line1. 
				pLine3d2->AddObservation(_pKF1, matchedLineIndices->at<int>(i, 0));
				_pKF1->AddLine3D(pLine3d2, matchedLineIndices->at<int>(i, 0));
				continue;
			}
		}

		Mat Rcw1 = Tcw1.rowRange(0, 3).colRange(0, 3);
		Mat Rcw2 = Tcw2.rowRange(0, 3).colRange(0, 3);

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
		normalC1 = SkewSymMat(normPtS1.at<float>(0), normPtS1.at<float>(1), 1) * normPtE1;
		Mat normalW1 = Rcw1.t() * normalC1;
		normalW1.rowRange(0, 3).copyTo(plane1.rowRange(0, 3));
		plane1.at<float>(3) = -normalC1.dot(Ocw1);

		// Get plane p2 = [px', py', pz', pw'] in world coordinates
		Mat plane2 = Mat::zeros(4, 1, CV_32F);
		Mat normalC2 = Mat::zeros(3, 1, CV_32F);   // nomral in C2 coordinate
		normalC2 = SkewSymMat(normPtS2.at<float>(0), normPtS2.at<float>(1), 1) * normPtE2;
		Mat normalW2 = Rcw2.t() * normalC2;
		normalW2.rowRange(0, 3).copyTo(plane2.rowRange(0, 3));
		plane2.at<float>(3) = -normalC2.dot(Ocw2);

		// Triangulate only if we have enough parallax.
		float test1 = (Rcw1.t() * normPtS1).dot(normalW2);
		float depthTest1 = _pKF1->ComputeSceneMedianDepth(2);
		float angle = acos(normalW1.dot(normalW2) / (MagMat(normalW1)*MagMat(normalW2))) * 180 / 3.141592;
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
		float dist1 = PointToLineDist(projectedS2C1, ptS1, ptE1);
		float dist2 = PointToLineDist(projectedE2C1, ptS1, ptE1);
		float dist3 = PointToLineDist(projectedS1C2, ptS2, ptE2);
		float dist4 = PointToLineDist(projectedE1C2, ptS2, ptE2);

		Mat matArray[] = { point3DS1 , point3DE1, point3DS2, point3DE2 };
		Mat endPts;
		cv::vconcat(matArray, 4, endPts);

		ORB_SLAM2::Line3d *newLine = new ORB_SLAM2::Line3d(triangulated_line, endPts, _pMap);
		newLine->AddObservation(_pKF1, matchedLineIndices->at<int>(i, 0));
		newLine->AddObservation(_pKF2, matchedLineIndices->at<int>(i, 1));

		_pKF1->AddLine3D(newLine, matchedLineIndices->at<int>(i, 0));
		_pKF2->AddLine3D(newLine, matchedLineIndices->at<int>(i, 1));

		_pMap->AddLine3d(newLine);
		nCreatedLines++;
	}

	return nCreatedLines;
}

int LineMapping::LineRegistration(ORB_SLAM2::System &SLAM, vector<string> &vstrImageFilenames, string &writeKFinfo, string &imgDir) {

	ORB_SLAM2::Map* _mpMap = SLAM.GetMap();
	vector<ORB_SLAM2::KeyFrame*> vpKFS = _mpMap->GetAllKeyFrames();
	vector<int> vKFindices;
	vKFindices.reserve(vpKFS.size());

	// Prepare for line matching.
	bool isProvidedLines = true;
	bool isPrecomputedF = true;
	LSM* lineMatching = new LSM(isProvidedLines, isPrecomputedF);

	// Get all of the ids of keyframes. 
		
	for (vector<ORB_SLAM2::KeyFrame*>::iterator vit = vpKFS.begin(), vend = vpKFS.end(); vit != vend; vit++) {
		vKFindices.push_back((*vit)->mnFrameId);
		sort(vKFindices.begin(), vKFindices.end(), less<int>());
	}
	
	

	// Save KF info into text. 
	SaveKFinfo(vKFindices, writeKFinfo);
	
	int count = 0;
	int totalNlines = 0;
	vector<int> vDoneIdx;
	vDoneIdx.reserve(vpKFS.size());

	////// Get all of the ids of keyframes. 
	for (vector<ORB_SLAM2::KeyFrame*>::iterator vit = vpKFS.begin(), vend = vpKFS.end(); vit != vend; vit++) {
		// Start with first keyframe. 
		ORB_SLAM2::KeyFrame* pCurrentKF = *vit;
		Mat K = pCurrentKF->mK;
		Mat invK = K.inv();

		cout << count << "/" << vpKFS .size() << "KeyFrames has done. " <<endl;
		count++;
		//if (count < 34)
		//	continue;

		// Perform triangulation only for co-visible keyframes. 
		vector<ORB_SLAM2::KeyFrame*> vCovisibleKFs = pCurrentKF->GetBestCovisibilityKeyFrames(15);

		CIO io;
		Mat lines1, lines2;
		string strImgName1 = imgDir + "/" + vstrImageFilenames[pCurrentKF->mnFrameId];
		string strLineName1 = imgDir + "/results/" + to_string(pCurrentKF->mnFrameId + 1) + "_lines.txt";
		char* imgName1 = &strImgName1[0];
		char* lineName1 = &strLineName1[0];
		io.loadData(lineName1, lines1);
		pCurrentKF->SetExtracted2DLines(lines1);

		// Starts from farthest frame
		for (vector<ORB_SLAM2::KeyFrame*>::reverse_iterator vTmpit= vCovisibleKFs.rbegin(); vTmpit != vCovisibleKFs.rend(); ++vTmpit) {
		//for (vector<ORB_SLAM2::KeyFrame*>::iterator vTmpit = vCovisibleKFs.begin(), vTmpend2 = vCovisibleKFs.end(); vTmpit != vTmpend2; vTmpit++) {
			ORB_SLAM2::KeyFrame* pTmpKF = *vTmpit;

			// If it's already processed, pass. 
			if (find(vDoneIdx.begin(), vDoneIdx.end(), pTmpKF->mnFrameId) != vDoneIdx.end())
				continue;						

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

			// Compute F Matrix from computed R,t. 
			Mat Tcw1 = pCurrentKF->GetPose();
			Mat Tcw2 = pTmpKF->GetPose();
			Mat T21 = Tcw2 * Tcw1.inv();
			Mat Fmat = ComputeFMatrix(T21, pCurrentKF->mK);

			// matchedLines has a form of  (ps1.x, ps1.y, pe1.x, pe1.y, ps2.x, ps2.y, pe2.x, pe2.y)xN rows.
			if (isPrecomputedF)
				lineMatching->setFmat(Fmat);
			pair<Mat*, Mat*> mLines = lineMatching->lsm(lines1, lines2);
			int nCreatedLines = TwoViewTriangulation(mLines, K, invK, pCurrentKF, pTmpKF, _mpMap);

			totalNlines += nCreatedLines;
			cout << "************* " << nCreatedLines << " lines have newly created.. || Total created lines so far : "<< totalNlines << " *************\n" <<endl;
			
			vector<Line3d*> lines = pTmpKF->Get3DLines();

			for (vector<ORB_SLAM2::Line3d*>::iterator vit = lines.begin(), vend = lines.end(); vit != vend; vit++) {
				//Vertex. 
				Line3d* pLine = *vit;
				if (!pLine)
					continue;
				if (pLine->GetNumObservations() < 3)
					continue;
				ORB_SLAM2::LineOptimizer::LineOptimization(pLine);
			}


			/****************To do**************************************************/
			//After lines have added, Optimization are done for robust registration. 
			//if(nCreatedLines > 0)
			//ORB_SLAM2::Optimizer::GlobalStructureOnlyBA.. 
		}
		vDoneIdx.push_back(pCurrentKF->mnFrameId);

	}

	cout << "Line Registration done. Total " << totalNlines << " lines are created. \n" << endl;

	return 0;
}

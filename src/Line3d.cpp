
#include "Line3d.h"

namespace ORB_SLAM2 {
	//Describes 3D lines. 
	Line3d::Line3d() {}

	Line3d::Line3d(cv::Mat &_plucker, cv::Mat &_endPts, Map* pMap) :mPlucker(_plucker), mEndPts(_endPts), mpMap(pMap), nObs(0) {}

	void Line3d::AddObservation(KeyFrame* pKF, size_t idx)
	{
		if (mLineObservations.count(pKF))
			return;
		mLineObservations[pKF] = idx;
		nObs++;
	}
	void Line3d::EraseObservation(KeyFrame* pKF)
	{
		bool bBad = false;
		{
			if (mLineObservations.count(pKF))
			{
				int idx = mLineObservations[pKF];
				nObs--;
				mLineObservations.erase(pKF);

				//// If only 2 observations or less, discard point
				//if (nObs <= 2)
				//	bBad = true;
			}
		}
	}
	
	void Line3d::AddCPObservation(KeyFrame* pKF, set<size_t> vIndices)
	{
		set<size_t> stmpIdx = vIndices;
		for (set<size_t>::iterator sit = stmpIdx.begin(), send = stmpIdx.end(); sit != send; sit++) {
			mCPLineObservations[pKF].insert(*sit);
		}
	}

	void Line3d::EraseCPObservation(KeyFrame* pKF, size_t idx)
	{
		set<size_t>::iterator findIdx = mCPLineObservations[pKF].find(idx);
		if (findIdx != mCPLineObservations[pKF].end()) {
			mCPLineObservations[pKF].erase(findIdx);
		}
	}

	map<KeyFrame*, set<size_t>> Line3d::GetCPLineObservations() {
		return mCPLineObservations;
	}

	set<size_t> Line3d::GetCPLineObservations(KeyFrame *pKF) {
		return mCPLineObservations[pKF];
	}

	void Line3d::UpdateCoplanarLine3d() {
		// For all observations, gather coplanar 2d lines and finally update coplanar 3d lines. 
		for (map<KeyFrame*, size_t>::iterator mit = mLineObservations.begin(), mend = mLineObservations.end(); mit != mend; mit++) {
			KeyFrame* pTmpKF = mit->first;
			size_t sTmpIdx = mit->second;

			set<size_t> sCPLineObervation = mCPLineObservations[pTmpKF];
			for (set<size_t>::iterator sit = sCPLineObervation.begin(), send = sCPLineObervation.end(); sit != send; sit++) {
				Line3d *pTmpLine3d = pTmpKF->Get3DLine(*sit);
				if (!pTmpLine3d)
					continue;
				
				AddCoplanarLine3d(pTmpLine3d);
			}
		}
	}

	void Line3d::UpdateEndpts() {
		//Update all of the endpts. 
		cv::Mat tmpAllEndpts;
		cv::Mat tmp1Ddist;
		cv::Mat sortedIdx;
		int count = 0;
		for (map<KeyFrame*, size_t>::iterator vit = mLineObservations.begin(), vend = mLineObservations.end(); vit != vend; vit++) {
			if (count >= 2)
				continue;

			count++;
			KeyFrame* pKF = vit->first;
			cv::Mat tmpLines = pKF->Get2DLine(vit->second);

			//Get two 3D lines passing through two normalized end points.
			cv::Mat Ocw1 = pKF->GetCameraCenter();
			cv::Mat Rcw1 = pKF->GetRotation();
			cv::Mat Tcw1 = pKF->GetPose();
			cv::Mat Twc1 = Tcw1.inv();
			cv::Mat K = pKF->mK;
			cv::Mat invK = K.inv();
			cv::Mat ptS1 = (cv::Mat_<float>(3, 1, CV_32F) << tmpLines.at<float>(0), tmpLines.at<float>(1), 1);
			cv::Mat ptE1 = (cv::Mat_<float>(3, 1, CV_32F) << tmpLines.at<float>(2), tmpLines.at<float>(3), 1);

			// Get normalized coordinates (viewing ray in Camera coordinates)
			cv::Mat normPtS1 = invK * ptS1;
			cv::Mat normPtE1 = invK * ptE1;

			// Convert viewing ray into world coordinates.
			cv::Mat worldRayS1 = Rcw1.t() * normPtS1;
			cv::Mat worldRayE1 = Rcw1.t() * normPtE1;

			// Get a point and n_vec from plucker coordinates of registered 3D lines. 
			cv::Mat p1 = ClosestPointfromOrigin();
			cv::Mat d1 = mPlucker.rowRange(3, 6);
			d1 = d1 / MagMat(d1);

			cv::Mat newEndpt1 = ClosestPointfromLine(p1, d1, Ocw1.rowRange(0, 3), worldRayS1).t();
			cv::Mat newEndpt2 = ClosestPointfromLine(p1, d1, Ocw1.rowRange(0, 3), worldRayE1).t();
			tmpAllEndpts.push_back(newEndpt1.clone());
			tmpAllEndpts.push_back(newEndpt2.clone());

			float dist1 = ComputeSigned1DDist(p1, newEndpt1.t());
			float dist2 = ComputeSigned1DDist(p1, newEndpt2.t());
			tmp1Ddist.push_back(dist1);
			tmp1Ddist.push_back(dist2);
		}

		cv::sortIdx(tmp1Ddist, sortedIdx, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

		// Update Endpts
		/*cv::Mat newEndpt1 = tmpAllEndpts.row(0);
		cv::Mat newEndpt2 = tmpAllEndpts.row(1);*/
		cv::Mat newEndpt1 = tmpAllEndpts.row(sortedIdx.at<int>(0));
		cv::Mat newEndpt2 = tmpAllEndpts.row(sortedIdx.at<int>(sortedIdx.rows - 1));
		//int quarter = (sortedIdx.rows) / 4;
		//int idx1 = quarter;
		//int idx2 = (sortedIdx.rows-1) - quarter;
		//cv::Mat newEndpt1 = tmpAllEndpts.row(sortedIdx.at<int>(idx1));
		//cv::Mat newEndpt2 = tmpAllEndpts.row(sortedIdx.at<int>(idx2));
		newEndpt1.copyTo(mEndPts.row(0).colRange(0, 3));
		newEndpt2.copyTo(mEndPts.row(1).colRange(0, 3));
	}

	cv::Mat Line3d::ClosestPointfromOrigin() {
		cv::Mat p_ortho;
		if (mPlucker.empty()) {
			cerr << "plucker coordinate is empty!!" << endl;
			return p_ortho;
		}

		cv::Mat nvec = mPlucker.rowRange(0, 3);
		cv::Mat dvec = mPlucker.rowRange(3, 6);
		float magNvect = MagMat(dvec);

		nvec = nvec / magNvect;
		dvec = dvec / magNvect;

		p_ortho = dvec.cross(nvec);
		return p_ortho.clone();
	}

	int Line3d::GetIndexInKeyFrame(KeyFrame *pKF)
	{
		if (mLineObservations.count(pKF))
			return mLineObservations[pKF];
		else
			return -1;
	}

	cv::Mat Line3d::ClosestPointfromLine(cv::Mat p1, cv::Mat d1, cv::Mat p2, cv::Mat d2) {

		cv::Mat t = p1 - p2;
		cv::Mat q = d1.cross(d2);
		float q_mag = MagMat(q);
		float q_mag_sqr = q_mag*q_mag;
		cv::Mat r1 = p1 + ((q.dot(d2.cross(t))) / q_mag_sqr) * d1;
		return r1.clone();
	}

	float Line3d::MagMat(const cv::Mat &m) {
		float mag = 0;

		for (int i = 0; i < m.rows; i++) {
			for (int j = 0; j < m.cols; j++) {
				mag += m.at<float>(i, j) * m.at<float>(i, j);
			}
		}
		return sqrt(mag);
	}

	float Line3d::ComputeSigned1DDist(const cv::Mat &m1, const cv::Mat &m2) {
		if (m1.rows != m2.rows) {
			cerr << "Size of rows don't match between m1, m2! " << endl;
			return 0;
		}
		else if (m1.cols != m2.cols) {
			cerr << "Size of cols don't match between m1, m2! " << endl;
			return 0;
		}

		float mag = 0;

		for (int i = 0; i < m1.rows; i++) {
			for (int j = 0; j < m1.cols; j++) {
				float diff = m1.at<float>(i, j) - m2.at<float>(i, j);
				mag += diff*diff;
			}
		}

		// Assuming 1D, return signed dist.
		if (m1.at<float>(0, 0) > m2.at<float>(0, 0))
			return sqrt(mag);
		else
			return -sqrt(mag);


	}

	// Add Coplanar lines.
	void Line3d::AddCoplanarLine3d(Line3d* pLine3d) {
		msCoplanarLine3ds.insert(pLine3d);
	}

	// Erase Coplanar lines.
	void Line3d::EraseCoplanarLine3d(Line3d* pLine3d) {
		msCoplanarLine3ds.erase(pLine3d);
	}


	// Get all of the Coplanar lines.
	set<Line3d*> Line3d::GetCoplanarLine3d() {
		return msCoplanarLine3ds;
	}

#ifdef FUNC_MAP_SAVE_LOAD
	template<class Archive>
	void Line3d::serialize(Archive &ar, const unsigned int version)
	{
		// don't save the mutex
		ar & mPlucker;
		ar & mEndPts;
		ar & mLineObservations;
		ar & mpMap;
		ar & nObs;
		ar & mCPLineObservations;
		ar & msCoplanarLine3ds;
		ar & nCoPlanarLine3ds;

		//ar & mnId & nNextId & mnFirstKFid & mnFirstFrame & nObs;
		//// Tracking related vars
		//ar & mTrackProjX;
		//ar & mTrackProjY;
		//ar & mTrackProjXR;
		//ar & mbTrackInView;
		//ar & mnTrackScaleLevel;
		//ar & mTrackViewCos;
		//ar & mnTrackReferenceForFrame;
		//ar & mnLastFrameSeen;
		//// Local Mapping related vars
		//ar & mnBALocalForKF & mnFuseCandidateForKF;
		//// Loop Closing related vars
		//ar & mnLoopPointForKF & mnCorrectedByKF & mnCorrectedReference & mPosGBA & mnBAGlobalForKF;
		//// don't save the mutex
		//ar & mWorldPos;
		//ar & mObservations;
		//ar & mNormalVector;
		//ar & mDescriptor;
		//ar & mpRefKF;
		//ar & mnVisible & mnFound;
		//ar & mbBad & mpReplaced;
		//ar & mfMinDistance & mfMaxDistance;
		//ar & mpMap;
		//// don't save the mutex
	}
	template void Line3d::serialize(boost::archive::binary_iarchive&, const unsigned int);
	template void Line3d::serialize(boost::archive::binary_oarchive&, const unsigned int);
#endif

}
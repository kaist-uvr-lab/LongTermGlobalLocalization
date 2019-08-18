
#include "Line3d.h"

namespace ORB_SLAM2 {
	//Describes 3D lines. 
	Line3d::Line3d() {}

	Line3d::Line3d(cv::Mat &_plucker, cv::Mat &_endPts, Map* pMap):mPlucker(_plucker),mEndPts(_endPts), mpMap(pMap), nObs(0){}

	void Line3d::AddObservation(KeyFrame* pKF, size_t idx)
	{
		if (mLineObservations.count(pKF))
			return;
		mLineObservations[pKF] = idx;

		nObs++;
	}

}
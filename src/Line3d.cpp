
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
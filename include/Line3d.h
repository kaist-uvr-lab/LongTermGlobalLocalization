#ifndef LINE3D_H
#define LINE3D_H

#include <System.h>
#include <KeyFrame.h>

namespace ORB_SLAM2 {
	class Map;
	class KeyFrame;

	//Describes 3D lines. 
	class Line3d {

	public:
		Line3d(cv::Mat &_plucker, cv::Mat &_endPts, Map* pMap);

		void SetPluckerWorld(cv::Mat &_plucker) { _plucker.copyTo(mPlucker); };
		void SetEndPts(pair<float, float> &_endPts) { _endPts = make_pair(_endPts.first, _endPts.second); };

		cv::Mat GetPluckerWorld() { return mPlucker; };
		cv::Mat GetEndPts() { return mEndPts; };

		void Line3d::AddObservation(KeyFrame* pKF, size_t idx);

	private:
		// Position in plucker coordinate & world coordinate
		cv::Mat mPlucker, mEndPts;

		// Keyframes observing the line and associated index in keyframe
		std::map<KeyFrame*, size_t> mLineObservations;

		// Reference KeyFrame
		Map* mpMap;

		// number of observed Keyframes
		int nObs;

#ifdef FUNC_MAP_SAVE_LOAD
	public:
		// for serialization
		Line3d();
	private:
		// serialize is recommended to be private
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive &ar, const unsigned int version);
#endif
	};

} // namespace ORB_SLAM

#endif // LINE3D_H
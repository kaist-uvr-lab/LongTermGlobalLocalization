
#include <System.h>
#include <KeyFrame.h>

namespace ORB_SLAM2 {
	class Map;
	class KeyFrame;

	//Describes 3D lines. 
	class Line3d {

	public:
		Line3d();

		void SetPluckerWorld(cv::Mat &_plucker) { _plucker.copyTo(mPlucker); };
		void SetEndPts(pair<float, float> &_endPts) { _endPts = make_pair(_endPts.first, _endPts.second); };

		cv::Mat GetPluckerWorld() { return mPlucker; };
		pair<float, float> GetEndPts() { return mEndPts; };

	private:
		// Position in plucker coordinate in world coordinate
		cv::Mat mPlucker;
		pair<float, float> mEndPts;

		// Keyframes observing the line and associated index in keyframe
		std::map<KeyFrame*, size_t> mLineObservations;

		// Reference KeyFrame
		Map* mpMap;


	};

}
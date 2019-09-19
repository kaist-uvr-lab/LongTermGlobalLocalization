#ifndef JUNCTION_H
#define JUNCTION_H

#include "KeyFrame.h"

namespace ORB_SLAM2 {

	class KeyFrame;

	class LineSegment {
	public:
		LineSegment();
		LineSegment(KeyFrame *pKF, int idx) :mpKF(pKF), mIdx(idx) {};

	private:
		KeyFrame *mpKF;
		int mIdx;
	};
	
	class Junction {
	public:
		Junction();
		Junction(LineSegment *pLS1, LineSegment *pLS2) :mpLS1(pLS1), mpLS2(pLS2) {};
		Junction(LineSegment *pLS1, LineSegment *pLS2, cv::Point3f worldPos) :mpLS1(pLS1), mpLS2(pLS2), mWorldPos(worldPos) {};

		void SetWorldPos(cv::Point3f worldPos) { mWorldPos = worldPos; };
		cv::Point3f GetWorldPops() { return mWorldPos; };

	private:
		LineSegment *mpLS1, *mpLS2;
		cv::Point3f mWorldPos;
	};

	class JunctionPair {

	public:
		JunctionPair();
		JunctionPair(Junction *junction1, Junction *junction2) :mJunctionImg1(junction1), mJunctionImg2(junction2) { CreateJunction(mJunctionImg1, mJunctionImg2); };
		JunctionPair(KeyFrame *pKF1, KeyFrame *pKF2, int img1LineIdx1, int img1LineIdx2, int img2LineIdx1, int img2LineIdx2) {

			mLS1_Img1 = new LineSegment(pKF1, img1LineIdx1);
			mLS2_Img1 = new LineSegment(pKF1, img1LineIdx2);
			mLS1_Img2 = new LineSegment(pKF2, img2LineIdx1);
			mLS2_Img2 = new LineSegment(pKF2, img2LineIdx2);

			mJunctionImg1 = new Junction(mLS1_Img1, mLS2_Img1);
			mJunctionImg2 = new Junction(mLS1_Img2, mLS2_Img2);
			
			CreateJunction(mJunctionImg1, mJunctionImg2);
		};

		// Return junction. 
		pair<Junction*, Junction*> GetJunctionPair() { return mJunctionPair; };
		void CreateJunction(Junction *junction1, Junction *junction2) { mJunctionPair = make_pair(junction1, junction2); };

	private:
		pair<Junction*, Junction*> mJunctionPair;
		Junction *mJunctionImg1, *mJunctionImg2;
		LineSegment *mLS1_Img1, *mLS2_Img1, *mLS1_Img2, *mLS2_Img2;
	};
}

#endif

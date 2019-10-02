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
		~Line3d();
		Line3d(cv::Mat &_plucker, cv::Mat &_endPts, Map* pMap);

		void SetPluckerWorld(cv::Mat &_plucker) { _plucker.copyTo(mPlucker); };
		void SetEndPts(cv::Mat &_endPts) { _endPts.copyTo(mEndPts); };

		cv::Mat GetPluckerWorld() { return mPlucker; };
		cv::Mat GetEndPts() { return mEndPts; };

		void Line3d::AddObservation(KeyFrame* pKF, size_t idx);
		std::map<KeyFrame*, size_t> GetObservations() {
			return mLineObservations;
		}
		int GetNumObservations() {
			return nObs;
		}

		// return magnitude of the matrix.
		float MagMat(const cv::Mat &m);

		// return distance between two points; 
		float ComputeSigned1DDist(const cv::Mat &m1, const cv::Mat &m2);

		// Update Endpts after optimization
		void UpdateEndpts();

		// Compute closest point from origin using Plucker Coordinate.
		// Using it for getting a point passing through the line.
		cv::Mat ClosestPointfromOrigin();

		// Compute a point on L1 that makes closest distance between L1, L2. 
		cv::Mat ClosestPointfromLine(cv::Mat p1, cv::Mat d1, cv::Mat p2, cv::Mat d2);

		// Get Line3d index in give Keyframe
		int GetIndexInKeyFrame(KeyFrame *pKF);

		// Erase given KF from the observation of Line3d.
		void EraseObservation(KeyFrame* pKF);

		// Add coplanar line observation for this Line3d.
		void AddCPObservation(KeyFrame* pKF, set<size_t> vIndices);

		// For given KF, erase all of coplanar line observations for this Line3d.
		void EraseCPObservations(KeyFrame* pKF, size_t idx);

		// For given KF, erase coplanar line observation for this Line3d.
		void EraseKFCPObservation(KeyFrame* pKF);

		// For given KF, erase a single coplanar line observation for this Line3d.
		void EraseSingleCPObservation(KeyFrame* pKF, size_t idx);

		// Get all Coplanar line observations.
		std::map<KeyFrame*, set<size_t>> GetCPLineObservations();

		// Get all Coplanar line observations in given KF.
		std::set<size_t> GetCPLineObservations(KeyFrame *pKF);

		// Add Coplanar 3D lines.
		void AddCoplanarLine3d(Line3d* pLine3d);

		// Erase Coplanar 3D lines. 
		void EraseCoplanarLine3d(Line3d* pLine3d);

		// Get all of the Coplanar lines.
		set<Line3d*> GetCoplanarLine3d();

		// Update coplanar line3ds infos. 
		void UpdateCoplanarLine3d();

		void SetIsSelected(bool bSelect) { bIsSelected = bSelect; };
		bool GetIsSelected() { return bIsSelected; };

	private:
		// Position in plucker coordinate & world coordinate
		cv::Mat mPlucker, mEndPts;

		// Keyframes observing the line and associated index in keyframe
		std::map<KeyFrame*, size_t> mLineObservations;

		// number of observed Keyframes
		int nObs;

		// Keyframes observing the coplanar line and associated index in keyframe
		std::map<KeyFrame*, set<size_t>> mCPLineObservations;

		// Reference KeyFrame
		Map* mpMap;

		// coplanar lines
		set<Line3d*> msCoplanarLine3ds;

		// number of coplanar lines
		int nCoPlanarLine3ds;

		// just for viz. 
		bool bIsSelected;

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
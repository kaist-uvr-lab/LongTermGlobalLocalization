#ifndef LINEOPTIMIZER_H
#define LINEOPTIMIZER_H

#include "Line3d.h"
#include "opencv2\opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

namespace ORB_SLAM2 {
	class Map;
	class KeyFrame;

	//Describes 3D lines. 
	class LineOptimizer {

	public:
		void static LineOptimization(Line3d* pLine);
		void static LineJunctionOptimization();

	private:
	};

} // namespace ORB_SLAM

#endif // LINEOPTIMIZER_H
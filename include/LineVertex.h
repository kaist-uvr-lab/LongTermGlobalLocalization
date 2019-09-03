#ifndef LINEVERTEX_H
#define LINEVERTEX_H

#include "Line3d.h"
#include "../Thirdparty/g2o/g2o/core/base_vertex.h"
#include "../Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "opencv2\opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace g2o {

	using namespace Eigen;
	//typedef BlockSolver< BlockSolverTraits<4, 2> > BlockSolver_4_2;

	class LineConverter {
	public:
		static Vector4d ConvertParam(cv::Mat L);
		static Vector3d ConvertEulerAnglesFromRotationMatrix(Matrix3d R);
		static Matrix<double, 3, 3> ConvertRotationMatrixFromEulerAngles(Vector3d vec);
		static Vector6d ConvertPlukerCoordinates(const Vector4d param);
		static void CalcTheta(double phi, double& w1, double& w2);
	};

	class LineVertex : public BaseVertex<4, Eigen::Vector4d>{

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		LineVertex();
		bool read(std::istream& is);
		bool write(std::ostream& os) const;
		virtual void setToOriginImpl() {
			_estimate.fill(0.);
		}
		virtual void oplusImpl(const double* update_) {
			
			/*Eigen::Map<const Eigen::Vector4d> v(update_);*/
<<<<<<< HEAD
			Eigen::Map<const Eigen::Vector4d> v(update_);
=======
			Eigen::Vector4d v(update_);
>>>>>>> 95e2869da816d2542bdf0b9b73f212b4f27eff42
			//std::cout << "update param = " << v<<std::endl<<"before param="<<_estimate << std::endl;
			_estimate += v;
			//setEstimate(v + estimate());
			//std::cout << "after = " << estimate() << std::endl;

		}

		
		//Vector6d Lw;

	private:
	};

	class EdgeLineOptimization : public BaseUnaryEdge<2, Eigen::Vector2d, LineVertex> {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeLineOptimization();
		bool read(std::istream& is);

		bool write(std::ostream& os) const;

		void computeError();
		virtual void linearizeOplus();

		Vector3d spt, ept;
		Matrix<double, 6, 6> T;
		Matrix<double, 3, 3> K;


	};

} // namespace ORB_SLAM

#endif // LINEVERTEX_H
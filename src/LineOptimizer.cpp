#include "LineOptimizer.h"
#include "LineVertex.h"
#include <opencv2/core/eigen.hpp>
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace ORB_SLAM2{

	cv::Mat SKEW(cv::Mat mat) {
		cv::Mat res = cv::Mat::zeros(3, 3, CV_32FC1);
		res.at<float>(0, 1) = -mat.at<float>(2);
		res.at<float>(1, 0) = mat.at<float>(2);
		res.at<float>(0, 2) = mat.at<float>(1);
		res.at<float>(2, 0) = -mat.at<float>(1);
		res.at<float>(1, 2) = -mat.at<float>(0);
		res.at<float>(2, 1) = mat.at<float>(0);
		return res;
	}

	void LineOptimizer::LineOptimization(Line3d* pLine){
		std::cout << "LineOptimization\n";

		g2o::SparseOptimizer optimizer;

		//
		g2o::BlockSolver_4_2::LinearSolverType * linearSolver;

		linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_4_2::PoseMatrixType>();

		g2o::BlockSolver_4_2 * solver_ptr = new g2o::BlockSolver_4_2(linearSolver);

		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		optimizer.setAlgorithm(solver);
		
		//Set Line Vertex
		g2o::LineVertex* pLineVertex = new g2o::LineVertex();
		//플리커 코디네잇에서 변환하는 과정
		pLineVertex->setEstimate(g2o::LineConverter::ConvertParam(pLine->GetPluckerWorld()));
		pLineVertex->setId(0);
		pLineVertex->setFixed(false);

		//cv::Mat Lw = pLine->GetPluckerWorld();
		//cv::cv2eigen(Lw, pLineVertex->Lw);
		
		optimizer.addVertex(pLineVertex);

		//Set Line Edge
		std::vector<g2o::EdgeLineOptimization*> vpEdgesLineOptimization;
		vpEdgesLineOptimization.reserve(pLine->GetNumObservations());

		//chi값의 설정 필요
		const float deltaMono = sqrt(5.991);

		auto mObservations = pLine->GetObservations();
		for (auto iter = mObservations.begin(); iter != mObservations.end(); iter++) {

			KeyFrame* pKF = iter->first;
			int idx = iter->second;
			
			//type은 float
			
			cv::Mat R = pKF->GetRotation();
			cv::Mat t = pKF->GetTranslation();

			cv::Mat SkewT = SKEW(t);
			SkewT *= R;

			cv::Mat K = pKF->mK.clone();
			cv::Mat line2D = pKF->Get2DLine(idx);
			
			cv::Mat spt = cv::Mat::ones(3, 1, CV_32FC1);
			spt.at<float>(0) = line2D.at<float>(0);
			spt.at<float>(1) = line2D.at<float>(1);
			cv::Mat ept = cv::Mat::ones(3, 1, CV_32FC1);
			ept.at<float>(0) = line2D.at<float>(2);
			ept.at<float>(1) = line2D.at<float>(3);
			//spt = K.inv()*spt;
			//ept = K.inv()*ept;
			cv::Mat T = cv::Mat::zeros(6, 6, CV_32FC1);
			R.copyTo(T.colRange(0, 3).rowRange(0, 3));
			R.copyTo(T.colRange(3, 6).rowRange(3, 6));
			SkewT.copyTo(T.colRange(3, 6).rowRange(0, 3));

			//std::cout <<"LIne2D::"<<line2D.type()<<", "<<K.type()<<", "<<spt<<ept << std::endl;

			g2o::EdgeLineOptimization* e = new g2o::EdgeLineOptimization();
			e->setLevel(0);
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));

			e->setInformation(Eigen::Matrix2d::Identity());
			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			rk->setDelta(deltaMono);
			e->setRobustKernel(rk);
			
			e->spt << spt.at<float>(0), spt.at<float>(1), spt.at<float>(2);
			e->spt << spt.at<float>(0), spt.at<float>(1), spt.at<float>(2);

			// Missing assignment for e->ept
			e->ept << ept.at<float>(0), ept.at<float>(1), ept.at<float>(2);
			e->ept << ept.at<float>(0), ept.at<float>(1), ept.at<float>(2);


			//e->T = Eigen::Matrix<double, 6, 6>(T.data);

			float fx = pKF->fx;
			float fy = pKF->fy;
			float cx = pKF->cx;
			float cy = pKF->cy;
			cv::Mat eK = cv::Mat::zeros(3, 3, CV_32FC1);
			eK.at<float>(0, 0) = fy;
			eK.at<float>(1, 1) = fx;
			eK.at<float>(2, 0) = -fy*cx;
			eK.at<float>(2, 1) = -fx*cy;
			eK.at<float>(2, 2) = fx*fy;

			cv::cv2eigen(T, e->T);
			cv::cv2eigen(eK, e->K);

			//std::cout << e->T<<T << std::endl;

			optimizer.addEdge(e);
			vpEdgesLineOptimization.push_back(e);

			//e->computeError();

			//추가적으로 필요한 값 설정
			//T : Lw를 Lc로 변경하기 위한 것
			//K : Lc를 이미지에 프로젝션하기 위한 것
			//end points 설정 3x1 2개이면 될 듯
		}


		//optimizations
		const float chi2Mono[4] = { 5.991,5.991,5.991,5.991 };
		const int its[4] = { 10,10,10,10 };

		for (size_t it = 0; it < 4; it++)
		{
			//nBad = 0;
			optimizer.initializeOptimization(0);
			
			//std::cout << "before starting optimize" << std::endl;
			//for (size_t i = 0, iend = vpEdgesLineOptimization.size(); i < iend; i++)
			//{

			//	g2o::EdgeLineOptimization* e = vpEdgesLineOptimization[i];
			//	float chi2 = e->chi2();
			//	std::cout << "iter=" << it << "err::" << chi2 << std::endl;
			//}
			//
			optimizer.optimize(its[it]);

			//for (size_t i = 0, iend = vpEdgesLineOptimization.size(); i < iend; i++)
			//{
			//	
			//	g2o::EdgeLineOptimization* e = vpEdgesLineOptimization[i];
			//	float chi2 = e->chi2();
			//	std::cout <<"iter="<<it<< "err::" << chi2 << std::endl;
			//}
			
			//g2o::LineVertex* vRecover = static_cast<g2o::LineVertex*>(optimizer.vertex(0));
			//cv::Mat Lw;
			//Eigen::Vector4d param = vRecover->estimate();
			//cv::eigen2cv(g2o::LineConverter::ConvertPlukerCoordinates(vRecover->estimate()), Lw);
			//std::cout << "test = " << Lw << std::endl << "param = " << param << std::endl;
		}
		cv::Mat Lw;
		g2o::LineVertex* vRecover = static_cast<g2o::LineVertex*>(optimizer.vertex(0));
		Eigen::Vector4d param = vRecover->estimate();
		cv::eigen2cv(g2o::LineConverter::ConvertPlukerCoordinates(vRecover->estimate()), Lw);
		Lw.convertTo(Lw, CV_32F);
		pLine->SetPluckerWorld(Lw);
		
		//cv::Mat temp = cv::Mat::zeros(100, 100, CV_8UC1);
		//imshow("test", temp);
		//cv::waitKey(0);
	}
}
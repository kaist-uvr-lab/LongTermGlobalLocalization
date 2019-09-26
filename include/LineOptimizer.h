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

		void LineOptimizer::LineJunctionOptimization() {
			//Junction
			//Line

			g2o::SparseOptimizer optimizer;
			g2o::BlockSolver_4_4::LinearSolverType *linearSolver;
			linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_4_4::PoseMatrixType>();
			g2o::BlockSolver_4_4 * solver_ptr = new g2o::BlockSolver_4_4(linearSolver);
			g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
			optimizer.setAlgorithm(solver);

			//Add Line Vertex
			vector<g2o::LineVertex*> vpLineVertices;
			vector<Line3d*> vpLines;

			for (int i = 0; i < vpLines.size(); i++) {
				g2o::LineVertex* pLineVertex = new g2o::LineVertex();
				//플리커 코디네잇에서 변환하는 과정
				pLineVertex->setEstimate(g2o::LineConverter::ConvertParam(vpLines[i]->GetPluckerWorld()));
				pLineVertex->setId(0);
				pLineVertex->setFixed(false);

				optimizer.addVertex(pLineVertex);
				vpLineVertices.push_back(pLineVertex);
			}
			//chi값의 설정 필요
			const float deltaMono = sqrt(5.991);
			//line edge
			std::vector<g2o::EdgeLineOptimization*> vpEdgesLineOptimization;

			for (int i = 0; i < vpLines.size(); i++) {
				Line3d* pLine = vpLines[i];

				auto mObservations = pLine->GetObservations();
				for (auto iter = mObservations.begin(); iter != mObservations.end(); iter++) {

					KeyFrame* pKF = iter->first;
					int idx = iter->second;

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

					cv::Mat T = cv::Mat::zeros(6, 6, CV_32FC1);
					R.copyTo(T.colRange(0, 3).rowRange(0, 3));
					R.copyTo(T.colRange(3, 6).rowRange(3, 6));
					SkewT.copyTo(T.colRange(3, 6).rowRange(0, 3));

					g2o::EdgeLineOptimization* e = new g2o::EdgeLineOptimization();
					e->setLevel(0);
					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vpLineVertices[i]));

					e->spt << spt.at<float>(0), spt.at<float>(1), spt.at<float>(2);
					e->ept << ept.at<float>(0), ept.at<float>(1), ept.at<float>(2);
					float dx = spt.at<float>(0) - ept.at<float>(0);
					float dy = spt.at<float>(1) - ept.at<float>(1);
					float length = sqrt(dx*dx + dy*dy);
					float alpha = 100;
					e->length = length;
					e->alpha = alpha;

					e->setInformation(Eigen::Matrix2d::Identity());
					g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
					//rk->setDelta(deltaMono);
					// Set adaptive threshold for huber cost function.
					rk->setDelta(deltaMono * alpha / length);
					e->setRobustKernel(rk);

					//e->T = Eigen::Matrix<double, 6, 6>(T.data);

					//float fx = pKF->fx;
					//float fy = pKF->fy;
					//float cx = pKF->cx;
					//float cy = pKF->cy;
					float fx = K.at<float>(0, 0);
					float fy = K.at<float>(1, 1);
					float cx = K.at<float>(0, 2);
					float cy = K.at<float>(1, 2);
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
				}//for observations
			}//for vplines

			 //Line Junction Edges
			vector<g2o::LineJunctionOptimizationEdge*> vpLineJunctionEdges;
			vector<JunctionPair*> vpJunctions;

			for (int i = 0; i < vpJunctions.size(); i++) {
				JunctionPair* junctionPair = vpJunctions[i];
				auto tjunctionPair = junctionPair->GetJunctionPair();
				Junction* junc1 = tjunctionPair.first;
				Junction* junc2 = tjunctionPair.second;

				Line3d* line1;
				Line3d* line2;
				int lIdx1, lIdx2;

				g2o::LineJunctionOptimizationEdge* e = new g2o::LineJunctionOptimizationEdge();
				e->setLevel(0);
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vpLineVertices[lIdx1]));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vpLineVertices[lIdx2]));
				optimizer.addEdge(e);
				vpLineJunctionEdges.push_back(e);
			}

			const float chi2Mono[4] = { 5.991,5.991,5.991,5.991 };
			const int its[4] = { 10,10,10,10 };

			for (size_t it = 0; it < 4; it++)
			{
				//nBad = 0;
				optimizer.initializeOptimization(0);
				optimizer.optimize(its[it]);
			}

			//restore line data
			for (int i = 0; i < vpLineVertices.size(); i++) {
				Line3d* pLine = vpLines[i];
				cv::Mat Lw;
				g2o::LineVertex* vRecover = static_cast<g2o::LineVertex*>(optimizer.vertex(i));
				cv::eigen2cv(g2o::LineConverter::ConvertPlukerCoordinates(vRecover->estimate()), Lw);
				Lw.convertTo(Lw, CV_32F);
				pLine->SetPluckerWorld(Lw);
			}


		}


	private:
	};

} // namespace ORB_SLAM

#endif // LINEOPTIMIZER_H
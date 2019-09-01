#include "LineVertex.h"
#include "math.h"

namespace  g2o {


	LineVertex::LineVertex():BaseVertex<4, Vector4d>() {

	}
	bool LineVertex::read(std::istream& is) {
		Vector4d lv;
		for (int i = 0; i<4; i++)
			is >> _estimate[i];
		return true;
	}
	bool LineVertex::write(std::ostream& os) const {
		Vector4d lv = estimate();
		for (int i = 0; i< 4; i++) {
			os << lv[i] << " ";
		}
		return os.good();
	}
	EdgeLineOptimization::EdgeLineOptimization():BaseUnaryEdge<2, Vector2d, LineVertex>(){}
	bool EdgeLineOptimization::read(std::istream& is) {
		std::cout << "read???" << std::endl;
		return true;
	}

	bool EdgeLineOptimization::write(std::ostream& os) const {
		std::cout << "WRITE???" << std::endl;
		return true;
	}

	void EdgeLineOptimization::computeError() {
		const LineVertex* v1 = static_cast<const LineVertex*>(_vertices[0]);
		Vector4d param = v1->estimate();
		Vector6d Lw = LineConverter::ConvertPlukerCoordinates(param);
		Vector6d Lc = T*Lw;
		Vector3d Nc = Lc.block<3, 1>(0, 0);
		Vector3d Limg = K*Nc;//;K*Nc;
		//Vector3d Nimg = Limg.block<3, 1>(0, 0);

		double len1 = sqrt(Limg[0] * Limg[0] + Limg[1] * Limg[1]);
		_error[0] = Limg.dot(spt) / len1;
		_error[1] = Limg.dot(ept) / len1;

		//double len2 = Nc[0] * Nc[0] + Nc[1] * Nc[1];
		//std::cout <<"Limg::"<< Limg << std::endl<<"Nc="<<Lc.block<3,1>(0,0);
		//std::cout << Nc.dot(spt) << ", " << Nc.dot(ept) << std::endl;
		//std::cout<<
		//estimate를 L로 변환해야 함
		//Eigen::Matrix<double, 2,3>
	}
	void EdgeLineOptimization::linearizeOplus() {
		
		//const LineVertex* v1 = static_cast<const LineVertex*>(_vertices[0]);
		LineVertex* v1 = (LineVertex*)_vertices[0];
		Vector4d param = v1->estimate();
		Vector6d Lw = LineConverter::ConvertPlukerCoordinates(param);
		Vector6d Lc = T*Lw;

		Vector3d Nc = Lc.block<3, 1>(0, 0);
		double len = sqrt(Nc[0] * Nc[0] + Nc[1] * Nc[1]);
		double len3 = len*len*len;
		Matrix<double, 2, 3> J1;
		J1.row(0) << Nc[0] * spt.dot(Nc) / len3 + spt[0] / len, Nc[1] * spt.dot(Nc) / len3 + spt[1] / len, 1 / len;
		J1.row(1) << Nc[0] * ept.dot(Nc) / len3 + ept[0] / len, Nc[1] * ept.dot(Nc) / len3 + ept[1] / len, 1 / len;

		Matrix<double, 3, 6> J2;
		J2.block<3, 3>(0, 0) = K;
		Matrix<double, 6, 4> J3;

		Vector3d psi = v1->estimate().block<3, 1>(0, 0);
		Matrix3d U = LineConverter::ConvertRotationMatrixFromEulerAngles(psi);
		double phi = v1->estimate()[3];

		double w1, w2;
		LineConverter::CalcTheta(phi, w1, w2);

		J3.block<3, 1>(0, 1) = -w1 * U.col(2);
		J3.block<3, 1>(0, 2) = w1 * U.col(1);
		J3.block<3, 1>(0, 3) = -w2 * U.col(0);

		J3.block<3, 1>(3, 0) = w2 * U.col(2);
		J3.block<3, 1>(3, 2) = -w2 * U.col(0);
		J3.block<3, 1>(3, 3) = w1 * U.col(1);

		J3 = T.inverse()*J3;

		Matrix<double, 2, 4> J = J1*J2*J3;

		//std::cout << "J=" << J << std::endl;
		
		_jacobianOplusXi(0, 0) = J(0, 0);
		_jacobianOplusXi(0, 1) = J(0, 1);
		_jacobianOplusXi(0, 2) = J(0, 2);
		_jacobianOplusXi(0, 3) = J(0, 3);

		_jacobianOplusXi(1, 0) = J(1, 0);
		_jacobianOplusXi(1, 1) = J(1, 1);
		_jacobianOplusXi(1, 2) = J(1, 2);
		_jacobianOplusXi(1, 3) = J(1, 3);

		//std::cout << "J=" << _jacobianOplusXi << std::endl;
	}

	
	Vector4d LineConverter::ConvertParam(cv::Mat _L) {
		Vector6d L;
		cv::cv2eigen(_L, L);
		Vector3d N = L.block<3, 1>(0, 0);
		Vector3d d = L.block<3, 1>(3, 0);

		double Ndist = sqrt(N.dot(N));
		double Ddist = sqrt(d.dot(d));

		Vector3d Nnorm = N/ Ndist;
		Vector3d Dnorm = d/ Ddist;
		Vector3d ND = N.cross(d);
		double NDDist = sqrt(ND.dot(ND));
		Vector3d NDnorm = ND / NDDist;
		
		double dist = sqrt(Ndist*Ndist + Ddist*Ddist);
		//U
		Matrix3d U;
		U.col(0) = Nnorm;
		U.col(1) = Dnorm;
		U.col(2) = NDnorm;
		Vector3d psi = ConvertEulerAnglesFromRotationMatrix(U);
		//w
		//angle or radian
		
		double phi1 = acos(Ndist / dist);
		double test2 = cos(phi1);

		Vector4d res;
		res.block<3, 1>(0, 0) = psi;
		res(3) = phi1;

		Vector6d Lw2 = ConvertPlukerCoordinates(res);
		double testDist = sqrt(Ndist*Ndist + Ddist*Ddist);
		
		//std::cout << "L1 = " << L / testDist << std::endl << "L2=" << Lw2 << std::endl;
		Matrix3d U2 = ConvertRotationMatrixFromEulerAngles(psi);
		//std::cout << "U==" << U << std::endl << "U2=" << U2 << ", " << std::endl;
		//std::cout<<"COS::"<<cos(0.0)<<", "<<cos(90.0)<<", "<<cos(90.0/180*CV_PI) << std::endl;
		//std::cout << "param = " << res << std::endl;
		return res;
	}

	Vector3d LineConverter::ConvertEulerAnglesFromRotationMatrix(Matrix3d R) {
		double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

		bool singular = sy < 1e-6; // If

		float x, y, z;
		if (!singular)
		{
			x = atan2(R(2, 1), R(2, 2));
			y = atan2(-R(2, 0), sy);
			z = atan2(R(1, 0), R(0, 0));
		}
		else
		{
			x = atan2(-R(1, 2), R(1, 1));
			y = atan2(-R(2, 0), sy);
			z = 0;
		};
		return Vector3d(x, y, z);
	}
	Matrix<double, 3, 3> LineConverter::ConvertRotationMatrixFromEulerAngles(Vector3d vec) {
		Matrix3d Rx, Ry, Rz;
		Rx << 1, 0, 0,
			0, cos(vec[0]), -sin(vec[0]),
			0, sin(vec[0]), cos(vec[0]);
		Ry << cos(vec[1]), 0, sin(vec[1]),
			0, 1, 0,
			-sin(vec[1]), 0, cos(vec[1]);
		Rz << cos(vec[2]), -sin(vec[2]), 0,
			sin(vec[2]), cos(vec[2]), 0,
			0, 0, 1;
		return Rz*Ry*Rx;
	}
	void LineConverter::CalcTheta(double phi, double& w1, double& w2) {
		w1 = std::cos(phi);
		w2 = std::sin(phi);
	}
	Vector6d LineConverter::ConvertPlukerCoordinates(const Vector4d param) {
		Vector3d psi = param.block<3, 1>(0, 0);
		double phi = param[3];

		double w1, w2;
		CalcTheta(phi, w1, w2);

		Matrix3d U = ConvertRotationMatrixFromEulerAngles(psi);
		Vector6d res;
		res << w1*U.col(0), w2*U.col(1);
		return res;

	}
}
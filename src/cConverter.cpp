/**
* This file is part of MultiCol-SLAM
*
* Copyright (C) 2015-2016 Steffen Urban <urbste at googlemail.com>
* For more information see <https://github.com/urbste/MultiCol-SLAM>
*
* MultiCol-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* MultiCol-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with MultiCol-SLAM . If not, see <http://www.gnu.org/licenses/>.
*/

/*
* MultiCol-SLAM is based on ORB-SLAM2 which was also released under GPLv3
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
*/

#include "cConverter.h"

namespace MultiColSLAM
{
	cv::Matx44d cConverter::invMat(const cv::Matx44d& M)
	{
		cv::Matx33d R = M.get_minor<3, 3>(0, 0);
		R = R.t();
		cv::Vec3d t(M(0, 3), M(1, 3), M(2, 3));
		t = -R * t;
		cv::Matx44d out(
			R(0, 0), R(0, 1), R(0, 2), t(0),
			R(1, 0), R(1, 1), R(1, 2), t(1),
			R(2, 0), R(2, 1), R(2, 2), t(2),
			0.0, 0.0, 0.0, 1.0);

		return out;
	}

	cv::Matx<double, 4, 4> cConverter::ogv2ocv(const Eigen::Matrix<double, 3, 4>& ogv_mat)
	{
		cv::Matx34d ocv_mat;
		cv::eigen2cv(ogv_mat, ocv_mat);

		return cv::Matx<double, 4, 4>(
			ocv_mat(0, 0), ocv_mat(0, 1), ocv_mat(0, 2), ocv_mat(0, 3),
			ocv_mat(1, 0), ocv_mat(1, 1), ocv_mat(1, 2), ocv_mat(1, 3),
			ocv_mat(2, 0), ocv_mat(2, 1), ocv_mat(2, 2), ocv_mat(2, 3),
			0.0, 0.0, 0.0, 1.0);
	}

	std::vector<cv::Mat> cConverter::toDescriptorVector(const std::vector<cv::Mat>& Descriptors)
	{
		std::vector<cv::Mat> vDesc;
		for (int c = 0; c < Descriptors.size(); ++c)
			for (int j = 0; j < Descriptors[c].rows; j++)
				vDesc.push_back(Descriptors[c].row(j));

		return vDesc;
	}

	std::vector<cv::Mat> cConverter::toDescriptorVector(const cv::Mat& Descriptors)
	{
		std::vector<cv::Mat> vDesc;
		vDesc.reserve(Descriptors.rows);
		for (int j = 0; j < Descriptors.rows; j++)
			vDesc.push_back(Descriptors.row(j));

		return vDesc;
	}

	g2o::SE3Quat cConverter::toSE3Quat(const cv::Matx44d& homCV)
	{
		Eigen::Matrix<double, 3, 3> R;
		R << homCV(0, 0), homCV(0, 1), homCV(0, 2),
			homCV(1, 0), homCV(1, 1), homCV(1, 2),
			homCV(2, 0), homCV(2, 1), homCV(2, 2);

		Eigen::Matrix<double, 3, 1> t(homCV(0, 3), homCV(1, 3), homCV(2, 3));

		return g2o::SE3Quat(R, t);
	}

	cv::Matx44d cConverter::toCvMat(const g2o::SE3Quat& SE3)
	{
		Eigen::Matrix<double, 4, 4> eigMat = SE3.to_homogeneous_matrix();
		return toCvMat(eigMat);
	}

	cv::Matx44d cConverter::toCvMat(const g2o::Sim3& Sim3)
	{
		Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
		Eigen::Vector3d eigt = Sim3.translation();
		double s = Sim3.scale();
		return toCvSE3(s*eigR, eigt);
	}

	cv::Matx44d cConverter::toCvMat(const Eigen::Matrix<double, 4, 4>& m)
	{
		cv::Matx44d cvMat = cv::Matx44d::eye();
		cv::eigen2cv(m, cvMat);
		return cvMat;
	}

	cv::Matx33d cConverter::toCvMat(const Eigen::Matrix3d& m)
	{
		cv::Matx33d cvMat = cv::Matx33d::eye();
		cv::eigen2cv(m, cvMat);
		return cvMat;
	}


	cv::Matx44d cConverter::toCvSE3(const Eigen::Matrix<double, 3, 3>& R,
		const Eigen::Matrix<double, 3, 1> &t)
	{
		cv::Matx44d cvMat = cv::Matx44d::eye();
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				cvMat(i, j) = R(i, j);

		for (int i = 0; i < 3; ++i)
			cvMat(i, 3) = t(i);

		return cvMat;
	}


	Eigen::Matrix<double, 3, 1> cConverter::toVector3d(const cv::Vec4d& cvVector)
	{
		Eigen::Matrix<double, 3, 1> v;
		v << cvVector(0), cvVector(1), cvVector(2);
		return v;
	}

	Eigen::Matrix<double, 3, 1> cConverter::toVector3d(const cv::Vec3d& cvVector)
	{
		Eigen::Matrix<double, 3, 1> v;
		v << cvVector(0), cvVector(1), cvVector(2);

		return v;
	}

	Eigen::Matrix<double, 3, 3> cConverter::toMatrix3d(const cv::Matx33d& cvMat3)
	{
		Eigen::Matrix<double, 3, 3> M;

		M << cvMat3(0, 0), cvMat3(0, 1), cvMat3(0, 2),
			cvMat3(1, 0), cvMat3(1, 1), cvMat3(1, 2),
			cvMat3(2, 0), cvMat3(2, 1), cvMat3(2, 2);

		return M;
	}

	std::vector<double> cConverter::toQuaternion(const cv::Matx33d& M)
	{
		Eigen::Matrix<double, 3, 3> eigMat = toMatrix3d(M);
		Eigen::Quaterniond q(eigMat);

		std::vector<double> v(4);
		v[0] = q.x();
		v[1] = q.y();
		v[2] = q.z();
		v[3] = q.w();

		return v;
	}

	cv::Mat cConverter::toMat(const cv::Matx44d& matx44d)
	{
		cv::Mat out = cv::Mat::zeros(4, 4, CV_64FC1);
		for (int c = 0; c < 4; ++c)
			for (int r = 0; r < 4; ++r)
				out.ptr<double>(r)[c] = matx44d(r, c);
		return out;
	}
}
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
*/

#ifndef CONVERTER_H
#define CONVERTER_H

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <g2o/types/types_six_dof_expmap.h>
#include <g2o/types/types_seven_dof_expmap.h>
#include <g2o/types/se3quat.h>

namespace MultiColSLAM
{

	class cConverter
	{
	public:
		static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);
		static std::vector<cv::Mat> toDescriptorVector(const std::vector<cv::Mat>& Descriptors);

		static g2o::SE3Quat toSE3Quat(const cv::Matx44d& homCV);

		static cv::Matx44d toCvMat(const g2o::SE3Quat& SE3);
		static cv::Matx44d toCvMat(const g2o::Sim3& Sim3);
		static cv::Matx44d toCvMat(const Eigen::Matrix<double, 4, 4>& m);
		static cv::Matx33d toCvMat(const Eigen::Matrix3d& m);
		static cv::Vec3d toCvVec3d(const Eigen::Matrix<double, 3, 1>& m) {
			return cv::Vec3d(m(0), m(1), m(2)); }
		static cv::Matx44d toCvSE3(const Eigen::Matrix<double, 3, 3>& R,
			const Eigen::Matrix<double, 3, 1>& t);
		static cv::Matx<double, 4, 4> ogv2ocv(const Eigen::Matrix<double, 3, 4>& ogv_mat);

		// cv to eigen
		static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Vec4d& cvVector);
		static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Vec3d& cvVector);
		static Eigen::Matrix<double, 2, 1> toVector2d(const cv::Vec2d& cvVector) {
			return Eigen::Matrix<double, 2, 1>(cvVector(0), cvVector(1)); }

		static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Matx33d& cvMat3);

		static std::vector<double> toQuaternion(const cv::Matx33d& M);
		// between mats and vecs
		static cv::Matx33d Hom2R(const cv::Matx44d& homCV) {
			return cv::Matx33d(homCV(0, 0), homCV(0, 1), homCV(0, 2),
				homCV(1, 0), homCV(1, 1), homCV(1, 2),
				homCV(2, 0), homCV(2, 1), homCV(2, 2)); }

		static cv::Vec3d Hom2T(const cv::Matx44d& homCV) {
			return cv::Vec3d(homCV(0, 3), homCV(1, 3), homCV(2, 3)); }

		static cv::Matx44d Rt2Hom(const cv::Matx33d& R, const cv::Vec3d& t){
			return cv::Matx44d(R(0, 0), R(0, 1), R(0, 2), t(0),
				R(1, 0), R(1, 1), R(1, 2), t(1),
				R(2, 0), R(2, 1), R(2, 2), t(2),
				0.0, 0.0, 0.0, 1.0); }

		// between vecs
		static cv::Vec4d toVec4d(const cv::Vec3d& in) {
			return cv::Vec4d(in(0), in(1), in(2), 1.0); }
		static cv::Vec4d toVec4d(const Eigen::Matrix<double, 3, 1>& in){
			return cv::Vec4d(in(0), in(1), in(2), 1.0); }

		// between matx and mat
		static cv::Mat toMat(const cv::Matx44d& matx44d);

		// hom
		static cv::Matx44d invMat(const cv::Matx44d& M);

	};
}
#endif // cCONVERTER_H

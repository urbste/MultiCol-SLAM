/**
* This file is part of MultiCol-SLAM
* It is based on the file orb.cpp from the OpenCV library (see BSD license below).
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
#include "misc.h"

namespace MultiColSLAM
{
	cv::Vec3d triangulate_point(
		const cv::Matx31d& relOri_t12,
		const cv::Matx33d& relOri_R12,
		const cv::Vec3d& v1,
		const cv::Vec3d& v2)
	{
		// this one is adapted to opencv from opengv::triangulate2
		cv::Vec3d pt3(0, 0, 0);
		cv::Vec3d f2_unrotated = relOri_R12 * v2;
		cv::Vec2d b;
		b[0] = relOri_t12.dot(v1);
		b[1] = relOri_t12.dot(f2_unrotated);
		cv::Matx22d A;
		A(0, 0) = v1.dot(v1);
		A(1, 0) = v1.dot(f2_unrotated);
		A(0, 1) = -A(1, 0);
		A(1, 1) = -f2_unrotated.dot(f2_unrotated);
		cv::Vec2d lambda = A.inv() * b;
		cv::Matx31d xm = lambda[0] * v1;
		cv::Matx31d xn = relOri_t12 + lambda[1] * f2_unrotated;
		cv::Matx31d tmpres = (xm + xn);
		pt3(0) = tmpres(0, 0) / 2.0;
		pt3(1) = tmpres(1, 0) / 2.0;
		pt3(2) = tmpres(2, 0) / 2.0;
		return pt3;
	}


	bool CheckDistEpipolarLine(const cv::Vec3d &ray1,
		const cv::Vec3d &ray2,
		const cv::Matx33d &E12,
		const double& thresh)
	{
		cv::Vec<double, 1> nom = ray2.t()*E12*ray1;
		cv::Vec3d Ex1 = E12*ray1;
		cv::Vec3d Etx2 = E12.t()*ray2;

		const double den = Ex1(0)*Ex1(0) + Ex1(1)*Ex1(1) + Ex1(2)*Ex1(2) +
			Etx2(0)*Etx2(0) + Etx2(1)*Etx2(1) + Etx2(2)*Etx2(2);

		if (den == 0.0)
			return false;
		const double dsqr = (nom(0)*nom(0)) / den;
		return dsqr < thresh;
	}

	cv::Matx33d ComputeE(const cv::Matx44d& T1, const cv::Matx44d& T2)
	{
		cv::Matx33d R1w = T1.get_minor<3, 3>(0, 0);
		cv::Matx33d R2w = T2.get_minor<3, 3>(0, 0);

		cv::Vec3d t1w = cv::Vec3d(T1(0, 3), T1(1, 3), T1(2, 3));
		cv::Vec3d t2w = cv::Vec3d(T2(0, 3), T2(1, 3), T2(2, 3));

		cv::Matx33d R12 = R1w*R2w.t();
		cv::Vec3d t12 = -R1w*R2w.t()*t2w + t1w;
		t12 /= cv::norm(t12);
		cv::Matx33d t12x = Skew(t12);

		return t12x*R12;
	}

	double T_in_ms(HResClk::time_point start,
		HResClk::time_point end)
	{
		return static_cast<double>(
			std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
	}

	double T_in_ns(HResClk::time_point start,
		HResClk::time_point end)
	{
		return static_cast<double>(
			std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
	}

}
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
#ifndef MISC_H
#define MISC_H

#include <opencv2/opencv.hpp>
#include <chrono>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>

namespace MultiColSLAM
{
	// some misc defines
#ifndef M_PID
#define M_PID   3.1415926535897932384626433832795028841971693993
#endif

#ifndef M_PIf
#define M_PIf    3.1415926535897932384626f
#endif

	const double RHOd = 180.0 / M_PID;
	const float  RHOf = 180.0f / M_PIf;

	typedef std::chrono::high_resolution_clock HResClk;

	const float pi2 = M_PIf;
	const float pi32 = 2.0 * M_PIf;
	const float deg2radf = M_PIf / 180.0f;
	const float rad2degf = 180.0f / M_PIf;
	const float pihalf = M_PIf / 2.0f;

	cv::Vec3d triangulate_point(
		const cv::Matx31d& relOri_t12,
		const cv::Matx33d& relOri_R12,
		const cv::Vec3d& v1,
		const cv::Vec3d& v2);


	inline cv::Matx33d Skew(const cv::Vec3d& v)
	{
		return cv::Matx33d(
			0.0, -v(2), v(1),
			v(2), 0.0, -v(0),
			-v(1), v(0), 0.0);
	}

	/**
	* Returns time in milliseconds in double
	*
	* @param start start time
	* @param end end time
	*
	* @return     time in milliseconds
	*/
	double T_in_ms(HResClk::time_point start,
		HResClk::time_point end);
	/**
	* Returns time in nanoseconds in double
	*
	* @param start start time
	* @param end end time
	*
	* @return     time in nanoseconds
	*/
	double T_in_ns(HResClk::time_point start,
		HResClk::time_point end);


	/**
	* median
	*
	* @param data vector of elements
	*
	* @return     median value
	*/
	template<typename T>
	T median(std::vector<T> &v)
	{
		if (v.size() > 0)
		{
			std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
			return v[v.size() / 2];
		}
		else return T(0);
	}

	/**
	* evaluate a polynom using Horner
	*
	* @param coeffs  T* of coefficients
	* @param s		 number of polynomial coefficients
	* @param x		 x value to evaluate
	*
	* @return      function value
	*/
	inline double horner(
		const double* coeffs, const int& s, const double& x)
	{
		double res = 0.0;
		for (int i = s - 1; i >= 0; i--)
			res = res * x + coeffs[i];
		return res;
	}


	/**
	* Cayley representation to 3x3 rotation matrix
	*
	* @param cayParamIn  Cayley parameter
	*
	* @return            rotation matrix
	*/
	template<typename T>
	cv::Matx<T, 3, 3> cayley2rot(const cv::Matx<T, 3, 1>& cayParamIn)
	{
		cv::Matx<T, 3, 3>  R = cv::Matx<T, 3, 3>::eye();

		T c1 = cayParamIn(0, 0);
		T c2 = cayParamIn(1, 0);
		T c3 = cayParamIn(2, 0);

		T c1sqr = c1*c1;
		T c2sqr = c2*c2;
		T c3sqr = c3*c3;

		T scale = T(1) + c1sqr + c2sqr + c3sqr;

		R(0, 0) = 1 + c1sqr - c2sqr - c3sqr;
		R(0, 1) = 2 * (c1*c2 - c3);
		R(0, 2) = 2 * (c1*c3 + c2);
		R(1, 0) = 2 * (c1*c2 + c3);
		R(1, 1) = 1 - c1sqr + c2sqr - c3sqr;
		R(1, 2) = 2 * (c2*c3 - c1);
		R(2, 0) = 2 * (c1*c3 - c2);
		R(2, 1) = 2 * (c2*c3 + c1);
		R(2, 2) = 1 - c1sqr - c2sqr + c3sqr;

		R = (1 / scale) * R;

		return R;
	}


	/**
	* 3x3 rotation matrix to Cayley representation
	*
	* @param R	rotation matrix
	*
	* @return   Cayley parameters
	*/

	template<typename T>
	cv::Matx<T, 3, 1> rot2cayley(const cv::Matx<T, 3, 3>& R)
	{
		cv::Matx<T, 3, 3> eyeM = cv::Matx<T, 3, 3>::eye();

		cv::Matx<T, 3, 3> C1 = R - eyeM;
		cv::Matx<T, 3, 3> C2 = R + eyeM;
		cv::Matx<T, 3, 3> C = C1 * C2.inv();

		cv::Matx<T, 3, 1> cayley(-C(1, 2), C(0, 2), -C(0, 1));

		return cayley;
	}

	/**
	* 4x4 homogeneous transformation matrix to Cayley + translation representation
	*
	* @param T	4x4 homogeneous transformation matrix
	*
	* @return c  6x1 Cayley parameters and translation
	*/
	template<typename T>
	cv::Matx<T, 6, 1> hom2cayley(const cv::Matx<T, 4, 4>& M)
	{
		cv::Matx<T, 3, 3> R(M(0, 0), M(0, 1), M(0, 2),
			M(1, 0), M(1, 1), M(1, 2),
			M(2, 0), M(2, 1), M(2, 2));
		cv::Matx<T, 3, 1> C = rot2cayley(R);

		return cv::Matx<T, 6, 1>(C(0, 0), C(1, 0), C(2, 0),
			M(0, 3), M(1, 3), M(2, 3));
	}

	/**
	* 6x1 minimal homogeneous transformation vector to homogeneous 4x4 transformation matrix
	*
	* @param c	6x1 Cayley parameters and translation
	*
	* @return T 4x4 homogeneous transformation matrix
	*/
	template<typename T>
	cv::Matx<T, 4, 4> cayley2hom(const cv::Matx<T, 6, 1>& cayleyRep)
	{
		cv::Matx<T, 3, 1> cayleyR(cayleyRep(0, 0), cayleyRep(1, 0), cayleyRep(2, 0));
		cv::Matx<T, 3, 3> R = cayley2rot(cayleyR);

		cv::Matx<T, 4, 4> homM(
			R(0, 0), R(0, 1), R(0, 2), cayleyRep(3, 0),
			R(1, 0), R(1, 1), R(1, 2), cayleyRep(4, 0),
			R(2, 0), R(2, 1), R(2, 2), cayleyRep(5, 0),
			T(0), T(0), T(0), T(1));

		return homM;
	}

	bool CheckDistEpipolarLine(const cv::Vec3d &ray1,
		const cv::Vec3d &ray2,
		const cv::Matx33d &E12,
		const double& thresh);

	cv::Matx33d ComputeE(const cv::Matx44d& T1, const cv::Matx44d& T2);

	inline cv::Matx33d ComputeE(const cv::Matx44d& Trel)
	{
		cv::Matx33d R = Trel.get_minor<3, 3>(0, 0);
		cv::Vec3d t = cv::Vec3d(Trel(0, 3), Trel(1, 3), Trel(2, 3));
		t /= cv::norm(t);
		cv::Matx33d t12x = Skew(t);

		return t12x*R;
	}
}
#endif
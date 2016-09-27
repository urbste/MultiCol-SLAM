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
cam_model_omni.h

@brief:
This class implements Scaramuzzas general atan camera model.
Calibrated with: https://github.com/urbste/ImprovedOcamCalib

ToDo:

Author: Steffen Urban
Date: 23.08.2016
*/

#ifndef CAM_MODEL_GENERAL_H
#define CAM_MODEL_GENERAL_H

// extern includes
#include <opencv2/opencv.hpp>
#include <limits>
#include <Eigen/Dense>


namespace MultiColSLAM
{
	class cCamModelGeneral_
	{
	public:
		// construtors
		cCamModelGeneral_() :
			c(1),
			d(0),
			e(0),
			u0(0),
			v0(0),
			p((cv::Mat_<double>(1, 1) << 1)),
			invP((cv::Mat_<double>(1, 1) << 1)),
			p_deg(1),
			invP_deg(1),
			Iwidth(0),
			Iheight(0),
			p1(1)
		{}

		cCamModelGeneral_(double cdeu0v0[], cv::Mat_<double> p_, cv::Mat_<double> invP_) :
			c(cdeu0v0[0]),
			d(cdeu0v0[1]),
			e(cdeu0v0[2]),
			u0(cdeu0v0[3]),
			v0(cdeu0v0[4]),
			p(p_),
			invP(invP_)
		{
			// initialize degree of polynomials
			p_deg = (p_.rows > 1) ? p_.rows : p_deg = p_.cols;
			invP_deg = (p_.rows > 1) ? invP_deg = invP_.rows : invP_deg = invP_.cols;

			cde1 = (cv::Mat_<double>(2, 2) << c, d, e, 1.0);
			p1 = p.at<double>(0);
			invAffine = c - d*e;
		}

		cCamModelGeneral_(double cdeu0v0[],
			cv::Mat_<double> p_, cv::Mat_<double> invP_,
			double Iw_, double Ih_) :
			c(cdeu0v0[0]),
			d(cdeu0v0[1]),
			e(cdeu0v0[2]),
			u0(cdeu0v0[3]),
			v0(cdeu0v0[4]),
			p(p_),
			invP(invP_),
			Iwidth(Iw_),
			Iheight(Ih_)
		{
			// initialize degree of polynomials
			p_deg = (p_.rows > 1) ? p_.rows : p_deg = p_.cols;
			invP_deg = (p_.rows > 1) ? invP_deg = invP_.rows : invP_deg = invP_.cols;

			cde1 = (cv::Mat_<double>(2, 2) << c, d, e, 1.0);
			p1 = p.at<double>(0);
			invAffine = c - d*e;
		}

		~cCamModelGeneral_(){}

		void WorldToImg(const double& x, const double& y, const double& z,    // 3D scene point
			double& u, double& v) const;

		void WorldToImg(const cv::Point3_<double>& X,			// 3D scene point
			cv::Point_<double>& m);

		void WorldToImg(const cv::Vec3d& X,			// 3D scene point
			cv::Vec2d& m);

		void WorldToImg(const cv::Vec3d& X,			// 3D scene point
			cv::Vec2f& m);

		void ImgToWorld(double& x, double& y, double& z,						// 3D scene point
			const double& u, const double& v);

		void ImgToWorld(cv::Point3_<double>& X,						// 3D scene point
			const cv::Point_<double>& m);

		void ImgToWorld(cv::Vec3d& X,						// 3D scene point
			const cv::Vec2d& m);

		void undistortPointsOcam(
			const double& ptx, const double& pty,
			const double& undistScaleFactor,
			double& out_ptx, double& out_pty)
		{
			double x = 0.0;
			double y = 0.0;
			double z = 0.0;
			this->ImgToWorld(x, y, z, ptx, pty);
			out_ptx = -x / z * undistScaleFactor;
			out_pty = -y / z * undistScaleFactor;
		}

		void distortPointsOcam(
			const double& ptx, const double& pty,
			double& dist_ptx, double& dist_pty)
		{
			WorldToImg(ptx, pty, -p1, dist_ptx, dist_pty);
		}

		// get functions
		double Get_c() { return c; }
		double Get_d() { return d; }
		double Get_e() { return e; }

		double Get_u0() { return u0; }
		double Get_v0() { return v0; }

		int GetInvDeg() { return invP_deg; }
		int GetPolDeg() { return p_deg; }

		cv::Mat_<double> Get_invP() { return invP; }
		cv::Mat_<double> Get_P() { return p; }

		double GetWidth() { return Iwidth; }
		double GetHeight() { return Iheight; }

		cv::Mat GetMirrorMask(int pyrL) { return mirrorMasks[pyrL]; }
		void SetMirrorMasks(std::vector<cv::Mat> mirrorMasks_) { mirrorMasks = mirrorMasks_; }

		bool isPointInMirrorMask(const double& u, const double& v, int pyr);


		inline double operator [](int i) const
		{
			assert(i < (12 + 5));
			if (i > 4)
				return invP.at<double>(i - 5, 0);
			if (i == 0)
				return c;
			if (i == 1)
				return d;
			if (i == 2)
				return e;
			if (i == 3)
				return u0;
			if (i == 4)
				return v0;
		}

		// ATTENTION, used fixed size here!!
		// this will not work in general!!
		inline Eigen::Matrix<double, 12 + 5, 1> toVector() const
		{
			Eigen::Matrix<double, 12 + 5, 1> tmp =
				Eigen::Matrix<double, 12 + 5, 1>::Zero();
			tmp(0, 0) = c;
			tmp(1, 0) = d;
			tmp(2, 0) = e;
			tmp(3, 0) = u0;
			tmp(4, 0) = v0;
			for (int i = 0; i < invP.rows; ++i)
				tmp(5 + i, 0) = invP.at<double>(i, 0);

			return tmp;
		}

		inline void fromVector(const Eigen::Matrix<double, 12 + 5, 1> tmp)
		{
			c = tmp(0, 0);
			d = tmp(1, 0);
			e = tmp(2, 0);
			u0 = tmp(3, 0);
			v0 = tmp(4, 0);
			for (int i = 0; i < invP.rows; ++i)
				invP.at<double>(i, 0) = tmp(5 + i, 0);
		}

		inline cCamModelGeneral_ operator+ (const Eigen::Matrix<double, 12 + 5, 1>& values2add) const
		{
			cCamModelGeneral_ result(*this);
			Eigen::Matrix<double, 12 + 5, 1> valuesOld = result.toVector();
			result.fromVector(valuesOld + values2add);
			return result;
		}

	protected:

		// affin
		double c;
		double d;
		double e;
		double invAffine;
		cv::Mat_<double> cde1;
		// principal
		double u0;
		double v0;
		// polynomial
		double p1;
		cv::Mat_<double> p;
		int p_deg;
		// inverse polynomial
		cv::Mat_<double> invP;

		int invP_deg;
		// image width and height
		double Iwidth;
		double Iheight;
		// mirror mask on pyramid levels
		std::vector<cv::Mat> mirrorMasks;
	};


	void CreateMirrorMask(cCamModelGeneral_ camera,
		int pyrLevel,
		std::vector<cv::Mat>& mirror_masks);

}
#endif
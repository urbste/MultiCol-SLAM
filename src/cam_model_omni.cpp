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
#include "cam_model_omni.h"
// own includes
#include "misc.h"

namespace MultiColSLAM
{
	using namespace cv;
	using namespace std;

	void cCamModelGeneral_::ImgToWorld(cv::Point3_<double>& X,						// 3D scene point
		const cv::Point_<double>& m) 			            // 2D image point
	{
		//double invAff = c - d*e;
		const double u_t = m.x - u0;
		const double v_t = m.y - v0;
		// inverse affine matrix image to sensor plane conversion
		X.x = (1 * u_t - d * v_t) / this->invAffine;
		X.y = (-e * u_t + c * v_t) / this->invAffine;
		const double X2 = X.x * X.x;
		const double Y2 = X.y * X.y;
		X.z = -horner((double*)p.data, p_deg, sqrt(X2 + Y2));

		// normalize vectors spherically
		const double norm = sqrt(X2 + Y2 + X.z*X.z);
		X.x /= norm;
		X.y /= norm;
		X.z /= norm;
	}

	void cCamModelGeneral_::ImgToWorld(double& x, double& y, double& z,						// 3D scene point
		const double& u, const double& v) 			    // 2D image point
	{
		//double invAff = c - d*e;
		const double u_t = u - u0;
		const double v_t = v - v0;
		// inverse affine matrix image to sensor plane conversion
		x = (u_t - d * v_t) / this->invAffine;
		y = (-e * u_t + c * v_t) / this->invAffine;
		const double X2 = x * x;
		const double Y2 = y * y;
		z = -horner((double*)p.data, p_deg, sqrt(X2 + Y2));

		// normalize vectors spherically
		double norm = sqrt(X2 + Y2 + z*z);
		x /= norm;
		y /= norm;
		z /= norm;
	}

	void cCamModelGeneral_::ImgToWorld(cv::Vec3d& X,						// 3D scene point
		const cv::Vec2d& m) 			            // 2D image point
	{
		//double invAff = c - d*e;
		const double u_t = m(0) - u0;
		const double v_t = m(1) - v0;
		// inverse affine matrix image to sensor plane conversion
		X(0) = (u_t - d * v_t) / this->invAffine;
		X(1) = (-e * u_t + c * v_t) / this->invAffine;
		const double X2 = X(0) * X(0);
		const double Y2 = X(1) * X(1);
		X(2) = -horner((double*)p.data, p_deg, sqrt(X2 + Y2));

		// normalize vectors spherically
		double norm = sqrt(X2 + Y2 + X(2)*X(2));
		X(0) /= norm;
		X(1) /= norm;
		X(2) /= norm;
	}


	void cCamModelGeneral_::WorldToImg(const cv::Point3_<double>& X,			// 3D scene point
		cv::Point_<double>& m)			// 2D image point
	{
		double norm = sqrt(X.x*X.x + X.y*X.y);

		if (norm == 0.0)
			norm = 1e-14;

		const double theta = atan(-X.z / norm);
		const double rho = horner((double*)invP.data, invP_deg, theta);

		const double uu = X.x / norm * rho;
		const double vv = X.y / norm * rho;

		m.x = uu*c + vv*d + u0;
		m.y = uu*e + vv + v0;
	}

	void cCamModelGeneral_::WorldToImg(const cv::Vec3d& X,			// 3D scene point
		cv::Vec2d& m)			// 2D image point
	{

		double norm = cv::sqrt(X(0)*X(0) + X(1)*X(1));

		if (norm == 0.0)
			norm = 1e-14;

		const double theta = atan(-X(2) / norm);
		const double rho = horner((double*)invP.data, invP_deg, theta);

		const double uu = X(0) / norm * rho;
		const double vv = X(1) / norm * rho;

		m(0) = uu*c + vv*d + u0;
		m(1) = uu*e + vv + v0;
	}

	void cCamModelGeneral_::WorldToImg(const cv::Vec3d& X,			// 3D scene point
		cv::Vec2f& m)			// 2D image point
	{
		double norm = cv::sqrt(X(0)*X(0) + X(1)*X(1));

		if (norm == 0.0)
			norm = 1e-14;

		const double theta = atan(-X(2) / norm);

		const double rho = horner((double*)invP.data, invP_deg, theta);

		const double uu = X(0) / norm * rho;
		const double vv = X(1) / norm * rho;

		m(0) = uu*c + vv*d + u0;
		m(1) = uu*e + vv + v0;
	}

	void cCamModelGeneral_::WorldToImg(const double& x, const double& y, const double& z,    // 3D scene point
		double& u, double& v) const							 // 2D image point
	{
		double norm = sqrt(x*x + y*y);
		if (norm == 0.0)
			norm = 1e-14;

		const double theta = atan(-z / norm);
		const double rho = horner((double*)invP.data, invP_deg, theta);

		const double uu = x / norm * rho;
		const double vv = y / norm * rho;

		u = uu*c + vv*d + u0;
		v = uu*e + vv + v0;
	}

	bool cCamModelGeneral_::isPointInMirrorMask(
		const double& u,
		const double& v,
		int pyr)
	{
		const int ur = cvRound(u);
		const int vr = cvRound(v);
		// check image bounds
		if (ur >= mirrorMasks[pyr].cols || ur <= 0 ||
			vr >= mirrorMasks[pyr].rows || vr <= 0)
			return false;
		// check mirror
		if (mirrorMasks[pyr].ptr<uchar>(vr)[ur] > 0)
			return true;
		else return false;
	}


	void CreateMirrorMask(cCamModelGeneral_ camera,
		int pyrLevel,
		vector<Mat>& mirror_masks)
	{
		int w = (int)camera.GetWidth();
		int h = (int)camera.GetHeight();
		float u0 = (float)camera.Get_v0();
		float v0 = (float)camera.Get_u0();
		Mat sizeDef = Mat::zeros(h, w, CV_8UC1);
		vector<Mat> sizeDefvec;
		buildPyramid(sizeDef, sizeDefvec, pyrLevel);
		// Mirror mask for pyramid
		float offset[4] = { 22.0f, 10.0f, 5.0f, 1.0f };
		//float offset[4] = { 50.0f, 50.0f, 50.0f, 50.0f};
		for (int mIdx = 0; mIdx < pyrLevel; mIdx++)
		{
			if (mIdx != 0)
			{
				w = sizeDefvec[mIdx].cols;
				h = sizeDefvec[mIdx].rows;

				u0 = ceil(u0 / 2.0f);
				v0 = ceil(v0 / 2.0f);
			}
			Mat tempMask = Mat::zeros(h, w, CV_8UC1);
			for (int i = 0; i < h; ++i)
			{
				for (int j = 0; j < w; ++j)
				{
					float ans = sqrt((float)pow(i - u0, 2) + (float)pow(j - v0, 2));
					if (ans < (u0 + offset[mIdx]))
						tempMask.at<uchar>(i, j) = 255;
					else
						tempMask.at<uchar>(i, j) = 0;
				}

			}
			mirror_masks.push_back(tempMask);
		}
	}

}
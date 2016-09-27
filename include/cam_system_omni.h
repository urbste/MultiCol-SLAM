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
multi_cam_system_omni.h

@brief:
This class represents arbitrary, rigidly coupled multi-(fisheye) camera systems
It holds the calibration matrices M_c and N attached cameras

See also:
MultiCol Bundle Adjustment: 
A Generic Method for Pose Estimation, Simultaneous Self-Calibration 
and Reconstruction for Arbitrary Multi-Camera Systems
http://link.springer.com/article/10.1007/s11263-016-0935-0

ToDo:
Author: Steffen Urban
Date:   23.08.2016
*/

#ifndef MULTI_CAMERA_SYSTEM_H
#define MULTI_CAMERA_SYSTEM_H

// external includes
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opengv/types.hpp>

// own includes
#include "misc.h"
#include "cam_model_omni.h"
#include "cConverter.h"
namespace MultiColSLAM
{
	class cMultiCamSys_
	{
	public:
		cMultiCamSys_() : nrCams(0), flagMcMt(false),
			M_t(cv::Matx44d::eye()), M_t_min(cv::Matx61d::zeros()){}

		// constructor initalizes MCS pose and MCS calibration
		cMultiCamSys_(cv::Matx<double, 4, 4> M_t_,
			std::vector<cv::Matx<double, 4, 4>> M_c_,
			std::vector<cCamModelGeneral_> camModels_) :
			M_t(M_t_), M_c(M_c_), camModels(camModels_), flagMcMt(true)
		{
			nrCams = (int)M_c_.size();
			M_t_min = hom2cayley<double>(M_t);
			//M_t_min = hom2rodrigues<double>(M_t);
			M_t_inv = cConverter::invMat(M_t);

			MtMc = std::vector<cv::Matx<double, 4, 4>>(nrCams);
			MtMc_inv = std::vector<cv::Matx<double, 4, 4>>(nrCams);
			for (int c = 0; c < nrCams; ++c)
			{
				MtMc[c] = M_t_ * M_c_[c];
				MtMc_inv[c] = cConverter::invMat(MtMc[c]);
				M_c_min.push_back(hom2cayley<double>(M_c[c]));
				//M_c_min.push_back(hom2rodrigues<double>(M_c[c]));
				// opengv conversion
				opengv::rotation_t R;
				opengv::translation_t t;
				cv::Mat Rcv(M_c[c].get_minor<3, 3>(0, 0));
				cv::cv2eigen(Rcv, R);
				cv::Mat_<double> tcv =
					(cv::Mat_<double>(3, 1) << M_c[c](0, 3), M_c[c](1, 3), M_c[c](2, 3));
				cv::cv2eigen(tcv, t);
				this->camRotations.push_back(R);
				this->camOffsets.push_back(t);
			}
		}

		void WorldToCam(int c,
			cv::Point3_<double>& pt3,
			cv::Point_<double>& pt2);

		void WorldToCam(int c,
			cv::Point3_<double>& pt3,
			cv::Vec2d& pt2);

		void WorldToCamHom(int c,
			cv::Vec<double, 4>& pt4,
			cv::Vec2d& pt2);

		bool WorldToCamHom_fast(int c,
			cv::Vec<double, 4>& pt4,
			cv::Vec<double, 2>& pt2);

		void WorldToCamHom_fast(int c,
			cv::Vec<double, 3>& pt3,
			cv::Vec<double, 2>& pt2);

		void CamToWorld_ogv(int c,
			opengv::bearingVector_t& bearingV,
			cv::Point_<double> pt2);

		void CamToWorld(int c, cv::Point3_<double>& pt3, cv::Point_<double>& pt2);
		void CamToWorld(int c, cv::Vec<double, 3>& pt3, cv::Point_<double>& pt2);

		// set pose
		void Set_M_t_from_min(cv::Matx<double, 6, 1> M_t_minRep);
		void Set_M_t(cv::Matx<double, 4, 4> M_t_);

		// set calibration
		void Set_M_c_from_min(int c, cv::Matx<double, 6, 1> M_c_minRep);
		void Set_M_c(int c, cv::Matx<double, 4, 4> M_c_);
		void Set_All_M_c(std::vector<cv::Matx<double, 4, 4>> M_c_);
		void Set_All_M_c_from_min(std::vector<cv::Matx<double, 6, 1>> M_c_min_);


		// add MCS camera pose
		// and set the last one to the actual pose
		void Add_M_c(cv::Matx<double, 4, 4> M_c_);
		void Add_M_c_from_min(cv::Matx<double, 6, 1> M_c_min_);

		void Add_M_c_from_min_and_IO(cv::Matx<double, 6, 1> M_c_min_,
			cCamModelGeneral_ camM);

		void Set_M_c_from_min_and_IO(int c, cv::Matx<double, 6, 1> M_c_min_, cCamModelGeneral_ camM);

		void Set_IO(int c, cCamModelGeneral_ camM) { camModels[c] = camM; }
		void Set_IOs(std::vector<cCamModelGeneral_> camModels_) { camModels = camModels_; }

		// get functions
		int GetNrCams() { return static_cast<int>(camModels.size()); }

		cCamModelGeneral_ GetCamModelObj(int c) { return camModels[c]; }
		cv::Matx<double, 6, 1> Get_M_t_min() { return M_t_min; }

		cv::Matx<double, 6, 1> Get_M_c_min(int c) { return M_c_min[c]; }

		cv::Matx<double, 4, 4> Get_M_t() { return M_t; }
		cv::Matx<double, 4, 4> Get_M_c(int c) { return M_c[c]; }

		Eigen::Matrix3d Get_R_c_ogv(int c) { return camRotations[c]; }
		Eigen::Vector3d Get_t_c_ogv(int c) { return camOffsets[c]; }

		opengv::rotations_t Get_All_R_c_ogv() { return camRotations; }
		opengv::translations_t Get_All_t_c_ogv() { return camOffsets; }

		std::vector<cv::Matx<double, 4, 4>> Get_All_M_c() { return M_c; }

		cv::Matx<double, 4, 4> Get_MtMc(int cam)
		{
			if (!flagMcMt)
				return (M_t*M_c[cam]);
			else
				return MtMc[cam];
		}

		cv::Matx<double, 4, 4> Get_MtMc_inv(int cam)
		{
			if (!flagMcMt)
				return cConverter::invMat(M_t*M_c[cam]);
			else
				return MtMc_inv[cam];
		}

	private:
		int nrCams;

		// current camera pose
		cv::Matx<double, 4, 4> M_t;			// MCS pose
		cv::Matx<double, 4, 4> M_t_inv;		// inverse MCS pose
		cv::Matx<double, 6, 1> M_t_min;		// MCS pose cayley

		// opencv
		std::vector<cv::Matx<double, 4, 4>> M_c;		// MCS calibration data
		std::vector<cv::Matx<double, 6, 1>> M_c_min;    // MCS calibration data as cayley rep
		// for opengv
		opengv::rotations_t camRotations;
		opengv::translations_t camOffsets;

		std::vector<cCamModelGeneral_> camModels; // specific camera model

		// transformation (MtMc)^-1 (Mc^-1*Mt^-1) at all time
		std::vector<cv::Matx<double, 4, 4> > MtMc;
		std::vector<cv::Matx<double, 4, 4> > MtMc_inv;

		bool flagMcMt;
	};



}
#endif





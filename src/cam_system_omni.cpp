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
#include "cam_system_omni.h"
#include "cConverter.h"

namespace MultiColSLAM
{
	void cMultiCamSys_::WorldToCam(int c,
		cv::Point3_<double>& pt3,
		cv::Point_<double>& pt2)
	{
		cv::Matx<double, 4, 4> M_ = M_t*M_c[c];

		cv::Matx<double, 4, 1> pt3_(pt3.x, pt3.y, pt3.z, 1.0);

		cv::Matx<double, 4, 1> ptRot = cConverter::invMat(M_)*pt3_;
		pt2.x = 0.0;
		pt2.y = 0.0;

		camModels[c].WorldToImg(
			ptRot(0, 0), ptRot(1, 0), ptRot(2, 0),
			pt2.x, pt2.y);
	}

	void cMultiCamSys_::WorldToCam(int c,
		cv::Point3_<double>& pt3,
		cv::Vec2d& pt2)
	{
		cv::Matx<double, 4, 1> ptRot;
		cv::Matx<double, 4, 1> pt3_(pt3.x, pt3.y, pt3.z, 1.0);
		if (!flagMcMt)
		{
			cv::Matx<double, 4, 4> M_ = M_t * M_c[c];
			ptRot = cConverter::invMat(M_)*pt3_;
		}
		else
			ptRot = MtMc_inv[c] * pt3_;

		pt2 = 0.0;
		pt2 = 0.0;

		cv::Point3_<double> ptRot3;
		ptRot3.x = ptRot(0, 0);
		ptRot3.y = ptRot(1, 0);
		ptRot3.z = ptRot(2, 0);

		camModels[c].WorldToImg(ptRot3.x, ptRot3.y, ptRot3.z, pt2(0), pt2(1));
	}

	void cMultiCamSys_::WorldToCamHom(int c,
		cv::Vec<double, 4>& pt4,
		cv::Vec2d& pt2)
	{
		cv::Matx<double, 4, 1> ptRot;
		if (!flagMcMt)
		{
			cv::Matx<double, 4, 4> M_ = M_t * M_c[c];
			ptRot = cConverter::invMat(M_)*pt4;
		}
		else
			ptRot = MtMc_inv[c] * pt4;

		pt2 = 0.0;
		pt2 = 0.0;

		cv::Point3_<double> ptRot3;
		ptRot3.x = ptRot(0, 0);
		ptRot3.y = ptRot(1, 0);
		ptRot3.z = ptRot(2, 0);

		camModels[c].WorldToImg(ptRot(0, 0), ptRot(1, 0), ptRot(2, 0),
			pt2(0), pt2(1));
	}

	bool cMultiCamSys_::WorldToCamHom_fast(int c,
		cv::Vec<double, 4>& pt4,
		cv::Vec<double, 2>& pt2)
	{
		cv::Matx<double, 4, 1> ptRot;
		if (!flagMcMt)
		{
			cv::Matx<double, 4, 4> M_ = M_t * M_c[c];

			ptRot = cConverter::invMat(M_) * pt4;
		}
		else
			ptRot = MtMc_inv[c] * pt4;

		pt2(0) = 0.0;
		pt2(1) = 0.0;

		camModels[c].WorldToImg(ptRot(0, 0), ptRot(1, 0), ptRot(2, 0),
			pt2(0), pt2(1));
		return ptRot(2, 0) <= 0.0;
	}

	void cMultiCamSys_::WorldToCamHom_fast(int c,
		cv::Vec<double, 3>& pt3,
		cv::Vec<double, 2>& pt2)
	{
		cv::Matx<double, 4, 1> ptRot;
		if (!flagMcMt)
		{
			cv::Matx<double, 4, 4> M_ = M_t * M_c[c];

			ptRot = cConverter::invMat(M_) * cv::Vec4d(pt3(0), pt3(1), pt3(2), 1.0);
		}
		else
			ptRot = MtMc_inv[c] * cv::Vec4d(pt3(0), pt3(1), pt3(2), 1.0);

		pt2(0) = 0.0;
		pt2(1) = 0.0;

		camModels[c].WorldToImg(ptRot(0, 0), ptRot(1, 0), ptRot(2, 0),
			pt2(0), pt2(1));
	}

	void cMultiCamSys_::CamToWorld_ogv(int c,
		opengv::bearingVector_t& bearingV,
		cv::Point_<double> pt2)
	{
		cv::Point3_<double> pt3;
		camModels[c].ImgToWorld(pt3,
			pt2);

		bearingV[0] = pt3.x;
		bearingV[1] = pt3.y;
		bearingV[2] = pt3.z;

	}

	void cMultiCamSys_::CamToWorld(int c, cv::Point3_<double>& pt3, cv::Point_<double>& pt2)
	{
		cv::Point3_<double> pt3t;
		camModels[c].ImgToWorld(pt3t, pt2);

		pt3.x = pt3t.x;
		pt3.y = pt3t.y;
		pt3.z = pt3t.z;
	}

	void cMultiCamSys_::CamToWorld(int c, cv::Vec<double, 3>& pt3, cv::Point_<double>& pt2)
	{
		cv::Point3_<double> pt3t;
		camModels[c].ImgToWorld(pt3t, pt2);

		pt3(0) = pt3t.x;
		pt3(1) = pt3t.y;
		pt3(2) = pt3t.z;
	}

	// set pose
	void cMultiCamSys_::Set_M_t_from_min(cv::Matx<double, 6, 1> M_t_minRep)
	{
		M_t_min = M_t_minRep;
		M_t = cayley2hom<double>(M_t_minRep);
		//M_t = rodrigues2hom<double>(M_t_minRep);
		M_t_inv = cConverter::invMat(M_t);

		for (int c = 0; c < nrCams; ++c)
		{
			MtMc[c] = M_t * this->M_c[c];
			MtMc_inv[c] = cConverter::invMat(MtMc[c]);
		}
		flagMcMt = true;
	}

	void cMultiCamSys_::Set_M_t(cv::Matx<double, 4, 4> M_t_)
	{
		M_t_min = hom2cayley<double>(M_t_);
		//M_t_min = hom2rodrigues<double>(M_t_);
		M_t = M_t_;
		M_t_inv = cConverter::invMat(M_t);

		for (int c = 0; c < nrCams; ++c)
		{
			MtMc[c] = M_t*this->M_c[c];
			MtMc_inv[c] = cConverter::invMat(MtMc[c]);
		}
		flagMcMt = true;
	}

	// set calibration
	void cMultiCamSys_::Set_M_c_from_min(int c, cv::Matx<double, 6, 1> M_c_minRep)
	{
		M_c_min[c] = M_c_minRep;
		M_c[c] = cayley2hom<double>(M_c_minRep);
		//M_c[c] = rodrigues2hom<double>(M_c_minRep);
		cv::Mat rTemp = cv::Mat(M_c[c])(cv::Rect(0, 0, 3, 3));
		cv::Mat tTemp = cv::Mat(M_c[c])(cv::Rect(3, 0, 1, 3));
		// to opengv
		Eigen::Matrix3d rotTemp;
		cv::cv2eigen<double>(rTemp, rotTemp);
		camRotations[c] = rotTemp;

		Eigen::Vector3d transTemp;
		cv::cv2eigen<double>(tTemp, transTemp);
		camOffsets[c] = transTemp;
	}

	void cMultiCamSys_::Set_M_c(int c, cv::Matx<double, 4, 4> M_c_)
	{
		M_c_min[c] = hom2cayley<double>(M_c_);
		//M_c_min[c] = hom2rodrigues<double>(M_c_);
		M_c[c] = M_c_;

		cv::Mat rTemp = cv::Mat(M_c_)(cv::Rect(0, 0, 3, 3));
		cv::Mat tTemp = cv::Mat(M_c_)(cv::Rect(3, 0, 1, 3));
		// to opengv
		Eigen::Matrix3d rotTemp;
		cv::cv2eigen<double>(rTemp, rotTemp);
		camRotations[c] = rotTemp;

		Eigen::Vector3d transTemp;
		cv::cv2eigen<double>(tTemp, transTemp);
		camOffsets[c] = transTemp;
	}
	void cMultiCamSys_::Set_All_M_c(std::vector<cv::Matx<double, 4, 4>> M_c_)
	{
		M_c = M_c_;
		for (int i = 0; i < M_c.size(); ++i)
		{
			M_c_min.push_back(hom2cayley<double>(M_c[i]));
			//M_c_min.push_back(hom2rodrigues<double>(M_c[i]));
			cv::Mat rTemp = cv::Mat(M_c[i])(cv::Rect(0, 0, 3, 3));
			cv::Mat tTemp = cv::Mat(M_c[i])(cv::Rect(3, 0, 1, 3));
			// to opengv
			Eigen::Matrix3d rotTemp;
			cv::cv2eigen<double>(rTemp, rotTemp);
			camRotations.push_back(rotTemp);

			Eigen::Vector3d transTemp;
			cv::cv2eigen<double>(tTemp, transTemp);
			camOffsets.push_back(transTemp);
		}
	}
	void cMultiCamSys_::Set_All_M_c_from_min(std::vector<cv::Matx<double, 6, 1>> M_c_min_)
	{
		M_c_min = M_c_min_;
		for (int i = 0; i < M_c_min.size(); ++i)
		{
			cv::Matx<double, 4, 4> c2h = cayley2hom<double>(M_c_min[i]);
			//cv::Matx<double, 4, 4> c2h = rodrigues2hom<double>(M_c_min[i]);
			M_c.push_back(c2h);
			cv::Mat rTemp = cv::Mat(c2h)(cv::Rect(0, 0, 3, 3));
			cv::Mat tTemp = cv::Mat(c2h)(cv::Rect(3, 0, 1, 3));

			// to opengv
			Eigen::Matrix3d rotTemp;
			cv::cv2eigen<double>(rTemp, rotTemp);
			camRotations.push_back(rotTemp);

			Eigen::Vector3d transTemp;
			cv::cv2eigen<double>(tTemp, transTemp);
			camOffsets.push_back(transTemp);
		}
	}

	// add MCS camera pose
	// and set the last one to the actual pose
	void cMultiCamSys_::Add_M_c(cv::Matx<double, 4, 4> M_c_)
	{
		M_c.push_back(M_c_);
		M_c_min.push_back(hom2cayley<double>(M_c_));
		//M_c_min.push_back(hom2rodrigues<double>(M_c_));
		cv::Mat rTemp = cv::Mat(M_c_)(cv::Rect(0, 0, 3, 3));
		cv::Mat tTemp = cv::Mat(M_c_)(cv::Rect(3, 0, 1, 3));

		// to opengv
		Eigen::Matrix3d rotTemp;
		cv::cv2eigen<double>(rTemp, rotTemp);
		camRotations.push_back(rotTemp);

		Eigen::Vector3d transTemp;
		cv::cv2eigen<double>(tTemp, transTemp);
		camOffsets.push_back(transTemp);

		nrCams++;
	}
	void cMultiCamSys_::Add_M_c_from_min(cv::Matx<double, 6, 1> M_c_min_)
	{
		M_c_min.push_back(M_c_min_);
		cv::Matx<double, 4, 4> c2h = cayley2hom(M_c_min_);
		//cv::Matx<double, 4, 4> c2h = rodrigues2hom(M_c_min_);
		M_c.push_back(c2h);

		cv::Mat rTemp = cv::Mat(c2h)(cv::Rect(0, 0, 3, 3));
		cv::Mat tTemp = cv::Mat(c2h)(cv::Rect(3, 0, 1, 3));

		// to opengv
		Eigen::Matrix3d rotTemp;
		cv::cv2eigen<double>(rTemp, rotTemp);
		camRotations.push_back(rotTemp);

		Eigen::Vector3d transTemp;
		cv::cv2eigen<double>(tTemp, transTemp);
		camOffsets.push_back(transTemp);

		nrCams++;
	}

	void cMultiCamSys_::Add_M_c_from_min_and_IO(cv::Matx<double, 6, 1> M_c_min_,
		cCamModelGeneral_ camM)
	{
		Add_M_c_from_min(M_c_min_);

		camModels.push_back(camM);

		//nrCams++; 
	}

	void cMultiCamSys_::Set_M_c_from_min_and_IO(int c, cv::Matx<double, 6, 1> M_c_min_, cCamModelGeneral_ camM)
	{
		M_c_min[c] = M_c_min_;
		M_c[c] = cayley2hom<double>(M_c_min_);
		//M_c[c] = rodrigues2hom<double>(M_c_min_);
		camModels[c] = camM;

		cv::Mat rTemp = cv::Mat(M_c[c])(cv::Rect(0, 0, 3, 3));
		cv::Mat tTemp = cv::Mat(M_c[c])(cv::Rect(3, 0, 1, 3));

		// to opengv
		Eigen::Matrix3d rotTemp;
		cv::cv2eigen<double>(rTemp, rotTemp);
		camRotations[c] = rotTemp;

		Eigen::Vector3d transTemp;
		cv::cv2eigen<double>(tTemp, transTemp);
		camOffsets[c] = transTemp;
	}
}
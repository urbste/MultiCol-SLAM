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
#ifndef G2O_MULTICOL_VERTICES_EDGES_H
#define G2O_MULTICOL_VERTICES_EDGES_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "g2o/types/types_seven_dof_expmap.h"
#include "g2o/core/base_multi_edge.h"
#include <Eigen/StdVector>

#include "misc.h"
#include "cam_model_omni.h"

namespace MultiColSLAM
{
	void mcsJacs1(const cv::Vec3d& pt3,
		const cv::Matx61d& M_t,
		const cv::Matx61d& M_c,
		const Eigen::Matrix<double, 12 + 5, 1>& camModelData,
		cv::Matx<double, 2, 32>& jacs);

	class VertexOmniCameraParameters : public g2o::BaseVertex<5 + 12, Eigen::Matrix<double, 5 + 12, 1>>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		VertexOmniCameraParameters() {}

		VertexOmniCameraParameters(cCamModelGeneral_ camModel_)
			: camModel(camModel_)
		{

		}

		virtual void setToOriginImpl()
		{
			Eigen::Matrix<double, 5 + 12, 1> tmpEstimate = camModel.toVector();
			_estimate = tmpEstimate;
		}
		virtual void oplusImpl(const double* update_)
		{
			Eigen::Map<const Eigen::Matrix<double, 5 + 12, 1>> update(update_);
			// update function x + dx, it is overloaded to update the camera parameters
			// see cam_model_omni.h
			setEstimate(estimate() + update);
			camModel.fromVector(_estimate);
		}

		bool read(std::istream& is)
		{
			std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
			return false;
		}

		bool write(std::ostream& os) const
		{
			std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
			return false;
		}
		cCamModelGeneral_ camModel;
	};
	/**
	* \brief
	*/
	class VertexMt_cayley : public g2o::BaseVertex<6, cv::Matx61d>{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			VertexMt_cayley() {}

		virtual void setToOriginImpl()
		{
			_estimate = cv::Matx61d(0, 0, 0, 0, 0, 0);
		}
		virtual void oplusImpl(const double* update_)
		{
			//Eigen::Map<const g2o::Vector6d> update(update_);
			cv::Matx61d update(update_);
			setEstimate(update + estimate()); // update function x + dx
		}

		bool read(std::istream& is)
		{
			std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
			return false;
		}

		bool write(std::ostream& os) const
		{
			std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
			return false;
		}

	};

	/**
	* \brief
	*/
	class VertexMc_cayley : public g2o::BaseVertex<6, cv::Matx61d>{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			VertexMc_cayley() {}

		virtual void setToOriginImpl()
		{
			_estimate = cv::Matx61d(0, 0, 0, 0, 0, 0);
		}
		virtual void oplusImpl(const double* update_)
		{
			cv::Matx61d update(update_);
			setEstimate(update + estimate()); // update function x + dx
		}

		bool read(std::istream& is)
		{
			std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
			return false;
		}

		bool write(std::ostream& os) const
		{
			std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
			return false;
		}

	};

	/**
	* \brief Point vertex, XYZ
	*/
	class VertexPointXYZ : public g2o::BaseVertex<3, cv::Vec3d>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			VertexPointXYZ() {}

		virtual void setToOriginImpl()
		{
			_estimate = cv::Vec3d(0, 0, 0);
		}

		virtual void oplusImpl(const double* update)
		{
			cv::Vec3d v(update);
			_estimate += v;
		}

		virtual bool read(std::istream& /*is*/)
		{
			std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
			return false;
		}

		virtual bool write(std::ostream& /*os*/) const
		{
			std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
			return false;
		}
		void SetID(int id) { ptID = id; }
		int GetID() { return ptID; }
		int ptID; // need this id for the Sim3 optimization
	};


	// edge, that projects a point to a multi camera system
	// vertex 0 : t -> Mt, which is the MCS pose
	// vertex 1 : i -> 3D point
	// vertex 2 : c -> Mc, transformation from MCS to camera
	// vertex 3 : c -> interior orientation
	class EdgeProjectXYZ2MCS : public g2o::BaseMultiEdge<2, g2o::Vector2D>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			EdgeProjectXYZ2MCS()
		{
			information().setIdentity();
			resize(4);
		}

		virtual bool read(std::istream& /*is*/)
		{
			std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
			return false;
		}

		virtual bool write(std::ostream& /*os*/) const
		{
			std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
			return false;
		}

		void computeError();

		void linearizeOplus();
	};



}
#endif
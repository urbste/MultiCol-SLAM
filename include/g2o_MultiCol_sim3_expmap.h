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

#ifndef G2O_MULTICAM_SIM3_EXPMAP_H
#define G2O_MULTICAM_SIM3_EXPMAP_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "g2o/core/robust_kernel_impl.h"
#include "g2o/types/types_seven_dof_expmap.h"

#include <Eigen/StdVector>

#include <unordered_map>

#include "misc.h"
#include "cam_model_omni.h"
#include "cam_system_omni.h"
#include "cConverter.h"
#include "g2o_MultiCol_vertices_edges.h"

namespace MultiColSLAM
{
	/**
	* \brief Sim3 Vertex, (x,y,z,qw,qx,qy,qz)
	* the parameterization for the increments constructed is a 7d vector
	* (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
	*/
	class simpleVertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			simpleVertexSim3Expmap();
		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setToOriginImpl() {
			_estimate = g2o::Sim3();
		}

		virtual void oplusImpl(const double* update_)
		{
			Eigen::Map<g2o::Vector7d> update(const_cast<double*>(update_));

			if (_fix_scale)
				update[6] = 0;

			g2o::Sim3 s(update);
			setEstimate(s*estimate());
		}

		bool _fix_scale;

	protected:
	};

	/**
	* \brief 7D edge between two Vertex7
	*/
	class edgeSim3 : public g2o::BaseBinaryEdge<7, g2o::Sim3,
		simpleVertexSim3Expmap, simpleVertexSim3Expmap>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			edgeSim3();
		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;
		void computeError()
		{
			const simpleVertexSim3Expmap* v1 = static_cast<const simpleVertexSim3Expmap*>(_vertices[0]);
			const simpleVertexSim3Expmap* v2 = static_cast<const simpleVertexSim3Expmap*>(_vertices[1]);

			g2o::Sim3 C(_measurement);
			g2o::Sim3 error_ = C*v1->estimate()*v2->estimate().inverse();
			_error = error_.log();
		}

		virtual double initialEstimatePossible(
			const g2o::OptimizableGraph::VertexSet&, g2o::OptimizableGraph::Vertex*)
		{
			return 1.;
		}
		virtual void initialEstimate(
			const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
		{
			simpleVertexSim3Expmap* v1 = static_cast<simpleVertexSim3Expmap*>(_vertices[0]);
			simpleVertexSim3Expmap* v2 = static_cast<simpleVertexSim3Expmap*>(_vertices[1]);
			if (from.count(v1) > 0)
				v2->setEstimate(measurement()*v1->estimate());
			else
				v1->setEstimate(measurement().inverse()*v2->estimate());
		}
	};

	class VertexSim3Expmap_Multi : public g2o::BaseVertex<7, g2o::Sim3>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			VertexSim3Expmap_Multi() {}
		VertexSim3Expmap_Multi(
			std::unordered_map<size_t, int>& kp_to_cam1,
			std::unordered_map<size_t, int>& kp_to_cam2);

		virtual void setToOriginImpl() {
			_estimate = g2o::Sim3();
		}

		virtual void oplusImpl(const double* update_)
		{
			Eigen::Map<g2o::Vector7d> update(const_cast<double*>(update_));

			if (_fix_scale)
				update[6] = 0;

			g2o::Sim3 s(update);
			setEstimate(s*estimate());
		}

		cMultiCamSys_* camSys1;
		cMultiCamSys_* camSys2;


		cv::Vec2d cam_map1(const Eigen::Vector3d& v, int ptIdx) const
		{
			cv::Vec2d res;
			int camIdx = keypoint_to_cam1.find(ptIdx)->second;
			cv::Vec4d v_in_cam = cConverter::invMat(camSys1->Get_M_c(camIdx))*cConverter::toVec4d(v);
			// not world2to cam but only projection
			camSys1->GetCamModelObj(camIdx).WorldToImg(
				v_in_cam(0), v_in_cam(1), v_in_cam(2), res(0), res(1));
			return res;
		}

		cv::Vec2d cam_map2(const Eigen::Vector3d& v, int ptIdx) const
		{
			cv::Vec2d res;
			int camIdx = keypoint_to_cam2.find(ptIdx)->second;
			cv::Vec4d v_in_cam = cConverter::invMat(camSys2->Get_M_c(camIdx))*cConverter::toVec4d(v);
			// not world2to cam but only projection
			camSys2->GetCamModelObj(camIdx).WorldToImg(
				v_in_cam(0), v_in_cam(1), v_in_cam(2), res(0), res(1));
			return res;
		}

		bool _fix_scale;

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

		std::unordered_map<size_t, int> keypoint_to_cam1;
		std::unordered_map<size_t, int> keypoint_to_cam2;
	};



	class EdgeSim3ProjectXYZ_Multi :
		public g2o::BaseBinaryEdge<2, g2o::Vector2D, VertexPointXYZ, VertexSim3Expmap_Multi>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			EdgeSim3ProjectXYZ_Multi() {}

		void computeError()
		{
			const VertexSim3Expmap_Multi* v1 = static_cast<const VertexSim3Expmap_Multi*>(_vertices[1]);
			const VertexPointXYZ* v2 = static_cast<const VertexPointXYZ*>(_vertices[0]);

			g2o::Vector2D obs(_measurement);
			cv::Vec3d est = v2->estimate();
			cv::Vec2d m = v1->cam_map1(
				v1->estimate().map(cConverter::toVector3d(est)),
				v2->ptID);

			_error[0] = obs[0] - m(0);
			_error[1] = obs[1] - m(1);
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
	/**/
	class EdgeInverseSim3ProjectXYZ_Multi :
		public g2o::BaseBinaryEdge<2, g2o::Vector2D, VertexPointXYZ, VertexSim3Expmap_Multi>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			EdgeInverseSim3ProjectXYZ_Multi() {}

		void computeError()
		{
			const VertexSim3Expmap_Multi* v1 = static_cast<const VertexSim3Expmap_Multi*>(_vertices[1]);
			const VertexPointXYZ* v2 = static_cast<const VertexPointXYZ*>(_vertices[0]);

			g2o::Vector2D obs(_measurement);
			cv::Vec3d est = v2->estimate();
			cv::Vec2d m = v1->cam_map2(
				v1->estimate().inverse().map(cConverter::toVector3d(est)),
				v2->ptID);

			_error[0] = obs[0] - m(0);
			_error[1] = obs[1] - m(1);
			//cout << "errorInv: " << _error << endl;
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
	* \brief 7D edge between two Vertex7
	*/
	class EdgeSim3_Multi :
		public g2o::BaseBinaryEdge <7, g2o::Sim3, VertexSim3Expmap_Multi, VertexSim3Expmap_Multi>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			EdgeSim3_Multi();

		void computeError()
		{
			const VertexSim3Expmap_Multi* v1 =
				static_cast<const VertexSim3Expmap_Multi*>(_vertices[0]);
			const VertexSim3Expmap_Multi* v2 =
				static_cast<const VertexSim3Expmap_Multi*>(_vertices[1]);

			g2o::Sim3 C(_measurement);
			g2o::Sim3 error_ = C*v1->estimate()*v2->estimate().inverse();
			_error = error_.log();
		}

		virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet&,
			g2o::OptimizableGraph::Vertex*)
		{
			return 1.;
		}

		virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from,
			g2o::OptimizableGraph::Vertex* /*to*/)
		{
			VertexSim3Expmap_Multi* v1 = static_cast<VertexSim3Expmap_Multi*>(_vertices[0]);
			VertexSim3Expmap_Multi* v2 = static_cast<VertexSim3Expmap_Multi*>(_vertices[1]);
			if (from.count(v1) > 0)
				v2->setEstimate(measurement()*v1->estimate());
			else
				v1->setEstimate(measurement().inverse()*v2->estimate());
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

}
#endif
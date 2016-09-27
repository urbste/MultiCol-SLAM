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
#include "g2o_MultiCol_sim3_expmap.h"

namespace MultiColSLAM
{

	VertexSim3Expmap_Multi::VertexSim3Expmap_Multi(
		std::unordered_map<size_t, int>& kp_to_cam1,
		std::unordered_map<size_t, int>& kp_to_cam2)
		: g2o::BaseVertex<7, g2o::Sim3>(),
		keypoint_to_cam1(kp_to_cam1),
		keypoint_to_cam2(kp_to_cam2)
	{
		_marginalized = false;
		_fix_scale = false;
	}

	simpleVertexSim3Expmap::simpleVertexSim3Expmap() : BaseVertex<7, g2o::Sim3>()
	{
		_fix_scale = false;
	}


	edgeSim3::edgeSim3() :
		BaseBinaryEdge<7, g2o::Sim3, simpleVertexSim3Expmap, simpleVertexSim3Expmap>()
	{
	}

	bool simpleVertexSim3Expmap::read(std::istream& is)
	{
		g2o::Vector7d cam2world;
		//for (int i = 0; i<6; i++){
		//	is >> cam2world[i];
		//}
		//is >> cam2world[6];
		////    if (! is) {
		////      // if the scale is not specified we set it to 1;
		////      std::cerr << "!s";
		////      cam2world[6]=0.;
		////    }

		//for (int i = 0; i<2; i++)
		//{
		//	is >> _focal_length1[i];
		//}
		//for (int i = 0; i<2; i++)
		//{
		//	is >> _principle_point1[i];
		//}

		setEstimate(g2o::Sim3(cam2world).inverse());
		return true;
	}

	bool simpleVertexSim3Expmap::write(std::ostream& os) const
	{
		g2o::Sim3 cam2world(estimate().inverse());
		g2o::Vector7d lv = cam2world.log();
		//for (int i = 0; i<7; i++){
		//	os << lv[i] << " ";
		//}
		//for (int i = 0; i<2; i++)
		//{
		//	os << _focal_length1[i] << " ";
		//}
		//for (int i = 0; i<2; i++)
		//{
		//	os << _principle_point1[i] << " ";
		//}
		return os.good();
	}


	bool edgeSim3::read(std::istream& is)
	{
		g2o::Vector7d v7;
		//for (int i = 0; i<7; i++){
		//	is >> v7[i];
		//}

		g2o::Sim3 cam2world(v7);
		setMeasurement(cam2world.inverse());

		//for (int i = 0; i<7; i++)
		//	for (int j = i; j<7; j++)
		//	{
		//		is >> information()(i, j);
		//		if (i != j)
		//			information()(j, i) = information()(i, j);
		//	}
		return true;
	}

	bool edgeSim3::write(std::ostream& os) const
	{
		g2o::Sim3 cam2world(measurement().inverse());
		g2o::Vector7d v7 = cam2world.log();
		//for (int i = 0; i<7; i++)
		//{
		//	os << v7[i] << " ";
		//}
		//for (int i = 0; i<7; i++)
		//	for (int j = i; j<7; j++){
		//		os << " " << information()(i, j);
		//	}
		return os.good();
	}
}
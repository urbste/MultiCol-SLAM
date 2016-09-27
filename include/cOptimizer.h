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
* Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "cMap.h"
#include "cMapPoint.h"
#include "cMultiKeyFrame.h"
#include "cLoopClosing.h"
#include "cMultiFrame.h"

namespace MultiColSLAM
{
	const double huberK = 3;
	const double huberK2 = huberK*huberK;


	//namespace ORB_SLAM
	//{

	class cLoopClosing;

	class cOptimizer
	{
	public:
		void static BundleAdjustment(const std::vector<cMultiKeyFrame*> &vpKF,
			const std::vector<cMapPoint*> &vpMP,
			bool poseOnly,
			int nIterations = 5,
			bool *pbStopFlag = NULL);

		void static GlobalBundleAdjustment(cMap* pMap,
			bool poseOnly = false,
			int nIterations = 5,
			bool *pbStopFlag = NULL);

		static std::list<cMultiKeyFrame*> LocalBundleAdjustment(cMultiKeyFrame* pKF,
			cMap* pMap,
			int nrIters = 10,
			bool getCovMats = false,
			bool *pbStopFlag = NULL);

		int static PoseOptimization(cMultiFrame* pFrame,
			double& inliers,
			const double& huberMultiplier = 2);

		bool static StructureOnly(cMultiKeyFrame* pKF,
			cMapPoint* &vpMP,
			vector<pair<int, cv::Vec2d> >& obs,
			const double& reprErr);

		void static OptimizeEssentialGraph(cMap* pMap,
			cMultiKeyFrame* pLoopMKF, cMultiKeyFrame* pCurMKF,
			g2o::Sim3 &Scurw,
			const cLoopClosing::KeyFrameAndPose &NonCorrectedSim3,
			const cLoopClosing::KeyFrameAndPose &CorrectedSim3,
			const std::map<cMultiKeyFrame*, std::set<cMultiKeyFrame*> > &LoopConnections);

		static int OptimizeSim3(cMultiKeyFrame* pKF1,
			cMultiKeyFrame* pKF2,
			std::vector<cMapPoint *> &vpMatches1,
			g2o::Sim3 &g2oS12);


		static std::list<cMultiKeyFrame*> MotionOnlyBA(cMultiKeyFrame* pKF,
			cMap* pMap);

		static int StructureOnlyBA(cMultiKeyFrame* pKF,
			cMap* pMap);

		static int AdjustMKF2Model(cMultiKeyFrame* pKF, cMap* pMap);

		static int AdjustFrame2Model(cMultiFrame* frame, cMap* pMap);

		static int CalibrateMKSonModel(cMultiKeyFrame* pKF, cMap* pMap);

		static double stdRecon;
		static double stdPose;
		static double stdSim;
		static double stdModel;
	};

}
#endif // OPTIMIZER_H

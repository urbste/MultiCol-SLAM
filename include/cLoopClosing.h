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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "cMultiKeyFrame.h"
#include "cLocalMapping.h"
#include "cMap.h"
#include "cORBVocabulary.h"
#include "cTracking.h"

#include <mutex>
#include "g2o/types/types_seven_dof_expmap.h"
namespace MultiColSLAM
{

	class cTracking;
	class cLocalMapping;
	class cKeyFrameDatabase;

	class cLoopClosing
	{
	public:

		typedef std::pair<std::set<cMultiKeyFrame*>, int> ConsistentGroup;
		typedef std::map<cMultiKeyFrame*, g2o::Sim3, std::less<cMultiKeyFrame*>,
			Eigen::aligned_allocator<std::pair<const cMultiKeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

		cLoopClosing(cMap* pMap, cMultiKeyFrameDatabase* pDB, ORBVocabulary* pVoc);

		void SetTracker(cTracking* pTracker);

		void SetLocalMapper(cLocalMapping* pLocalMapper);

		void Run();

		void InsertKeyFrame(cMultiKeyFrame *pKF);

		void RequestReset();

		void SetMatcherProperties(int _descDim, bool _havingMasks);

		void RequestFinish();
		bool isFinished();
	protected:

		bool CheckNewKeyFrames();

		bool DetectLoop();

		bool ComputeSim3();

		void SearchAndFuse(KeyFrameAndPose &CorrectedPosesMap);

		void CorrectLoop();

		void ResetIfRequested();
		bool mbResetRequested;
		std::mutex mMutexReset;

		cMap* mpMap;
		cTracking* mpTracker;

		cMultiKeyFrameDatabase* mpKeyFrameDB;
		ORBVocabulary* mpORBVocabulary;

		cLocalMapping *mpLocalMapper;

		std::list<cMultiKeyFrame*> mlpLoopKeyFrameQueue;

		std::mutex mMutexLoopQueue;

		std::vector<double> mvfLevelSigmaSquare;

		// Loop detector parameters
		double mnCovisibilityConsistencyTh;

		// Loop detector variables
		cMultiKeyFrame* mpCurrentKF;
		cMultiKeyFrame* mpMatchedKF;
		std::vector<ConsistentGroup> mvConsistentGroups;
		std::vector<cMultiKeyFrame*> mvpEnoughConsistentCandidates;
		std::vector<cMultiKeyFrame*> mvpCurrentConnectedKFs;
		std::vector<cMapPoint*> mvpCurrentMatchedPoints;
		std::vector<cMapPoint*> mvpLoopMapPoints;
		cv::Matx44d mScw;
		g2o::Sim3 mg2oScw;
		double mScale_cw;

		long unsigned int mLastLoopKFid;

		int descDim;
		bool havingMasks;

		bool CheckFinish();
		void SetFinish();
		bool mbFinishRequested;
		bool mbFinished;
		std::mutex mMutexFinish;
	};


}
#endif 

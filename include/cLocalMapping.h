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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "cMultiKeyFrame.h"
#include "cMap.h"
#include "cLoopClosing.h"
#include "cTracking.h"
#include "cMultiKeyFrameDatabase.h"

#include <mutex>

namespace MultiColSLAM
{

	class cTracking;
	class cLoopClosing;
	class cMap;

	class cLocalMapping
	{
	public:
		cLocalMapping(cMap* pMap);

		void SetLoopCloser(cLoopClosing* pLoopCloser);

		void SetTracker(cTracking* pTracker);

		void Run();

		void InsertMultiKeyFrame(cMultiKeyFrame* pKF);

		// Thread Synch
		void RequestStop();
		void RequestReset();

		bool Stop();
		bool SetNotStop(bool flag);

		void Release();

		bool isStopped();

		bool stopRequested();

		void RequestFinish();

		bool isFinished();

		bool AcceptMultiKeyFrames();
		void SetAcceptMultiKeyFrames(bool flag);

		void InterruptBA();

		void SetMatcherProperties(int _descDim, bool _havingMasks);

		std::vector<double> timingMapPointCreate;
		std::vector<double> timingLocalBA;
		std::vector<double> timingMapPointFusion;
		std::vector<double> timingMKFInsertion;

		std::vector<double> timingDEPTHimgs;
		std::vector<double> timingEDGEExtraction;
		std::vector<double> timingEDGEFeatureExtraction;
		std::vector<double> timingRenderAllImgs;
		std::vector<double> timingAdjustModel2MKF;
	protected:

		bool CheckNewMultiKeyFrames();
		void ProcessNewMultiKeyFrame();
		void CreateNewMapPoints();

		void MapPointCulling();
		void SearchInNeighbors();
		void UpdateMapPointStatus();

		void KeyFrameCulling();

		void ResetIfRequested();
		bool CheckFinish();
		void SetFinish();

		bool mbResetRequested;
		std::mutex mMutexReset;

		cMap* mpMap;

		cLoopClosing* mpLoopCloser;
		cTracking* mpTracker;

		std::list<cMultiKeyFrame*> mlNewMultiKeyFrames;

		cMultiKeyFrame* mpCurrentMultiKeyFrame;

		std::list<cMapPoint*> mlpRecentAddedMapPoints;

		std::mutex mMutexNewKFs;

		bool scaleInitialMap;

		bool mbAbortBA;

		bool mbStopped;
		bool mbStopRequested;
		bool mbNotStop;
		std::mutex mMutexStop;

		bool mbAcceptMultiKeyFrames;
		std::mutex mMutexAccept;

		int descDim;
		bool havingMasks;



		bool mbFinishRequested;
		bool mbFinished;
		std::mutex mMutexFinish;
	};

}
#endif // LOCALMAPPING_H

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
#ifndef VIEWER_H
#define VIEWER_H

#include "cMultiFramePublisher.h"
#include "cMapPublisher.h"
#include "cTracking.h"
#include "cSystem.h"

#include <mutex>

namespace MultiColSLAM
{
	class cTracking;
	class cMapPublisher;
	class cSystem;
	class cMultiFramePublisher;

	class cViewer
	{
	public:
		cViewer(cSystem* pSystem,
			cMultiFramePublisher* pFrameDrawer,
			cMapPublisher* pMapDrawer,
			cTracking *pTracking,
			const std::string &strSettingPath);

		// Main thread function. Draw points, keyframes, the current camera pose and the last processed
		// frame. Drawing is refreshed according to the camera fps. We use Pangolin.
		void Run();

		void RequestFinish();

		void RequestStop();

		bool isFinished();

		bool isStopped();

		void Release();

	private:

		bool Stop();

		cSystem* mpSystem;
		cMultiFramePublisher* mpFrameDrawer;
		cMapPublisher* mpMapDrawer;
		cTracking* mpTracker;

		// 1/fps in ms
		double mT;
		float mImageWidth, mImageHeight;
		int drawNrCams;
		float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

		bool CheckFinish();
		void SetFinish();
		bool mbFinishRequested;
		bool mbFinished;
		std::mutex mMutexFinish;

		bool mbStopped;
		bool mbStopRequested;
		std::mutex mMutexStop;

	};
}
#endif // VIEWER_H


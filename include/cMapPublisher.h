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

#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include "cMap.h"
#include "cMapPoint.h"
#include "cMultiKeyFrame.h"
#include "cMultiFramePublisher.h"

#ifndef _DEBUG
#include <pangolin/pangolin.h>
#endif

namespace MultiColSLAM
{
	class cMultiFramePublisher;

	class cMapPublisher
	{
	public:
		cMapPublisher(cMap* pMap,
			const string &strSettingPath);

		cMap* mpMap;

		void PublishMapPoints();
		void PublishMultiKeyFrames(const bool bDrawKF = true,
			const bool bDrawGraph = true);

#ifndef _DEBUG
		void PublishCurrentCamera(
			const pangolin::OpenGlMatrix& Tcw);

		void GetCurrentOpenGLMCSPose(
			pangolin::OpenGlMatrix& _Tcw);
#endif

		void SetCurrentCameraPose(const cv::Matx44d Tcw);
		void SetMCS(const std::vector<cv::Matx44d> M_c_s);

		void SetFinish();
		void RequestFinish();
		bool CheckFinish();
		bool isFinished();
		void RequestStop();
		bool isStopped();
		void Release();
		bool Stop();
	private:

		cv::Matx44d GetCurrentCameraPose();
		bool isCamUpdated();
		void ResetCamFlag();

		float mPointSize;
		float mCameraSize;
		float mCameraLineWidth;
		float mMultiKeyFrameSize;
		float mGraphLineWidth;
		float mMultiKeyFrameLineWidth;

		cv::Matx44d mCameraPose;
		std::vector<cv::Matx44d> mM_c_s;
		bool mbCameraUpdated;
		bool modelPointsSet;
		int cnter;
		int laufSave;

		std::mutex mMutexCamera;
		std::mutex mMutexFinish;
		std::mutex mMutexStop;

		bool mbFinishRequested;
		bool mbFinished;
		bool mbStopped;
		bool mbStopRequested;
	};
}
#endif // MAPPUBLISHER_H

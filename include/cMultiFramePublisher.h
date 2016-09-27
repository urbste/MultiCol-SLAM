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

#ifndef MULTIFRAMEPUBLISHER_H
#define MULTIFRAMEPUBLISHER_H

#include "cTracking.h"
#include "cMapPoint.h"
#include "cMap.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>

namespace MultiColSLAM
{
	class cTracking;
	class cMultiCamSys_;

	class cMultiFramePublisher
	{
	public:
		cMultiFramePublisher(cMap* pMap);

		void Update(cTracking *pTracker);

		void SetMap(cMap* pMap);

		void SetMCS(cMultiCamSys_* camSys);

		void DrawMultiFrame(std::vector<cv::Mat>& imgs);

		bool isMapUpdated();
	protected:
		void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

		std::vector<cv::Mat> mImages;
		std::vector<cv::KeyPoint> mvCurrentKeys;

		std::vector<bool> mvbOutliers;
		std::unordered_map<size_t, int> keyp_to_cam;
		std::vector<cMapPoint*> mvpMatchedMapPoints;
		int mnTracked;
		std::vector<cv::KeyPoint> mvIniKeys;
		std::vector<int> mvIniMatches;

		int nrCams;

		int mState;

		bool mbUpdated;
		bool mcsSet;
		cMap* mpMap;
		int cntWrite;
		std::mutex mMutex;
	};
}
#endif // FRAMEPUBLISHER_H

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

#ifndef MAP_H
#define MAP_H

// own includes
#include "cMapPoint.h"
#include "cMultiKeyFrame.h"
// external includes
#include <set>
#include <mutex>
namespace MultiColSLAM
{

	class cMapPoint;
	class cMultiKeyFrame;

	class cMap
	{
	public:
		cMap();

		void AddKeyFrame(cMultiKeyFrame* pKF);
		void AddMapPoint(cMapPoint* pMP);
		void EraseMapPoint(cMapPoint* pMP);
		void EraseKeyFrame(cMultiKeyFrame* pKF);
		void SetCurrentCameraPose(cv::Mat Tcw);
		void SetReferenceKeyFrames(const std::vector<cMultiKeyFrame*> &vpKFs);
		void SetReferenceMapPoints(const std::vector<cMapPoint*> &vpMPs);

		std::vector<cMultiKeyFrame*> GetAllKeyFrames();
		std::vector<cMapPoint*> GetAllMapPoints();

		std::vector<cMultiKeyFrame*> GetReferenceKeyFrames();
		std::vector<cMapPoint*> GetReferenceMapPoints();

		void GetModelPoints(std::vector<cv::Vec3d>& modelPts,
			std::vector<cv::Vec3d>& modelCompanionPts);
		void SetModelPoints(const std::vector<cv::Vec3d>& _modelPts,
			const std::vector<cv::Vec3d>& _modelCompanionPts);

		int MapPointsInMap();
		int KeyFramesInMap();

		void SetFlagAfterBA();
		bool isMapUpdated();
		void ResetUpdated();

		unsigned int GetMaxKFid();

		void clear();

		//std::mutex mMutexMapUpdate;

	protected:
		std::set<cMapPoint*> mspMapPoints;
		std::set<cMultiKeyFrame*> mspKeyFrames;
		std::vector<cMapPoint*> mvpReferenceMapPoints;

		unsigned int mnMaxKFid;

		std::mutex mMutexMap;
		bool mbMapUpdated;

	};

}
#endif // MAP_H

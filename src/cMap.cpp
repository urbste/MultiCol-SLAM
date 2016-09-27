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

#include "cMap.h"

namespace MultiColSLAM
{
	cMap::cMap()
	{
		mbMapUpdated = false;
		mnMaxKFid = 0;
	}

	void cMap::AddKeyFrame(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mspKeyFrames.insert(pKF);
		if (pKF->mnId > mnMaxKFid)
			mnMaxKFid = pKF->mnId;
		mbMapUpdated = true;
	}

	void cMap::AddMapPoint(cMapPoint *pMP)
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mspMapPoints.insert(pMP);
		mbMapUpdated = true;
	}

	void cMap::EraseMapPoint(cMapPoint *pMP)
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mspMapPoints.erase(pMP);
		mbMapUpdated = true;
	}

	void cMap::EraseKeyFrame(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mspKeyFrames.erase(pKF);
		mbMapUpdated = true;
	}

	void cMap::SetReferenceMapPoints(const std::vector<cMapPoint*> &vpMPs)
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mvpReferenceMapPoints = vpMPs;
		mbMapUpdated = true;
	}

	std::vector<cMultiKeyFrame*> cMap::GetAllKeyFrames()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return std::vector<cMultiKeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
	}

	std::vector<cMapPoint*> cMap::GetAllMapPoints()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return std::vector<cMapPoint*>(mspMapPoints.begin(), mspMapPoints.end());
	}

	int cMap::MapPointsInMap()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return mspMapPoints.size();
	}

	int cMap::KeyFramesInMap()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return mspKeyFrames.size();
	}

	std::vector<cMapPoint*> cMap::GetReferenceMapPoints()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return mvpReferenceMapPoints;
	}

	bool cMap::isMapUpdated()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return mbMapUpdated;
	}

	void cMap::SetFlagAfterBA()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mbMapUpdated = true;
	}

	void cMap::ResetUpdated()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		mbMapUpdated = false;
	}

	unsigned int cMap::GetMaxKFid()
	{
		std::unique_lock<std::mutex> lock(mMutexMap);
		return mnMaxKFid;
	}

	void cMap::clear()
	{
		for (std::set<cMapPoint*>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end();
			sit != send; ++sit)
			delete *sit;

		for (std::set<cMultiKeyFrame*>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end();
			sit != send; ++sit)
			delete *sit;

		mspMapPoints.clear();
		mspKeyFrames.clear();
		mnMaxKFid = 0;
		mvpReferenceMapPoints.clear();
	}
}
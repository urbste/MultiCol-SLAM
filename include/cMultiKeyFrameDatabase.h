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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>
#include <thread>
#include <mutex>

#include "cMultiKeyFrame.h"
#include "cMultiFrame.h"
#include "cORBVocabulary.h"

namespace MultiColSLAM
{
	class cMultiKeyFrame;
	class cMultiFrame;
	class cCamModelGeneral_;
	class cMultiCamSys_;
	class cMultiKeyFrameDatabase
	{
	public:
		cMultiKeyFrameDatabase(const ORBVocabulary &voc);
		void add(cMultiKeyFrame* pKF);

		void erase(cMultiKeyFrame* pKF);

		void clear();
		// Loop Detection
		std::vector<cMultiKeyFrame*> DetectLoopCandidates(cMultiKeyFrame* pKF, double minScore);
		// Relocalisation
		std::vector<cMultiKeyFrame*> DetectRelocalisationCandidates(cMultiFrame* F);
	protected:
		// Associated vocabulary
		const ORBVocabulary* mpVoc;
		// Inverted file
		std::vector<std::list<cMultiKeyFrame*> > mvInvertedFile;
		// Mutex
		std::mutex mMutex;
	};
}
#endif

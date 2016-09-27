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

#include "cMultiKeyFrameDatabase.h"

#include "cMultiKeyFrame.h"
#include "DBoW2/DBoW2/BowVector.h"

namespace MultiColSLAM
{
	using namespace std;

	cMultiKeyFrameDatabase::cMultiKeyFrameDatabase(const ORBVocabulary &voc) :
		mpVoc(&voc)
	{
		mvInvertedFile.resize(voc.size());
	}


	void cMultiKeyFrameDatabase::add(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(mMutex);

		for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end();
			vit != vend; vit++)
			mvInvertedFile[vit->first].push_back(pKF);
	}

	void cMultiKeyFrameDatabase::erase(cMultiKeyFrame* pKF)
	{
		std::unique_lock<std::mutex> lock(mMutex);

		// Erase elements in the Inverse File for the entry
		for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end();
			vit != vend; ++vit)
		{
			// List of keyframes that share the word
			std::list<cMultiKeyFrame*> &lKFs = mvInvertedFile[vit->first];

			for (std::list<cMultiKeyFrame*>::iterator lit = lKFs.begin(), lend = lKFs.end();
				lit != lend; ++lit)
			{
				if (pKF == *lit)
				{
					lKFs.erase(lit);
					break;
				}
			}
		}
	}

	void cMultiKeyFrameDatabase::clear()
	{
		mvInvertedFile.clear();
		mvInvertedFile.resize(mpVoc->size());
	}


	vector<cMultiKeyFrame*> cMultiKeyFrameDatabase::DetectLoopCandidates(cMultiKeyFrame* pKF,
		double minScore)
	{
		set<cMultiKeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
		list<cMultiKeyFrame*> lKFsSharingWords;

		// Search all keyframes that share a word with current keyframes
		// Discard keyframes connected to the query keyframe
		{
			std::unique_lock<std::mutex> lock(mMutex);

			for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end();
				vit != vend; ++vit)
			{
				list<cMultiKeyFrame*> &lKFs = mvInvertedFile[vit->first];

				for (list<cMultiKeyFrame*>::iterator lit = lKFs.begin(), lend = lKFs.end();
					lit != lend; ++lit)
				{
					cMultiKeyFrame* pKFi = *lit;
					if (pKFi->mnLoopQuery != pKF->mnId)
					{
						pKFi->mnLoopWords = 0;
						if (!spConnectedKeyFrames.count(pKFi))
						{
							pKFi->mnLoopQuery = pKF->mnId;
							lKFsSharingWords.push_back(pKFi);
						}
					}
					pKFi->mnLoopWords++;
				}
			}
		}

		if (lKFsSharingWords.empty())
			return vector<cMultiKeyFrame*>();

		list<pair<double, cMultiKeyFrame*> > lScoreAndMatch;

		// Only compare against those multikeyframes that share enough words
		int maxCommonWords = 0;
		for (list<cMultiKeyFrame*>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end();
			lit != lend; ++lit)
		{
			if ((*lit)->mnLoopWords > maxCommonWords)
				maxCommonWords = (*lit)->mnLoopWords;
		}

		int minCommonWords = static_cast<int>((double)maxCommonWords * 0.8);

		int nscores = 0;

		// Compute similarity score. Retain the matches whose score is higher than minScore
		for (list<cMultiKeyFrame*>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end();
			lit != lend; lit++)
		{
			cMultiKeyFrame* pKFi = *lit;

			if (pKFi->mnLoopWords > minCommonWords)
			{
				nscores++;

				double si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

				pKFi->mLoopScore = si;
				if (si >= minScore)
					lScoreAndMatch.push_back(make_pair(si, pKFi));
			}
		}

		if (lScoreAndMatch.empty())
			return vector<cMultiKeyFrame*>();

		list<pair<double, cMultiKeyFrame*> > lAccScoreAndMatch;
		double bestAccScore = minScore;

		// Lets now accumulate score by covisibility
		for (list<pair<double, cMultiKeyFrame*> >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end();
			it != itend; ++it)
		{
			cMultiKeyFrame* pKFi = it->second;
			vector<cMultiKeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

			double bestScore = it->first;
			double accScore = it->first;
			cMultiKeyFrame* pBestKF = pKFi;
			for (vector<cMultiKeyFrame*>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end();
				vit != vend; ++vit)
			{
				cMultiKeyFrame* pKF2 = *vit;
				if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords)
				{
					accScore += pKF2->mLoopScore;
					if (pKF2->mLoopScore > bestScore)
					{
						pBestKF = pKF2;
						bestScore = pKF2->mLoopScore;
					}
				}
			}

			lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
			if (accScore > bestAccScore)
				bestAccScore = accScore;
		}

		// Return all those keyframes with a score higher than 0.75*bestScore
		double minScoreToRetain = 0.75 * bestAccScore;

		set<cMultiKeyFrame*> spAlreadyAddedKF;
		std::vector<cMultiKeyFrame*> vpLoopCandidates;
		vpLoopCandidates.reserve(lAccScoreAndMatch.size());

		for (list<pair<double, cMultiKeyFrame*> >::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end();
			it != itend; ++it)
		{
			if (it->first > minScoreToRetain)
			{
				cMultiKeyFrame* pKFi = it->second;
				if (!spAlreadyAddedKF.count(pKFi))
				{
					vpLoopCandidates.push_back(pKFi);
					spAlreadyAddedKF.insert(pKFi);
				}
			}
		}


		return vpLoopCandidates;
	}

	std::vector<cMultiKeyFrame*> cMultiKeyFrameDatabase::DetectRelocalisationCandidates(cMultiFrame *F)
	{
		std::list<cMultiKeyFrame*> lKFsSharingWords;

		// Search all keyframes that share a word with current frame
		{
			std::unique_lock<std::mutex> lock(mMutex);

			for (DBoW2::BowVector::const_iterator vit = F->mBowVec.begin(), vend = F->mBowVec.end();
				vit != vend; ++vit)
			{
				std::list<cMultiKeyFrame*> &lKFs = mvInvertedFile[vit->first];

				for (std::list<cMultiKeyFrame*>::iterator lit = lKFs.begin(), lend = lKFs.end();
					lit != lend; lit++)
				{
					cMultiKeyFrame* pKFi = *lit;
					if (pKFi->mnRelocQuery != F->mnId)
					{
						pKFi->mnRelocWords = 0;
						pKFi->mnRelocQuery = F->mnId;
						lKFsSharingWords.push_back(pKFi);
					}
					pKFi->mnRelocWords++;
				}
			}
		}
		if (lKFsSharingWords.empty())
			return vector<cMultiKeyFrame*>();

		// Only compare against those keyframes that share enough words
		int maxCommonWords = 0;
		for (list<cMultiKeyFrame*>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end();
			lit != lend; lit++)
		{
			if ((*lit)->mnRelocWords > maxCommonWords)
				maxCommonWords = (*lit)->mnRelocWords;
		}

		int minCommonWords = static_cast<int>(maxCommonWords * 0.8);

		list<pair<double, cMultiKeyFrame*> > lScoreAndMatch;

		int nscores = 0;

		// Compute similarity score.
		for (list<cMultiKeyFrame*>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end();
			lit != lend; lit++)
		{
			cMultiKeyFrame* pKFi = *lit;

			if (pKFi->mnRelocWords > minCommonWords)
			{
				nscores++;
				double si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
				pKFi->mRelocScore = si;
				lScoreAndMatch.push_back(make_pair(si, pKFi));
			}
		}

		if (lScoreAndMatch.empty())
			return vector<cMultiKeyFrame*>();

		list<pair<double, cMultiKeyFrame*> > lAccScoreAndMatch;
		double bestAccScore = 0;

		// Lets now accumulate score by covisibility
		for (list<pair<double, cMultiKeyFrame*> >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end();
			it != itend; it++)
		{
			cMultiKeyFrame* pKFi = it->second;
			vector<cMultiKeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

			double bestScore = it->first;
			double accScore = bestScore;
			cMultiKeyFrame* pBestKF = pKFi;
			for (vector<cMultiKeyFrame*>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end();
				vit != vend; vit++)
			{
				cMultiKeyFrame* pKF2 = *vit;
				if (pKF2->mnRelocQuery != F->mnId)
					continue;

				accScore += pKF2->mRelocScore;
				if (pKF2->mRelocScore > bestScore)
				{
					pBestKF = pKF2;
					bestScore = pKF2->mRelocScore;
				}

			}
			lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
			if (accScore > bestAccScore)
				bestAccScore = accScore;
		}

		// Return all those keyframes with a score higher than 0.75*bestScore
		double minScoreToRetain = 0.75 * bestAccScore;
		set<cMultiKeyFrame*> spAlreadyAddedKF;
		std::vector<cMultiKeyFrame*> vpRelocCandidates;
		vpRelocCandidates.reserve(lAccScoreAndMatch.size());
		for (list<pair<double, cMultiKeyFrame*> >::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end();
			it != itend; it++)
		{
			const double &si = it->first;
			if (si > minScoreToRetain)
			{
				cMultiKeyFrame* pKFi = it->second;
				if (!spAlreadyAddedKF.count(pKFi))
				{
					vpRelocCandidates.push_back(pKFi);
					spAlreadyAddedKF.insert(pKFi);
				}
			}
		}

		return vpRelocCandidates;
	}
}
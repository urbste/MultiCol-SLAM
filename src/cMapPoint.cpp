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

#include "cMapPoint.h"
#include "cORBmatcher.h"

namespace MultiColSLAM
{
	long unsigned int cMapPoint::nNextId = 0;

	cMapPoint::cMapPoint(const cv::Vec3d &Pos,
		cMultiKeyFrame *pRefKF, cMap* pMap) :
		mnFirstKFid(pRefKF->mnId),
		mnTrackReferenceForFrame(0),
		mnLastFrameSeen(0),
		mnBALocalForKF(0),
		mnLoopPointForKF(0),
		mnCorrectedByKF(0),
		mnCorrectedReference(0),
		mpRefKF(pRefKF),
		mnVisible(1),
		mnFound(1),
		mbBad(false),
		mfMinDistance(0),
		mfMaxDistance(0),
		mpMap(pMap),
		mWorldPos(Pos),
		mNormalVector(cv::Vec3d(0.0, 0.0, 0.0))
	{
		int nrCams = pRefKF->camSystem.GetNrCams();
		for (int i = 0; i < nrCams; ++i)
		{
			mTrackProjX.push_back(0.0);
			mTrackProjY.push_back(0.0);
			mbTrackInView.push_back(false);
			mTrackViewCos.push_back(0.0);
			mnTrackScaleLevel.push_back(0);
		}
		mnId = nNextId++;
		mNormalVector = cv::Vec3d(0, 0, 0);
	}

	void cMapPoint::SetWorldPos(const cv::Vec3d &Pos)
	{
		std::unique_lock<std::mutex> lock(mMutexPos);
		mWorldPos = Pos;
	}

	cv::Vec3d cMapPoint::GetWorldPos()
	{
		std::unique_lock<std::mutex> lock(mMutexPos);
		return mWorldPos;
	}

	cv::Vec3d cMapPoint::GetNormal()
	{
		std::unique_lock<std::mutex> lock(mMutexPos);
		return mNormalVector;
	}

	cMultiKeyFrame* cMapPoint::GetReferenceKeyFrame()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mpRefKF;
	}

	void cMapPoint::AddObservation(cMultiKeyFrame* pKF, const size_t& idx)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		mObservations[pKF].push_back(idx); // push back since multiple image points per 3d point exist
	}

	void cMapPoint::EraseAllObservations(cMultiKeyFrame* pKF)
	{
		bool bBad = false;
		{
			std::unique_lock<std::mutex> lock(mMutexFeatures);
			if (mObservations.count(pKF))
			{
				mObservations.erase(pKF);

				if (mpRefKF == pKF)
					mpRefKF = mObservations.begin()->first;

				// If only 2 observations or less, discard point
				if (mObservations.size() <= 2)
					bBad = true;
			}
		}

		if (bBad)
			SetBadFlag();
	}

	void cMapPoint::EraseObservation(cMultiKeyFrame* pKF, const size_t& idx)
	{
		bool bBad = false;
		{
			std::unique_lock<std::mutex> lock(mMutexFeatures);

			int cnt = mObservations.count(pKF);
			//if (cnt && idx <= cnt)
			if (cnt)
			{
				// this is necessary, as we want to delete by value and not by position of idx
				mObservations[pKF].erase(
					std::remove(mObservations[pKF].begin(),
					mObservations[pKF].end(), idx),
					mObservations[pKF].end());

				if (mpRefKF == pKF)
					mpRefKF = mObservations.begin()->first;

				// If only 2 observations or less, discard point
			}
			size_t nrObs = 0;
			for (auto& it : mObservations)
				nrObs += it.second.size();

			//if (mObservations.size() > 4 && nrObs < 2)
			if (nrObs < 2)
				bBad = true;
		}

		if (bBad)
			SetBadFlag();
	}

	std::map<cMultiKeyFrame*, std::vector<size_t>> cMapPoint::GetObservations()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mObservations;
	}

	int cMapPoint::Observations()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mObservations.size();
	}

	int cMapPoint::TotalNrObservations()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		int cnt = 0;
		for (auto& it : mObservations)
		{
			if (it.first->isBad())
				continue;
			for (auto& it2 : it.second)
				++cnt;
		}
		return cnt;
	}

	void cMapPoint::SetScaledFlag()
	{
		std::unique_lock<std::mutex> lock1(mMutexFeatures);
		std::unique_lock<std::mutex> lock2(mMutexPos);
		scaled = true;
	}

	void cMapPoint::SetBadFlag()
	{
		std::map<cMultiKeyFrame*, std::vector<size_t>> obs;
		{
			std::unique_lock<std::mutex> lock1(mMutexFeatures);
			std::unique_lock<std::mutex> lock2(mMutexPos);
			mbBad = true;
			obs = mObservations;
			mObservations.clear();
		}
		for (std::map<cMultiKeyFrame*, std::vector<size_t>>::iterator mit = obs.begin(), mend = obs.end();
			mit != mend; mit++)
		{
			cMultiKeyFrame* pKF = mit->first;
			for (auto obsIt : mit->second)
				pKF->EraseMapPointMatch(obsIt);
		}

		mpMap->EraseMapPoint(this);
	}

	void cMapPoint::Replace(cMapPoint* pMP)
	{
		if (pMP->mnId == this->mnId)
			return;
		int visible = 0;
		int found = 0;
		std::map<cMultiKeyFrame*, std::vector<size_t>> obs;
		{
			std::unique_lock<std::mutex> lock1(mMutexFeatures);
			std::unique_lock<std::mutex> lock2(mMutexPos);
			obs = mObservations;
			mObservations.clear();
			mbBad = true;
			visible = mnVisible;
			found = mnFound;
		}
		bool havingMasks = false;
		for (std::map<cMultiKeyFrame*, std::vector<size_t>>::iterator mit = obs.begin(),
			mend = obs.end(); mit != mend; ++mit)
		{
			// Replace measurement in keyframe
			cMultiKeyFrame* pKF = mit->first;
			havingMasks = pKF->HavingMasks();
			for (auto obsIt : mit->second)
			{
				if (!pMP->IsInKeyFrame(pKF))
				{
					pKF->ReplaceMapPointMatch(obsIt, pMP);
					pMP->AddObservation(pKF, obsIt);
				}
				else
				{
					pKF->EraseMapPointMatch(obsIt);
				}
			}
		}
		pMP->ComputeDistinctiveDescriptors(havingMasks);
		cv::Mat desc = pMP->GetDescriptor();
		pMP->UpdateCurrentDescriptor(desc);
		pMP->IncreaseFound(found);
		pMP->IncreaseVisible(visible);
		mpMap->EraseMapPoint(this);
	}

	bool cMapPoint::isScaled()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		std::unique_lock<std::mutex> lock2(mMutexPos);
		return scaled;
	}

	bool cMapPoint::isBad()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		std::unique_lock<std::mutex> lock2(mMutexPos);
		return mbBad;
	}

	void cMapPoint::IncreaseVisible()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		++mnVisible;
	}

	void cMapPoint::IncreaseVisible(const int& val)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		mnVisible += val;
	}

	void cMapPoint::IncreaseFound()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		++mnFound;
	}

	void cMapPoint::IncreaseFound(const int& val)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		mnFound += val;
	}

	double cMapPoint::GetFoundRatio()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return static_cast<double>(mnFound) / mnVisible;
	}

	void cMapPoint::ComputeDistinctiveDescriptors(bool havingMasks)
	{
		// Retrieve all observed descriptors
		std::vector<cv::Mat> vDescriptors;
		std::vector<cv::Mat> vDescriptorMasks;

		std::map<cMultiKeyFrame*, std::vector<size_t> > observations;

		{
			std::unique_lock<std::mutex> lock1(mMutexFeatures);
			if (mbBad)
				return;
			observations = mObservations;
		}

		if (observations.empty())
			return;

		vDescriptors.reserve(observations.size());
		// loop through observations and get descriptors
		for (std::map<cMultiKeyFrame*, std::vector<size_t> >::iterator mit = observations.begin(), mend = observations.end();
			mit != mend; ++mit)
		{
			cMultiKeyFrame* pKF = mit->first;

			if (!pKF->isBad())
			{
				for (auto l : mit->second)
				{
					int cam = pKF->keypoint_to_cam.find(l)->second;
					int descIdx = pKF->cont_idx_to_local_cam_idx.find(l)->second;
					vDescriptors.push_back(pKF->GetDescriptor(cam, descIdx));
					if (havingMasks)
						vDescriptorMasks.push_back(pKF->GetDescriptorMask(cam, descIdx));
				}
			}
		}

		if (vDescriptors.empty())
			return;

		// Compute distances between them
		const size_t N = vDescriptors.size();

		//float Distances[N][N];
		cv::Mat Distances = cv::Mat::zeros(N, N, cv::DataType<int>::type);
		for (size_t i = 0; i < N; ++i)
		{
			//Distances.at<int>(i, i) = 0;
			for (size_t j = i + 1; j < N; ++j)
			{
				int distij = 0;
				if (havingMasks)
					distij = DescriptorDistance64Masked(vDescriptors[i].ptr<uint64_t>(0),
					vDescriptors[j].ptr<uint64_t>(0),
					vDescriptorMasks[i].ptr<uint64_t>(0),
					vDescriptorMasks[j].ptr<uint64_t>(0),
					vDescriptors[i].cols);
				else
					distij = DescriptorDistance64(vDescriptors[i].ptr<uint64_t>(0),
					vDescriptors[j].ptr<uint64_t>(0), vDescriptors[i].cols);

				Distances.ptr<int>(i)[j] = distij;
				Distances.ptr<int>(j)[i] = distij;
			}
		}

		// Take the descriptor with least median distance to the rest
		int BestMedian = INT_MAX;
		int BestIdx = 0;
		if (N > 2)
		{
			for (size_t i = 0; i < N - 1; ++i)
			{
				std::vector<int> vDists;
				for (int j = i + 1; j < N; ++j)
					vDists.push_back(Distances.ptr<int>(i)[j]);
				// get median distance
				int medianV = median(vDists);
				if (medianV < BestMedian)
				{
					BestMedian = medianV;
					BestIdx = i;
				}
			}
		}
		else BestIdx = 0;

		{
			std::unique_lock<std::mutex> lock(mMutexFeatures);
			mDescriptor = vDescriptors[BestIdx].clone();
			if (havingMasks)
				mDescriptorMask = vDescriptorMasks[BestIdx].clone();
		}
	}

	cv::Mat cMapPoint::GetDescriptor()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mDescriptor.clone();
	}

	cv::Mat cMapPoint::GetDescriptorMask()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mDescriptorMask.clone();
	}

	const uint64_t* cMapPoint::GetDescriptorMaskPtr()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mDescriptorMask.ptr<uint64_t>(0);
	}

	const uint64_t* cMapPoint::GetDescriptorPtr()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mDescriptor.ptr<uint64_t>(0);
	}

	cv::Mat cMapPoint::GetCurrentDescriptor()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mCurrentDescriptor.clone();
	}

	const uint64_t* cMapPoint::GetCurrentDescriptorPtr()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mCurrentDescriptor.ptr<uint64_t>(0);
	}

	void cMapPoint::UpdateCurrentDescriptor(cv::Mat& currDesc)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		//mCurrentDescriptor = currDesc.clone();
	}

	std::vector<size_t> cMapPoint::GetIndexInKeyFrame(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		std::vector<size_t> tmp;
		tmp.push_back(-1);
		if (mObservations.count(pKF))
			return mObservations[pKF];
		else
			return tmp;
	}

	bool cMapPoint::IsInKeyFrame(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return (mObservations.count(pKF));
	}

	void cMapPoint::UpdateNormalAndDepth()
	{
		std::map<cMultiKeyFrame*, std::vector<size_t> > observations;
		cMultiKeyFrame* pRefKF;
		cv::Vec3d Pos;
		{
			std::unique_lock<std::mutex> lock1(mMutexFeatures);
			std::unique_lock<std::mutex> lock2(mMutexPos);
			if (mbBad)
				return;
			observations = mObservations;
			pRefKF = mpRefKF;
			Pos = mWorldPos;
		}

		cv::Vec3d normal(0, 0, 0);
		int n = 0;
		for (std::map<cMultiKeyFrame*, std::vector<size_t> >::iterator mit = observations.begin(), mend = observations.end();
			mit != mend; ++mit)
		{
			cMultiKeyFrame* pKF = mit->first;
			cv::Vec3d Owi = pKF->GetCameraCenter();
			cv::Vec3d normali = mWorldPos - Owi;
			normal = normal + normali / cv::norm(normali);
			++n;
		}

		cv::Vec3d PC = Pos - pRefKF->GetCameraCenter();
		const double dist = cv::norm(PC);
		// TODO maybe for more than one observation?
		int level = 1;
		if (!observations[pRefKF].empty())
			level = pRefKF->GetKeyPointScaleLevel(observations[pRefKF][0]);
		const double scaleFactor = pRefKF->GetScaleFactor(level);
		const double levelScaleFactor = pRefKF->GetScaleFactor(level);
		const int nLevels = pRefKF->GetScaleLevels();

		{
			std::unique_lock<std::mutex> lock3(mMutexPos);
			mfMinDistance = (1.0 / scaleFactor)*dist / levelScaleFactor;
			mfMaxDistance = scaleFactor * dist * pRefKF->GetScaleFactor(nLevels - 1 - level);
			mNormalVector = normal / n;
		}
	}

	double cMapPoint::GetMinDistanceInvariance()
	{
		std::unique_lock<std::mutex> lock(mMutexPos);
		return 0.8*mfMinDistance;
	}

	double cMapPoint::GetMaxDistanceInvariance()
	{
		std::unique_lock<std::mutex> lock(mMutexPos);
		return 1.2*mfMaxDistance;
	}

}
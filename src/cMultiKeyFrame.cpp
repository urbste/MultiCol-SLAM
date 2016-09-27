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

#include "cMultiKeyFrame.h"
#include "cConverter.h"
#include "math.h"

namespace MultiColSLAM
{
	long unsigned int cMultiKeyFrame::nNextId = 0;

	cMultiKeyFrame::cMultiKeyFrame(cMultiFrame &F,
		cMap *pMap,
		cMultiKeyFrameDatabase *pKFDB) :
		mnFrameId(F.mnId),
		mTimeStamp(F.mTimeStamp),
		mfGridElementWidthInv(F.mfGridElementWidthInv),
		mfGridElementHeightInv(F.mfGridElementHeightInv),
		mnTrackReferenceForFrame(0), mnBALocalForKF(0),
		mnBAFixedForKF(0),
		mnLoopQuery(0),
		mnRelocQuery(0),
		mBowVec(F.mBowVec),
		mBowVecs(F.mBowVecs),
		mFeatVec(F.mFeatVec),
		mFeatVecs(F.mFeatVecs),
		images(F.images),
		camSystem(F.camSystem),
		mvKeys(F.mvKeys),
		mvKeysRays(F.mvKeysRays),
		mDescriptors(F.mDescriptors),
		mvpMapPoints(F.mvpMapPoints),
		keypoint_to_cam(F.keypoint_to_cam),
		cont_idx_to_local_cam_idx(F.cont_idx_to_local_cam_idx),
		mpKeyFrameDB(pKFDB),
		mpORBvocabulary(F.mpORBvocabulary),
		mbFirstConnection(true),
		mpParent(NULL),
		mbNotErase(false),
		mbToBeErased(false),
		mbBad(false),
		mnScaleLevels(F.mnScaleLevels),
		mvScaleFactors(F.mvScaleFactors),
		mvLevelSigma2(F.mvLevelSigma2),
		mvInvLevelSigma2(F.mvInvLevelSigma2),
		mpMap(pMap),
		mnMinX(F.mnMinX),
		mnMinY(F.mnMinY),
		mnMaxX(F.mnMaxX),
		mnMaxY(F.mnMaxY),
		mdBRIEF(F.Doing_mdBRIEF()),
		masksLearned(F.HavingMasks()),
		descDimension(F.DescDims()),
		mDescriptorMasks(F.mDescriptorMasks),
		IamTheReference(false),
		IamLoopCandidate(false),
		imageId(0)
	{
		int nrCams = camSystem.GetNrCams();
		mnId = nNextId++;
		mnGridCols.resize(nrCams);
		mnGridRows.resize(nrCams);
		mGrids.resize(nrCams);
		SetPose(F.GetPose());

		for (int c = 0; c < nrCams; ++c)
		{
			mnGridCols[c] = FRAME_GRID_COLS;
			mnGridRows[c] = FRAME_GRID_ROWS;
			mGrids[c].resize(mnGridCols[c]);
			for (int i = 0; i < mnGridCols[c]; i++)
			{
				mGrids[c][i].resize(mnGridRows[c]);
				for (int j = 0; j < mnGridRows[c]; j++)
					mGrids[c][i][j] = F.mGrids[c][i][j];
			}

		}
	}


	void cMultiKeyFrame::ComputeBoW()
	{
		if (mBowVec.empty() || mFeatVec.empty())
		{
			std::vector<cv::Mat> vCurrentDesc = cConverter::toDescriptorVector(mDescriptors);
			// Feature vector associate features with nodes in the 4th level (from leaves up)
			// We assume the vocabulary tree has 6 levels, change the 4 otherwise
			mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
		}
		const int nrCams = camSystem.GetNrCams();
		mBowVecs.resize(nrCams);
		mFeatVecs.resize(nrCams);
	}

	void cMultiKeyFrame::SetPose(const cv::Matx33d &Rcw, const cv::Vec3d &tcw)
	{
		std::unique_lock<std::mutex> lock(mMutexPose);
		camSystem.Set_M_t(cConverter::Rt2Hom(Rcw, tcw));
	}

	void cMultiKeyFrame::SetPose(const cv::Matx44d &Tcw_)
	{
		std::unique_lock<std::mutex> lock(mMutexPose);
		camSystem.Set_M_t(Tcw_);
		cv::Matx33d Rcw = cConverter::Hom2R(Tcw_);
		cv::Vec3d tcw = cConverter::Hom2T(Tcw_);
	}

	void cMultiKeyFrame::SetPose(const cv::Matx61d &Tcw_min_)
	{
		std::unique_lock<std::mutex> lock(mMutexPose);
		camSystem.Set_M_t_from_min(Tcw_min_);
		cv::Matx44d T = cayley2hom(Tcw_min_);
		//cv::Matx44d T = rodrigues2hom(Tcw_min_);
		cv::Matx33d Rcw = cConverter::Hom2R(T);
		cv::Vec3d tcw = cConverter::Hom2T(T);
	}

	cv::Matx44d cMultiKeyFrame::GetPose()
	{
		std::unique_lock<std::mutex> lock(mMutexPose);
		return camSystem.Get_M_t();
	}

	cv::Matx44d cMultiKeyFrame::GetPoseInverse()
	{
		std::unique_lock<std::mutex> lock(mMutexPose);
		cv::Matx44d Twc = cConverter::invMat(camSystem.Get_M_t());
		return Twc;
	}

	cv::Vec3d cMultiKeyFrame::GetCameraCenter()
	{
		std::unique_lock<std::mutex> lock(mMutexPose);
		cv::Matx44d Mt = camSystem.Get_M_t();
		cv::Vec3d Ow = cConverter::Hom2T(Mt);
		return Ow;
	}

	cv::Matx33d cMultiKeyFrame::GetRotation()
	{
		std::unique_lock<std::mutex> lock(mMutexPose);
		cv::Matx33d Rcw = cConverter::Hom2R(camSystem.Get_M_t());
		return Rcw;
	}

	cv::Vec3d cMultiKeyFrame::GetTranslation()
	{
		std::unique_lock<std::mutex> lock(mMutexPose);
		cv::Vec3d tcw = cConverter::Hom2T(camSystem.Get_M_t());
		return tcw;
	}

	void cMultiKeyFrame::AddConnection(cMultiKeyFrame *pKF, const int &weight)
	{
		{
			std::unique_lock<std::mutex> lock(mMutexConnections);
			if (!mConnectedKeyFrameWeights.count(pKF))
				mConnectedKeyFrameWeights[pKF] = weight;
			else if (mConnectedKeyFrameWeights[pKF] != weight)
				mConnectedKeyFrameWeights[pKF] = weight;
			else
				return;
		}

		UpdateBestCovisibles();
	}

	void cMultiKeyFrame::UpdateBestCovisibles()
	{
		std::unique_lock<std::mutex> lock(mMutexConnections);
		std::vector<std::pair<int, cMultiKeyFrame*> > vPairs;
		vPairs.reserve(mConnectedKeyFrameWeights.size());
		for (std::map<cMultiKeyFrame*, int>::iterator mit =
			mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end(); mit != mend; ++mit)
			vPairs.push_back(std::make_pair(mit->second, mit->first));

		std::sort(vPairs.begin(), vPairs.end());
		std::list<cMultiKeyFrame*> lKFs;
		std::list<int> lWs;
		for (size_t i = 0, iend = vPairs.size(); i < iend; i++)
		{
			lKFs.push_front(vPairs[i].second);
			lWs.push_front(vPairs[i].first);
		}

		mvpOrderedConnectedKeyFrames = std::vector<cMultiKeyFrame*>(lKFs.begin(), lKFs.end());
		mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
	}

	std::set<cMultiKeyFrame*> cMultiKeyFrame::GetConnectedKeyFrames()
	{
		std::unique_lock<std::mutex> lock(mMutexConnections);
		std::set<cMultiKeyFrame*> s;
		for (std::map<cMultiKeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin();
			mit != mConnectedKeyFrameWeights.end(); ++mit)
			s.insert(mit->first);
		return s;
	}

	std::vector<cMultiKeyFrame*> cMultiKeyFrame::GetVectorCovisibleKeyFrames()
	{
		std::unique_lock<std::mutex> lock(mMutexConnections);
		return mvpOrderedConnectedKeyFrames;
	}

	std::vector<cMultiKeyFrame*> cMultiKeyFrame::GetBestCovisibilityKeyFrames(const int &N)
	{
		std::unique_lock<std::mutex> lock(mMutexConnections);
		if ((int)mvpOrderedConnectedKeyFrames.size() < N)
			return mvpOrderedConnectedKeyFrames;
		else
			return std::vector<cMultiKeyFrame*>(
			mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);

	}

	std::vector<cMultiKeyFrame*> cMultiKeyFrame::GetCovisiblesByWeight(const int &w)
	{
		std::unique_lock<std::mutex> lock(mMutexConnections);

		if (mvpOrderedConnectedKeyFrames.empty())
			return std::vector<cMultiKeyFrame*>();

		std::vector<int>::iterator it =
			upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, cMultiKeyFrame::weightComp);
		if (it == mvOrderedWeights.end())
			return std::vector<cMultiKeyFrame*>();
		else
		{
			int n = it - mvOrderedWeights.begin();
			return std::vector<cMultiKeyFrame*>(
				mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
		}
	}

	int cMultiKeyFrame::GetWeight(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(mMutexConnections);
		if (mConnectedKeyFrameWeights.count(pKF))
			return mConnectedKeyFrameWeights[pKF];
		else
			return 0;
	}

	void cMultiKeyFrame::AddMapPoint(cMapPoint *pMP, const size_t &idx)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		mvpMapPoints[idx] = pMP;
	}

	void cMultiKeyFrame::EraseMapPointMatch(const size_t &idx)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		mvpMapPoints[idx] = NULL;
	}

	void cMultiKeyFrame::ReplaceMapPointMatch(const size_t &idx, cMapPoint* pMP)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		mvpMapPoints[idx] = pMP;
	}

	std::set<cMapPoint*> cMultiKeyFrame::GetMapPoints()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		std::set<cMapPoint*> s;
		for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; ++i)
		{
			if (!mvpMapPoints[i])
				continue;
			cMapPoint* pMP = mvpMapPoints[i];
			if (!pMP->isBad())
				s.insert(pMP);
		}
		return s;
	}

	int cMultiKeyFrame::TrackedMapPoints()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);

		int nPoints = 0;
		for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; ++i)
		{
			//if (!alreadyCounted.count(mvpMapPoints[i]->mnId))
			if (mvpMapPoints[i])
				++nPoints;
			//alreadyCounted[mvpMapPoints[i]->mnId] = 1;
		}

		return nPoints;
	}

	std::vector<cMapPoint*> cMultiKeyFrame::GetMapPointMatches()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mvpMapPoints;
	}

	cMapPoint* cMultiKeyFrame::GetMapPoint(const size_t &idx)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mvpMapPoints[idx];
	}

	cv::Vec3d cMultiKeyFrame::GetKeyPointRay(const size_t &idx) const
	{
		return mvKeysRays[idx];
	}

	cv::KeyPoint cMultiKeyFrame::GetKeyPoint(const size_t &idx) const
	{
		return mvKeys[idx];
	}

	int cMultiKeyFrame::GetKeyPointScaleLevel(const size_t &idx) const
	{
		return mvKeys[idx].octave;
	}

	cv::Mat cMultiKeyFrame::GetDescriptor(const int& cam, const size_t &idx) const
	{
		return mDescriptors[cam].row(idx).clone();
	}

	cv::Mat cMultiKeyFrame::GetDescriptorMask(const int& cam, const size_t &idx) const
	{
		return mDescriptorMasks[cam].row(idx).clone();
	}

	const uint64_t* cMultiKeyFrame::GetDescriptorMaskRowPtr(const int& cam, const size_t &idx) const
	{
		return mDescriptorMasks[cam].ptr<uint64_t>(idx);
	}

	const uint64_t* cMultiKeyFrame::GetDescriptorRowPtr(const int& cam, const size_t &idx) const
	{
		return mDescriptors[cam].ptr<uint64_t>(idx);
	}

	std::vector<cv::KeyPoint> cMultiKeyFrame::GetKeyPoints() const
	{
		return mvKeys;
	}

	std::vector<cv::Vec3d> cMultiKeyFrame::GetKeyPointsRays() const
	{
		return mvKeysRays;
	}

	DBoW2::FeatureVector cMultiKeyFrame::GetFeatureVector()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mFeatVec;
	}

	DBoW2::FeatureVector cMultiKeyFrame::GetFeatureVector(int& c)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mFeatVecs[c];
	}

	DBoW2::BowVector cMultiKeyFrame::GetBowVector()
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mBowVec;
	}

	DBoW2::BowVector cMultiKeyFrame::GetBowVector(int& c)
	{
		std::unique_lock<std::mutex> lock(mMutexFeatures);
		return mBowVecs[c];
	}

	cv::Mat cMultiKeyFrame::GetImage(const int& cam)
	{
		std::unique_lock<std::mutex> lock(mMutexImage);
		return images[cam].clone();
	}

	void cMultiKeyFrame::UpdateConnections()
	{
		std::map<cMultiKeyFrame*, int> KFcounter;

		std::vector<cMapPoint*> vpMP;

		{
			std::unique_lock<std::mutex> lockMPs(mMutexFeatures);
			vpMP = mvpMapPoints;
		}

		//For all map points in keyframe check in which other keyframes are they seen
		//Increase counter for those keyframes
		for (std::vector<cMapPoint*>::iterator vit = vpMP.begin(), vend = vpMP.end();
			vit != vend; vit++)
		{
			cMapPoint* pMP = *vit;

			if (!pMP)
				continue;

			if (pMP->isBad())
				continue;

			std::map<cMultiKeyFrame*, std::vector<size_t>> observations = pMP->GetObservations();

			for (std::map<cMultiKeyFrame*, std::vector<size_t>>::iterator mit = observations.begin(),
				mend = observations.end(); mit != mend; mit++)
			{
				if (mit->first->mnId == mnId)
					continue;
				//for (auto& obsIdx: mit->second)
				//KFcounter[mit->first] += mit->second.size();
				KFcounter[mit->first]++;
			}
		}

		if (KFcounter.empty())
			return;

		// If the counter is greater than threshold add connection
		// In case no keyframe counter is over threshold add the one with maximum counter
		int nmax = 0;
		cMultiKeyFrame* pKFmax = NULL;
		int th = 30;

		std::vector<std::pair<int, cMultiKeyFrame*> > vPairs;
		vPairs.reserve(KFcounter.size());
		for (std::map<cMultiKeyFrame*, int>::iterator mit = KFcounter.begin(),
			mend = KFcounter.end(); mit != mend; ++mit)
		{
			if (mit->second > nmax)
			{
				nmax = mit->second;
				pKFmax = mit->first;
			}
			if (mit->second >= th)
			{
				vPairs.push_back(std::make_pair(mit->second, mit->first));
				(mit->first)->AddConnection(this, mit->second);
			}
		}

		if (vPairs.empty())
		{
			vPairs.push_back(std::make_pair(nmax, pKFmax));
			pKFmax->AddConnection(this, nmax);
		}

		sort(vPairs.begin(), vPairs.end());
		std::list<cMultiKeyFrame*> lKFs;
		std::list<int> lWs;
		for (size_t i = 0; i < vPairs.size(); ++i)
		{
			lKFs.push_front(vPairs[i].second);
			lWs.push_front(vPairs[i].first);
		}

	{
		std::unique_lock<std::mutex> lockCon(mMutexConnections);

		//mspConnectedKeyFrames = spConnectedKeyFrames;
		mConnectedKeyFrameWeights = KFcounter;
		mvpOrderedConnectedKeyFrames = std::vector<cMultiKeyFrame*>(lKFs.begin(), lKFs.end());
		mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());

		if (mbFirstConnection && mnId != 0)
		{
			mpParent = mvpOrderedConnectedKeyFrames.front();
			mpParent->AddChild(this);
			mbFirstConnection = false;
		}

	}
	}

	void cMultiKeyFrame::AddChild(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lockCon(mMutexConnections);
		mspChildrens.insert(pKF);
	}

	void cMultiKeyFrame::EraseChild(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lockCon(mMutexConnections);
		mspChildrens.erase(pKF);
	}

	void cMultiKeyFrame::ChangeParent(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lockCon(mMutexConnections);
		mpParent = pKF;
		pKF->AddChild(this);
	}

	std::set<cMultiKeyFrame*> cMultiKeyFrame::GetChilds()
	{
		std::unique_lock<std::mutex> lockCon(mMutexConnections);
		return mspChildrens;
	}

	cMultiKeyFrame* cMultiKeyFrame::GetParent()
	{
		std::unique_lock<std::mutex> lockCon(mMutexConnections);
		return mpParent;
	}

	bool cMultiKeyFrame::hasChild(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lockCon(mMutexConnections);
		return mspChildrens.count(pKF);
	}

	void cMultiKeyFrame::AddLoopEdge(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lockCon(mMutexConnections);
		mbNotErase = true;
		mspLoopEdges.insert(pKF);
	}

	std::set<cMultiKeyFrame*> cMultiKeyFrame::GetLoopEdges()
	{
		std::unique_lock<std::mutex> lockCon(mMutexConnections);
		return mspLoopEdges;
	}

	void cMultiKeyFrame::SetNotErase()
	{
		std::unique_lock<std::mutex> lock(mMutexConnections);
		mbNotErase = true;
	}

	void cMultiKeyFrame::SetErase()
	{
		{
			std::unique_lock<std::mutex> lock(mMutexConnections);
			if (mspLoopEdges.empty())
			{
				mbNotErase = false;
			}
		}

		if (mbToBeErased)
		{
			SetBadFlag();
		}
	}

	void cMultiKeyFrame::SetBadFlag()
	{
		{
			std::unique_lock<std::mutex> lock(mMutexConnections);
			if (mnId == 0)
				return;
			else if (mbNotErase)
			{
				mbToBeErased = true;
				return;
			}
		}

		for (std::map<cMultiKeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(),
			mend = mConnectedKeyFrameWeights.end(); mit != mend; mit++)
			mit->first->EraseConnection(this);

		for (size_t i = 0; i < mvpMapPoints.size(); ++i)
			if (mvpMapPoints[i])
				mvpMapPoints[i]->EraseAllObservations(this);

		{
			std::unique_lock<std::mutex> lock(mMutexConnections);
			std::unique_lock<std::mutex> lock1(mMutexFeatures);

			mConnectedKeyFrameWeights.clear();
			mvpOrderedConnectedKeyFrames.clear();

			// Update Spanning Tree
			std::set<cMultiKeyFrame*> sParentCandidates;
			sParentCandidates.insert(mpParent);

			// Assign at each iteration one children with a parent (the pair with highest covisibility weight)
			// Include that children as new parent candidate for the rest
			while (!mspChildrens.empty())
			{
				bool bContinue = false;

				int max = -1;
				cMultiKeyFrame* pC;
				cMultiKeyFrame* pP;

				for (std::set<cMultiKeyFrame*>::iterator sit = mspChildrens.begin(), send = mspChildrens.end();
					sit != send; sit++)
				{
					cMultiKeyFrame* pKF = *sit;
					if (pKF->isBad())
						continue;

					// Check if a parent candidate is connected to the keyframe
					std::vector<cMultiKeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
					for (size_t i = 0, iend = vpConnected.size(); i < iend; i++)
					{
						for (std::set<cMultiKeyFrame*>::iterator spcit = sParentCandidates.begin(),
							spcend = sParentCandidates.end(); spcit != spcend; spcit++)
						{
							if (vpConnected[i]->mnId == (*spcit)->mnId)
							{
								int w = pKF->GetWeight(vpConnected[i]);
								if (w > max)
								{
									pC = pKF;
									pP = vpConnected[i];
									max = w;
									bContinue = true;
								}
							}
						}
					}
				}

				if (bContinue)
				{
					pC->ChangeParent(pP);
					sParentCandidates.insert(pC);
					mspChildrens.erase(pC);
				}
				else
					break;
			}

			// If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
			if (!mspChildrens.empty())
				for (std::set<cMultiKeyFrame*>::iterator sit = mspChildrens.begin();
					sit != mspChildrens.end(); sit++)
			{
				(*sit)->ChangeParent(mpParent);
			}

			mpParent->EraseChild(this);
			mbBad = true;
		}


		mpMap->EraseKeyFrame(this);
		mpKeyFrameDB->erase(this);
	}

	bool cMultiKeyFrame::isBad()
	{
		std::unique_lock<std::mutex> lock(mMutexConnections);
		return mbBad;
	}

	void cMultiKeyFrame::EraseConnection(cMultiKeyFrame* pKF)
	{
		bool bUpdate = false;
		{
			std::unique_lock<std::mutex> lock(mMutexConnections);
			if (mConnectedKeyFrameWeights.count(pKF))
			{
				mConnectedKeyFrameWeights.erase(pKF);
				bUpdate = true;
			}
		}

		if (bUpdate)
			UpdateBestCovisibles();
	}

	std::vector<size_t> cMultiKeyFrame::GetFeaturesInArea(
		const int& cam,
		const double &x,
		const double &y,
		const double &r) const
	{
		std::vector<size_t> vIndices;

		int nMinCellX = floor((x - mnMinX[cam] - r)*mfGridElementWidthInv[cam]);
		nMinCellX = std::max(0, nMinCellX);
		if (nMinCellX >= mnGridCols[cam])
			return vIndices;

		int nMaxCellX = ceil((x - mnMinX[cam] + r)*mfGridElementWidthInv[cam]);
		nMaxCellX = std::min(mnGridCols[cam] - 1, nMaxCellX);
		if (nMaxCellX < 0)
			return vIndices;

		int nMinCellY = floor((y - mnMinY[cam] - r)*mfGridElementHeightInv[cam]);
		nMinCellY = std::max(0, nMinCellY);
		if (nMinCellY >= mnGridRows[cam])
			return vIndices;

		int nMaxCellY = ceil((y - mnMinY[cam] + r)*mfGridElementHeightInv[cam]);
		nMaxCellY = std::min(mnGridRows[cam] - 1, nMaxCellY);
		if (nMaxCellY < 0)
			return vIndices;

		for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
		{
			std::vector<std::vector<size_t>> vCell = mGrids[cam][ix];
			for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
			{
				for (size_t j = 0, jend = vCell[iy].size(); j < jend; j++)
				{
					const cv::KeyPoint &kpUn = mvKeys[vCell[iy][j]];
					if (abs(kpUn.pt.x - x) <= r && abs(kpUn.pt.y - y) <= r)
						vIndices.push_back(vCell[iy][j]);
				}
			}
		}

		return vIndices;
	}

	bool cMultiKeyFrame::IsInImage(const int& cam, const double &x, const double &y) const
	{
		return (x >= mnMinX[cam] &&
			x < mnMaxX[cam] &&
			y >= mnMinY[cam] &&
			y < mnMaxY[cam]);
	}

	double cMultiKeyFrame::ComputeSceneMedianDepth(int q)
	{
		std::vector<cMapPoint*> vpMapPoints;
		cv::Matx44d Tcw_;
		{
			std::unique_lock<std::mutex> lock(mMutexFeatures);
			std::unique_lock<std::mutex> lock2(mMutexPose);
			vpMapPoints = mvpMapPoints;
		}

		std::vector<double> vDepths;
		vDepths.reserve(mvpMapPoints.size());
		for (size_t i = 0; i < mvpMapPoints.size(); ++i)
		{
			if (mvpMapPoints[i])
			{
				cMapPoint* pMP = mvpMapPoints[i];
				cv::Vec3d x3Dw = pMP->GetWorldPos();
				cv::Vec4d x4Dw = cv::Vec4d(x3Dw(0), x3Dw(1), x3Dw(2), 1.0);

				int camIdx = keypoint_to_cam.find(i)->second;
				cv::Matx44d rot = camSystem.Get_MtMc_inv(camIdx);
				cv::Vec4d rotVec = rot*x4Dw;
				double z = rotVec(2);
				vDepths.push_back(z);
			}
		}

		std::sort(vDepths.begin(), vDepths.end());

		return vDepths[(vDepths.size() - 1) / q];
	}

	size_t cMultiKeyFrame::GetValidMapPointCnt()
	{
		size_t cnt = 0;
		for (auto& it : mvpMapPoints)
			if (it)
				++cnt;
		return cnt;
	}

	size_t cMultiKeyFrame::GetNrKeypointsInFrame()
	{
		return this->mvKeys.size();
	}

	bool cMultiKeyFrame::IsReference()
	{
		std::unique_lock<std::mutex> lock(mMutexProperties);
		return IamTheReference;
	}

	bool cMultiKeyFrame::IsLoopCandidate()
	{
		std::unique_lock<std::mutex> lock(mMutexProperties);
		return IamLoopCandidate;
	}

	void cMultiKeyFrame::SetReference(const bool ref)
	{
		std::unique_lock<std::mutex> lock(mMutexProperties);
		IamTheReference = ref;
	}
	void cMultiKeyFrame::SetLoopCandidate(const bool ref)
	{
		std::unique_lock<std::mutex> lock(mMutexProperties);
		IamLoopCandidate = ref;
	}


}
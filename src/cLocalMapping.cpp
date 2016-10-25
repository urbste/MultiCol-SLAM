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

#include "cLocalMapping.h"
#include "cLoopClosing.h"
#include "cORBmatcher.h"
#include "cOptimizer.h"
#include "cConverter.h"

// opengv
#include <opengv/triangulation/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

namespace MultiColSLAM
{
	const double cosThresh = cos(3.0 * M_PID / 180.0);
	const double mean3DPointError = 0.25;
	const double pixelSize = 6e-6; // in meter
	const double minTriangQuali = 0.2;
	const double maxDIST = 25.0;

	cLocalMapping::cLocalMapping(cMap *pMap) :
		mbResetRequested(false),
		mpMap(pMap),
		mbAbortBA(false),
		mbStopped(false),
		mbStopRequested(false),
		mbAcceptMultiKeyFrames(true),
		scaleInitialMap(false),
		descDim(32),
		havingMasks(false),
		mbFinishRequested(false)
	{
	}

	void cLocalMapping::SetLoopCloser(cLoopClosing* pLoopCloser)
	{
		mpLoopCloser = pLoopCloser;
	}

	void cLocalMapping::SetTracker(cTracking *pTracker)
	{
		mpTracker = pTracker;
	}

	void cLocalMapping::Run()
	{
		std::chrono::steady_clock::time_point begin;
		std::chrono::steady_clock::time_point end;
		while (true)
		{
			// Tracking will see that Local Mapping is busy
			SetAcceptMultiKeyFrames(false);
			// Check if there are keyframes in the queue
			if (CheckNewMultiKeyFrames())
			{
				// BoW conversion and insertion in Map
				// and rendering of depth images
				ProcessNewMultiKeyFrame();

				// Check recent MapPoints
				MapPointCulling();

				// Triangulate new MapPoints
				CreateNewMapPoints();

				// Find more matches in neighbor keyframes and fuse point duplications
				if (!CheckNewMultiKeyFrames())
					SearchInNeighbors();

				mbAbortBA = false;
				if (!CheckNewMultiKeyFrames() && mpMap->KeyFramesInMap() >= 2 &&
					!stopRequested())
				{
					// Local BA
					cOptimizer::LocalBundleAdjustment(mpCurrentMultiKeyFrame, mpMap,
						5, true, &mbAbortBA);

					// Check redundant local Keyframes
					KeyFrameCulling();
				}
				mpLoopCloser->InsertKeyFrame(mpCurrentMultiKeyFrame);
			}
			// Safe area to stop
			else if (Stop())
			{
				while (isStopped() && !CheckFinish())
					std::this_thread::sleep_for(std::chrono::milliseconds(500));
				if (CheckFinish())
					break;
			}

			SetAcceptMultiKeyFrames(true);
			ResetIfRequested();

			if (CheckFinish())
				break;

			if (mpMap->KeyFramesInMap() > 4)
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			else
				std::this_thread::sleep_for(std::chrono::milliseconds(1));

		}
		SetFinish();
	}

	void cLocalMapping::InsertMultiKeyFrame(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(mMutexNewKFs);
		mlNewMultiKeyFrames.push_back(pKF);
		mbAbortBA = true;
		SetAcceptMultiKeyFrames(false);
	}

	bool cLocalMapping::CheckNewMultiKeyFrames()
	{
		std::unique_lock<std::mutex> lock(mMutexNewKFs);
		return (!mlNewMultiKeyFrames.empty());
	}

	void cLocalMapping::ProcessNewMultiKeyFrame()
	{
		{
			std::unique_lock<std::mutex> lock(mMutexNewKFs);
			mpCurrentMultiKeyFrame = mlNewMultiKeyFrames.front();
			mlNewMultiKeyFrames.pop_front();
		}

		std::chrono::steady_clock::time_point begin;
		std::chrono::steady_clock::time_point end;

		// Compute Bags of Words structures
		mpCurrentMultiKeyFrame->ComputeBoW();

		// Associate MapPoints to the new keyframe and update normal and descriptor
		std::vector<cMapPoint*> vpMapPointMatches =
			mpCurrentMultiKeyFrame->GetMapPointMatches();

		for (size_t i = 0; i < vpMapPointMatches.size(); ++i)
		{
			cMapPoint* pMP = vpMapPointMatches[i];
			if (pMP)
			{
				if (!pMP->isBad())
				{
					if (!pMP->IsInKeyFrame(mpCurrentMultiKeyFrame))
					{
						pMP->AddObservation(mpCurrentMultiKeyFrame, i);
						pMP->UpdateNormalAndDepth();
						pMP->ComputeDistinctiveDescriptors();
					}
				}
			}
		}

		// Update links in the Covisibility Graph
		mpCurrentMultiKeyFrame->UpdateConnections();

		// Insert Keyframe in Map
		mpMap->AddKeyFrame(mpCurrentMultiKeyFrame);
	}

	void cLocalMapping::MapPointCulling()
	{
		// Check Recent Added MapPoints
		std::list<cMapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
		const unsigned long int nCurrentKFid = mpCurrentMultiKeyFrame->mnId;
		while (lit != mlpRecentAddedMapPoints.end())
		{
			cMapPoint* pMP = *lit;
			if (pMP->isBad())
			{
				lit = mlpRecentAddedMapPoints.erase(lit);
				//mpMap->EraseMapPoint(pMP);
			}
			else if (pMP->GetFoundRatio() < 0.25)
			{
				pMP->SetBadFlag();
				lit = mlpRecentAddedMapPoints.erase(lit);
				//mpMap->EraseMapPoint(pMP);
			}
			else if ((nCurrentKFid - pMP->mnFirstKFid) >= 2 &&
				pMP->Observations() <= 2)
			{
				pMP->SetBadFlag();
				lit = mlpRecentAddedMapPoints.erase(lit);
				//mpMap->EraseMapPoint(pMP);
			}
			else if ((nCurrentKFid - pMP->mnFirstKFid) >= 3)
			{
				lit = mlpRecentAddedMapPoints.erase(lit);
				//mpMap->EraseMapPoint(pMP);
			}
			else
				lit++;
		}
	}

	void cLocalMapping::CreateNewMapPoints()
	{
		// Take neighbor keyframes in covisibility graph
		std::vector<cMultiKeyFrame*> vpNeighKFs = mpCurrentMultiKeyFrame->GetBestCovisibilityKeyFrames(5);

		cORBmatcher matcher(0.8, checkOrientation, descDim, havingMasks);

		cv::Vec3d Ow1 = mpCurrentMultiKeyFrame->GetCameraCenter();

		const double ratioFactor = 1.5 * mpCurrentMultiKeyFrame->GetScaleFactor();
		int testCnt = 0, fixCnt = 0;
		double c_in_m1 =
			std::abs(mpCurrentMultiKeyFrame->camSystem.GetCamModelObj(0).Get_P().at<double>(0)) *
			pixelSize;

		// TODO do that for all cameras!!!
		for (size_t i = 0; i < vpNeighKFs.size(); ++i)
		{
			const int nrCams = vpNeighKFs[i]->camSystem.GetNrCams();

			cMultiKeyFrame* pKF2 = vpNeighKFs[i];
			// Check first that baseline is not too short
			// Small translation errors for short baseline keyframes make scale to diverge
			cv::Vec3d Ow2 = pKF2->GetCameraCenter();
			cv::Vec3d vBaseline = Ow2 - Ow1;
			const double baseline = cv::norm(vBaseline);
			const double medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
			const double ratioBaselineDepth = baseline / medianDepthKF2;
			//cout << "ratioBaselineDepth " << ratioBaselineDepth << endl;
			//if (baseline < 0.25 && medianDepthKF2 < 0.04)
			if (ratioBaselineDepth < 0.01)
				continue;

			// Search matches that fulfill epipolar constraint
			std::vector<cv::Vec3d> vMatchedKeysRays1;
			std::vector<cv::Vec3d> vMatchedKeysRays2;

			std::vector<cv::KeyPoint> vMatchedKeys1;
			std::vector<cv::KeyPoint> vMatchedKeys2;

			std::vector<std::pair<size_t, size_t> > vMatchedIndices;
			matcher.SearchForTriangulationRaw(mpCurrentMultiKeyFrame, pKF2,
				vMatchedKeys1, vMatchedKeysRays1,
				vMatchedKeys2, vMatchedKeysRays2,
				vMatchedIndices);

			// Triangulate each match
			for (size_t ikp = 0, iendkp = vMatchedKeysRays1.size(); ikp < iendkp; ++ikp)
			{
				const int idx1 = vMatchedIndices[ikp].first;
				const int idx2 = vMatchedIndices[ikp].second;

				int camIdx1 = mpCurrentMultiKeyFrame->keypoint_to_cam.find(idx1)->second;
				int camIdx2 = vpNeighKFs[i]->keypoint_to_cam.find(idx2)->second;

				const cv::Vec3d &ray1 = vMatchedKeysRays1[ikp];
				const cv::Vec3d &ray2 = vMatchedKeysRays2[ikp];

				const cv::KeyPoint &kp1 = vMatchedKeys1[ikp];
				const cv::KeyPoint &kp2 = vMatchedKeys2[ikp];
				cv::Vec3d x3D(0.0, 0.0, 0.0);

				cv::Matx44d Tcw1 = mpCurrentMultiKeyFrame->camSystem.Get_MtMc(camIdx1);
				cv::Matx44d Tcw1inv = mpCurrentMultiKeyFrame->camSystem.Get_MtMc_inv(camIdx1);
				cv::Matx44d Tcw2 = vpNeighKFs[i]->camSystem.Get_MtMc(camIdx2);
				cv::Matx44d Tcw2inv = vpNeighKFs[i]->camSystem.Get_MtMc_inv(camIdx2);

				cv::Matx33d Rcw1 = Tcw1.get_minor<3, 3>(0, 0);
				cv::Vec3d tcw1 = cv::Vec3d(Tcw1(0, 3), Tcw1(1, 3), Tcw1(2, 3));
				cv::Matx33d Rwc1 = Rcw1.t();
				cv::Matx33d Rcw2 = Tcw2.get_minor<3, 3>(0, 0);
				cv::Vec3d tcw2 = cv::Vec3d(Tcw1(0, 3), Tcw1(1, 3), Tcw1(2, 3));
				cv::Matx33d Rwc2 = Rcw2.t();

				cv::Vec3d rayRot1 = Rcw1 * ray1; // transposed rotation world2cam
				cv::Vec3d rayRot2 = Rcw2 * ray2;

				const double cosParallax = rayRot1.dot(rayRot2) /
					(cv::norm(rayRot1) * cv::norm(rayRot2));

				if (cosParallax < 0 || cosParallax > cosThresh)
					continue;

				cv::Matx44d relOri = (Tcw1inv*Tcw2);
				cv::Vec3d t12 = cConverter::Hom2T(relOri);
				cv::Matx33d R12 = cConverter::Hom2R(relOri);

				x3D = triangulate_point(t12, R12, ray1, ray2);
				// now rotate the point to the world frame;
				cv::Vec4d x3D4(x3D(0), x3D(1), x3D(2), 1.0);
				x3D4 = Tcw1 * x3D4;
				x3D = cv::Vec3d(x3D4(0), x3D4(1), x3D4(2));

				// Check parallax between rays
				// Check triangulation in front of cameras
				// rotate point to camera frame

				//Check reprojection error in first keyframe
				double sigmaSquare1 = mpCurrentMultiKeyFrame->GetSigma2(kp1.octave);
				cv::Vec2d uv1(0.0, 0.0);
				bool zpos = mpCurrentMultiKeyFrame->camSystem.WorldToCamHom_fast(camIdx1, x3D4, uv1);
				if (zpos)
					continue;

				double errX1 = uv1(0) - cv::saturate_cast<double>(kp1.pt.x);
				double errY1 = uv1(1) - cv::saturate_cast<double>(kp1.pt.y);
				if (cv::sqrt(errX1*errX1 + errY1*errY1) > 4.0)
					continue;

				//Check reprojection error in second keyframe
				double sigmaSquare2 = pKF2->GetSigma2(kp2.octave);

				//double u2 = 0.0, v2 = 0.0;
				cv::Vec2d uv2(0.0, 0.0);
				zpos = pKF2->camSystem.WorldToCamHom_fast(camIdx2, x3D4, uv2);
				if (zpos)
					continue;

				double errX2 = uv2(0) - cv::saturate_cast<double>(kp2.pt.x);
				double errY2 = uv2(1) - cv::saturate_cast<double>(kp2.pt.y);
				if (cv::sqrt(errX2*errX2 + errY2*errY2) > 4.0)
					continue;

				//Check scale consistency
				cv::Vec3d normal1 = x3D - Ow1;
				double dist1 = cv::norm(normal1);

				cv::Vec3d normal2 = x3D - Ow2;
				double dist2 = cv::norm(normal2);
				double meanDist = ((dist1 + dist2) / 2.0);
				double ratio1 = pow(meanDist, 2) * pixelSize / (c_in_m1*baseline);

				//if (dist1 == 0 || dist2 == 0 || 
				//	dist1 > maxDIST || dist2 > maxDIST ||
				//	ratio1 >= minTriangQuali)
				//	continue;
				if (dist1 == 0 || dist2 == 0 ||
					dist1 > maxDIST || dist2 > maxDIST)
					continue;
				cMapPoint* pMP = new cMapPoint(x3D, mpCurrentMultiKeyFrame, mpMap);

				pMP->AddObservation(pKF2, idx2);
				pMP->AddObservation(mpCurrentMultiKeyFrame, idx1);

				mpCurrentMultiKeyFrame->AddMapPoint(pMP, idx1);
				pKF2->AddMapPoint(pMP, idx2);

				pMP->ComputeDistinctiveDescriptors(pKF2->HavingMasks());
				cv::Mat desc = pMP->GetDescriptor();
				pMP->UpdateCurrentDescriptor(desc);

				pMP->UpdateNormalAndDepth();

				mpMap->AddMapPoint(pMP);
				mlpRecentAddedMapPoints.push_back(pMP);
				++testCnt;
			}

		}
		std::cout << "______Finished Map Point Creation (" << testCnt << ")_______" << endl;
	}

	void cLocalMapping::SearchInNeighbors()
	{
		// Retrieve neighbor keyframes
		std::vector<cMultiKeyFrame*> vpNeighKFs =
			mpCurrentMultiKeyFrame->GetBestCovisibilityKeyFrames(15);
		std::vector<cMultiKeyFrame*> vpTargetKFs;
		for (std::vector<cMultiKeyFrame*>::iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end();
			vit != vend; ++vit)
		{
			cMultiKeyFrame* pKFi = *vit;
			if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentMultiKeyFrame->mnId)
				continue;
			vpTargetKFs.push_back(pKFi);
			pKFi->mnFuseTargetForKF = mpCurrentMultiKeyFrame->mnId;

			// Extend to some second neighbors
			std::vector<cMultiKeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
			for (std::vector<cMultiKeyFrame*>::iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end();
				vit2 != vend2; ++vit2)
			{
				cMultiKeyFrame* pKFi2 = *vit2;
				if (pKFi2->isBad() ||
					pKFi2->mnFuseTargetForKF == mpCurrentMultiKeyFrame->mnId ||
					pKFi2->mnId == mpCurrentMultiKeyFrame->mnId)
					continue;
				vpTargetKFs.push_back(pKFi2);
			}
		}

		// Search matches by projection from current KF in target KFs
		cORBmatcher matcher(0.8, checkOrientation, descDim, havingMasks);
		int nrFused = 0;
		std::vector<cMapPoint*> vpMapPointMatches = mpCurrentMultiKeyFrame->GetMapPointMatches();
		for (std::vector<cMultiKeyFrame*>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end();
			vit != vend; ++vit)
		{
			cMultiKeyFrame* pKFi = *vit;

			nrFused += matcher.Fuse(pKFi, mpCurrentMultiKeyFrame, vpMapPointMatches);
			//nrFused += matcher.Fuse(pKFi, vpMapPointMatches);
		}
		//// Search matches by projection from target KFs in current KF
		vector<cMapPoint*> vpFuseCandidates;
		vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

		for (vector<cMultiKeyFrame*>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end();
			vitKF != vendKF; ++vitKF)
		{
			cMultiKeyFrame* pKFi = *vitKF;

			vector<cMapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

			for (vector<cMapPoint*>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end();
				vitMP != vendMP; ++vitMP)
			{
				cMapPoint* pMP = *vitMP;
				if (!pMP)
					continue;
				if (pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentMultiKeyFrame->mnId)
					continue;
				pMP->mnFuseCandidateForKF = mpCurrentMultiKeyFrame->mnId;
				vpFuseCandidates.push_back(pMP);
			}
			//nrFused += matcher.Fuse(mpCurrentMultiKeyFrame, pKFi, vpFuseCandidates);
		}
		nrFused += matcher.Fuse(mpCurrentMultiKeyFrame, vpFuseCandidates);

		// Update connections in covisibility graph
		mpCurrentMultiKeyFrame->UpdateConnections();
	}

	void cLocalMapping::RequestStop()
	{
		std::unique_lock<std::mutex> lock(mMutexStop);
		mbStopRequested = true;
		std::unique_lock<std::mutex> lock2(mMutexNewKFs);
		mbAbortBA = true;
	}

	bool cLocalMapping::Stop()
	{
		std::unique_lock<mutex> lock(mMutexStop);
		if (mbStopRequested && !mbNotStop)
		{
			mbStopped = true;
			cout << "Local Mapping STOPPED" << endl;
			return true;
		}

		return false;
	}

	bool cLocalMapping::isStopped()
	{
		std::unique_lock<std::mutex> lock(mMutexStop);
		return mbStopped;
	}

	bool cLocalMapping::stopRequested()
	{
		std::unique_lock<std::mutex> lock(mMutexStop);
		return mbStopRequested;
	}

	void cLocalMapping::Release()
	{
		std::unique_lock<std::mutex> lock(mMutexStop);
		mbStopped = false;
		mbStopRequested = false;
		for (std::list<cMultiKeyFrame*>::iterator lit = mlNewMultiKeyFrames.begin(), lend = mlNewMultiKeyFrames.end();
			lit != lend; ++lit)
			delete *lit;
		mlNewMultiKeyFrames.clear();
	}

	bool cLocalMapping::AcceptMultiKeyFrames()
	{
		std::unique_lock<std::mutex> lock(mMutexAccept);
		return mbAcceptMultiKeyFrames;
	}

	void cLocalMapping::SetAcceptMultiKeyFrames(bool flag)
	{
		std::unique_lock<std::mutex> lock(mMutexAccept);
		mbAcceptMultiKeyFrames = flag;
	}

	void cLocalMapping::InterruptBA()
	{
		mbAbortBA = true;
	}

	void cLocalMapping::KeyFrameCulling()
	{
		// Check redundant keyframes (only local keyframes)
		// A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
		// in at least other 3 keyframes (in the same or finer scale)
		const int maxNrObs = 5;

		std::vector<cMultiKeyFrame*> vpLocalKeyFrames = mpCurrentMultiKeyFrame->GetVectorCovisibleKeyFrames();
		int minDist = 0;
		//for (auto vit : vpLocalKeyFrames)
		for (vector<cMultiKeyFrame*>::iterator vit =
			vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end(); vit != vend; ++vit)
		{
			cMultiKeyFrame* pKF = *vit;
			if (pKF->mnId == 0)
				continue;

			std::vector<cMapPoint*> vpMapPoints = pKF->GetMapPointMatches();

			int nRedundantObservations = 0;
			int nMPs = 0;
			for (size_t i = 0, iend = vpMapPoints.size(); i < iend; ++i)
			{
				cMapPoint* pMP = vpMapPoints[i];
				if (pMP)
				{
					if (!pMP->isBad())
					{
						++nMPs;
						// if a map point was observed more than 3 times we start to count
						// the occurances across covisibible keyframes
						if (pMP->Observations() > 3)
						{
							// scalelevel of overvation in current keyframe
							int scaleLevel = pKF->GetKeyPoint(i).octave;
							// get all observations
							std::map<cMultiKeyFrame*, std::vector<size_t>> observations =
								pMP->GetObservations();
							int nObs = 0;
							//for (auto mit : observations)
							for (map<cMultiKeyFrame*, std::vector<size_t>>::const_iterator mit =
								observations.begin(), mend = observations.end(); mit != mend; mit++)
							{
								cMultiKeyFrame* pKFi = mit->first;
								if (pKFi == pKF)
									continue;
								std::vector<size_t> localObs = mit->second;
								// a map point can be observed multiple times from one multikeyframe

								// just take the first, even if there are more
								if (mit->second.size() > 0)
								{
									//for (auto& l : localObs)
									//{
									int scaleLeveli = pKFi->GetKeyPoint(mit->second[0]).octave;
									if (scaleLeveli <= scaleLevel + 1)
										++nObs;
									if (nObs >= maxNrObs)
										break;
									//}
								}
								//}
							}
							if (nObs >= maxNrObs)
							{
								++nRedundantObservations;
							}
						}
					}
				}
			}

			if (nRedundantObservations > 0.9*nMPs)
				pKF->SetBadFlag();
		}

	}

	void cLocalMapping::RequestReset()
	{
		{
			std::unique_lock<std::mutex> lock(mMutexReset);
			mbResetRequested = true;
		}

		while (1)
		{
			{
				std::unique_lock<std::mutex> lock2(mMutexReset);
				if (!mbResetRequested)
					break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	void cLocalMapping::ResetIfRequested()
	{
		std::unique_lock<std::mutex> lock(mMutexReset);
		if (mbResetRequested)
		{
			mlNewMultiKeyFrames.clear();
			mlpRecentAddedMapPoints.clear();
			mbResetRequested = false;
		}
	}

	void cLocalMapping::SetMatcherProperties(int _descDim, bool _havingMasks)
	{
		descDim = _descDim;
		havingMasks = _havingMasks;
	}

	void cLocalMapping::RequestFinish()
	{
		std::unique_lock<std::mutex> lock(mMutexFinish);
		mbFinishRequested = true;
	}

	bool cLocalMapping::CheckFinish()
	{
		std::unique_lock<std::mutex>  lock(mMutexFinish);
		return mbFinishRequested;
	}

	void cLocalMapping::SetFinish()
	{
		std::unique_lock<std::mutex>  lock(mMutexFinish);
		mbFinished = true;
		std::unique_lock<std::mutex>  lock2(mMutexStop);
		mbStopped = true;
	}

	bool cLocalMapping::isFinished()
	{
		std::unique_lock<std::mutex>  lock(mMutexFinish);
		return mbFinished;
	}

	bool cLocalMapping::SetNotStop(bool flag)
	{
		unique_lock<mutex> lock(mMutexStop);

		if (flag && mbStopped)
			return false;

		mbNotStop = flag;

		return true;
	}


}



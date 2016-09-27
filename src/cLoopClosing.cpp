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

#include "cLoopClosing.h"
#include "cSim3Solver.h"
#include "cConverter.h"
#include "cOptimizer.h"
#include "cORBmatcher.h"



#include "g2o/types/types_seven_dof_expmap.h"

namespace MultiColSLAM
{
	cLoopClosing::cLoopClosing(cMap *pMap, cMultiKeyFrameDatabase *pDB, ORBVocabulary *pVoc) :
		mbResetRequested(false),
		mpMap(pMap),
		mpKeyFrameDB(pDB),
		mpORBVocabulary(pVoc),
		mLastLoopKFid(0),
		mbFinishRequested(false),
		mbFinished(false)
	{
		mnCovisibilityConsistencyTh = 3;
		mpMatchedKF = NULL;
	}

	void cLoopClosing::SetTracker(cTracking *pTracker)
	{
		mpTracker = pTracker;
	}

	void cLoopClosing::SetLocalMapper(cLocalMapping *pLocalMapper)
	{
		mpLocalMapper = pLocalMapper;
	}


	void cLoopClosing::Run()
	{

		while (1)
		{
			// Check if there are keyframes in the queue
			if (CheckNewKeyFrames())
			{
				// Detect loop candidates and check covisibility consistency
				if (DetectLoop())
				{
					// Compute similarity transformation [sR|t]
					if (ComputeSim3())
					{
						// Perform loop fusion and pose graph optimization
						cout << "========= Command Loop correction ============" << endl;
						CorrectLoop();
					}
				}
			}


			ResetIfRequested();

			if (CheckFinish())
				break;
			// for display purposes
			vector<cMultiKeyFrame*> vKeyFrames = mpMap->GetAllKeyFrames();
			for (size_t i = 0; i < vKeyFrames.size(); ++i)
				vKeyFrames[i]->SetLoopCandidate(false);

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		SetFinish();
	}

	void cLoopClosing::InsertKeyFrame(cMultiKeyFrame *pKF)
	{
		std::unique_lock<std::mutex> lock(mMutexLoopQueue);
		if (pKF->mnId != 0)
			mlpLoopKeyFrameQueue.push_back(pKF);
	}

	bool cLoopClosing::CheckNewKeyFrames()
	{
		std::unique_lock<std::mutex>  lock(mMutexLoopQueue);
		return (!mlpLoopKeyFrameQueue.empty());
	}

	bool cLoopClosing::DetectLoop()
	{
		//cout << "in loop detect" << endl;
		{
			std::unique_lock<std::mutex>  lock(mMutexLoopQueue);
			mpCurrentKF = mlpLoopKeyFrameQueue.front();
			mlpLoopKeyFrameQueue.pop_front();
			// Avoid that a keyframe can be erased while it is being process by this thread
			mpCurrentKF->SetNotErase();
		}

		//   //If the map contains less than 10 KF or less than 10KF have passed from last loop detection
		if (mpCurrentKF->mnId < mLastLoopKFid + 10)
		{
			mpKeyFrameDB->add(mpCurrentKF);
			mpCurrentKF->SetErase();
			return false;
		}

		// Compute reference BoW similarity score
		// This is the lowest score to a connected keyframe in the covisibility graph
		// We will impose loop candidates to have a higher similarity than this
		std::vector<cMultiKeyFrame*> vpConnectedKeyFrames =
			mpCurrentKF->GetVectorCovisibleKeyFrames();
		DBoW2::BowVector CurrentBowVec = mpCurrentKF->GetBowVector();
		double minScore = 1.0;
		for (size_t i = 0; i < vpConnectedKeyFrames.size(); ++i)
		{
			cMultiKeyFrame* pKF = vpConnectedKeyFrames[i];
			if (pKF->isBad())
				continue;

			DBoW2::BowVector BowVec = pKF->GetBowVector();

			double score = mpORBVocabulary->score(CurrentBowVec, BowVec);

			if (score < minScore)
				minScore = score;
		}
		//cout << "minScore: " << minScore << endl;
		// Query the database imposing the minimum score
		std::vector<cMultiKeyFrame*> vpCandidateKFs =
			mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

		// If there are no loop candidates, just add new keyframe and return false
		if (vpCandidateKFs.empty())
		{
			mpKeyFrameDB->add(mpCurrentKF);
			mvConsistentGroups.clear();
			mpCurrentKF->SetErase();
			return false;
		}
		cout << "======== HAVING A LOOP CANDIDATE ========" << endl;
		// For each loop candidate check consistency with previous loop candidates
		// Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
		// A group is consistent with a previous group if they share at least a keyframe
		// We must detect a consistent loop in several consecutive keyframe to accept it
		mvpEnoughConsistentCandidates.clear();

		std::vector<ConsistentGroup> vCurrentConsistentGroups;
		std::vector<bool> vbConsistentGroup(mvConsistentGroups.size(), false);
		for (size_t i = 0, iend = vpCandidateKFs.size(); i < iend; ++i)
		{
			cMultiKeyFrame* pCandidateKF = vpCandidateKFs[i];

			std::set<cMultiKeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
			spCandidateGroup.insert(pCandidateKF);

			bool bEnoughConsistent = false;
			bool bConsistentForSomeGroup = false;
			for (size_t iG = 0, iendG = mvConsistentGroups.size(); iG < iendG; ++iG)
			{
				std::set<cMultiKeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

				bool bConsistent = false;
				for (std::set<cMultiKeyFrame*>::iterator sit = spCandidateGroup.begin(), send = spCandidateGroup.end(); sit != send; sit++)
				{
					if (sPreviousGroup.count(*sit))
					{
						bConsistent = true;
						bConsistentForSomeGroup = true;
						break;
					}
				}

				if (bConsistent)
				{
					int nPreviousConsistency = mvConsistentGroups[iG].second;
					int nCurrentConsistency = nPreviousConsistency + 1;
					if (!vbConsistentGroup[iG])
					{
						ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
						vCurrentConsistentGroups.push_back(cg);
						vbConsistentGroup[iG] = true; //this avoid to include the same group more than once
					}
					if (nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent)
					{
						mvpEnoughConsistentCandidates.push_back(pCandidateKF);
						bEnoughConsistent = true; //this avoid to insert the same candidate more than once
					}
				}
			}

			// If the group is not consistent with any previous group insert with consistency counter set to zero
			if (!bConsistentForSomeGroup)
			{
				ConsistentGroup cg = make_pair(spCandidateGroup, 0);
				vCurrentConsistentGroups.push_back(cg);
			}
		}

		// Update Covisibility Consistent Groups
		mvConsistentGroups = vCurrentConsistentGroups;


		// Add Current Keyframe to database
		mpKeyFrameDB->add(mpCurrentKF);

		if (mvpEnoughConsistentCandidates.empty())
		{
			cout << "NOT ENOUGH CONISTENT CANDIDATES" << endl;
			mpCurrentKF->SetErase();
			return false;
		}
		else
		{
			cout << "ENOUGH CONISTENT CANDIDATES" << endl;
			return true;
		}

		mpCurrentKF->SetErase();
		return false;
	}

	bool cLoopClosing::ComputeSim3()
	{
		// For each consistent loop candidate we try to compute a Sim3
		const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

		// We compute first ORB matches for each candidate
		// If enough matches are found, we setup a Sim3Solver
		cORBmatcher matcher(0.9, checkOrientation, descDim, havingMasks);

		std::vector<cSim3Solver*> vpSim3Solvers;
		vpSim3Solvers.resize(nInitialCandidates);

		std::vector<std::vector<cMapPoint*> > vvpMapPointMatches;
		vvpMapPointMatches.resize(nInitialCandidates);

		std::vector<bool> vbDiscarded;
		vbDiscarded.resize(nInitialCandidates);

		int nCandidates = 0; //candidates with enough matches
		cout << "======== Computing SIM3 ========" << endl;
		//cout << "nInitialCandidates: " << nInitialCandidates << endl;
		for (int i = 0; i < nInitialCandidates; ++i)
		{
			cMultiKeyFrame* pKF = mvpEnoughConsistentCandidates[i];

			// avoid that local mapping erase it while it is being processed in this thread
			pKF->SetNotErase();

			if (pKF->isBad())
			{
				vbDiscarded[i] = true;
				continue;
			}
			pKF->SetLoopCandidate(true);
			int nmatches = matcher.SearchByBoW(mpCurrentKF, pKF, vvpMapPointMatches[i]);
			if (nmatches < 15)
			{
				cout << "======== NOT ENOUGH MATCHES (" << nmatches << ") ======== " << endl;
				vbDiscarded[i] = true;
				continue;
			}
			else
			{
				cout << "======== RANSAC (" << nmatches << ") ========" << endl;
				cSim3Solver* pSolver = new cSim3Solver(mpCurrentKF,
					pKF, vvpMapPointMatches[i], &mpCurrentKF->camSystem);
				pSolver->SetRansacParameters(0.98, 15, 300);
				vpSim3Solvers[i] = pSolver;
			}

			++nCandidates;
		}

		bool bMatch = false;

		// Perform alternatively RANSAC iterations for each candidate
		// until one is successful or all fail
		while (nCandidates > 0 && !bMatch)
		{
			for (int i = 0; i < nInitialCandidates; ++i)
			{
				if (vbDiscarded[i])
					continue;

				cMultiKeyFrame* pKF = mvpEnoughConsistentCandidates[i];

				// Perform 5 Ransac Iterations
				std::vector<bool> vbInliers;
				int nInliers;
				bool bNoMore;

				cSim3Solver* pSolver = vpSim3Solvers[i];
				cv::Matx44d Scm = cv::Matx44d::eye();
				bool success = pSolver->iterate(50, bNoMore, vbInliers, nInliers, Scm);
				// If Ransac reaches max. iterations discard keyframe
				if (bNoMore)
				{
					vbDiscarded[i] = true;
					nCandidates--;
				}

				// If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
				if (success)
				{
					std::vector<cMapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(),
						static_cast<cMapPoint*>(NULL));
					for (size_t j = 0, jend = vbInliers.size(); j < jend; ++j)
					{
						if (vbInliers[j])
							vpMapPointMatches[j] = vvpMapPointMatches[i][j];
					}

					cv::Matx33d R = pSolver->GetEstimatedRotation();
					cv::Vec3d t = pSolver->GetEstimatedTranslation();
					const double s = pSolver->GetEstimatedScale();
					//cout << "scale: " << s <<endl<< " R: " << R;
					matcher.SearchBySim3(mpCurrentKF, pKF, vpMapPointMatches, s, R, t, 10);

					g2o::Sim3 gScm(cConverter::toMatrix3d(R), cConverter::toVector3d(t), s);
					const int nInliers = cOptimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm);
					cout << "inliers horn after sim3 optimization: " << nInliers << endl;
					// If optimization is succesful stop ransac and continue
					if (nInliers >= 20)
					{
						bMatch = true;
						mpMatchedKF = pKF;
						cv::Matx44d pKf_inv = pKF->GetPoseInverse(); // pose in camera frame
						g2o::Sim3 gSmw(cConverter::toMatrix3d(cConverter::Hom2R(pKf_inv)),
							cConverter::toVector3d(cConverter::Hom2T(pKf_inv)), 1.0);
						// relative transformation between candidate and optimized sim3
						// mg2oScw is the new transformation matrix M_t for the candidate keyframe
						mg2oScw = gScm*gSmw;

						mScw = cConverter::toCvMat(mg2oScw);

						mvpCurrentMatchedPoints = vpMapPointMatches;
						break;
					}
				}
			}
		}

		if (!bMatch)
		{
			for (int i = 0; i < nInitialCandidates; ++i)
				mvpEnoughConsistentCandidates[i]->SetErase();
			mpCurrentKF->SetErase();
			return false;
		}

		// Retrieve MapPoints seen in Loop Keyframe and neighbors
		std::vector<cMultiKeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
		vpLoopConnectedKFs.push_back(mpMatchedKF);
		mvpLoopMapPoints.clear();
		for (std::vector<cMultiKeyFrame*>::iterator vit = vpLoopConnectedKFs.begin();
			vit != vpLoopConnectedKFs.end(); ++vit)
		{
			cMultiKeyFrame* pKF = *vit;
			std::vector<cMapPoint*> vpMapPoints = pKF->GetMapPointMatches();
			for (size_t i = 0, iend = vpMapPoints.size(); i < iend; ++i)
			{
				cMapPoint* pMP = vpMapPoints[i];
				if (pMP)
				{
					if (!pMP->isBad() && pMP->mnLoopPointForKF != mpCurrentKF->mnId)
					{
						mvpLoopMapPoints.push_back(pMP);
						pMP->mnLoopPointForKF = mpCurrentKF->mnId;
					}
				}
			}
		}

		// Find more matches projecting with the computed Sim3
		int matcherFound = matcher.SearchByProjection(mpCurrentKF,
			mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints, 10);
		cout << "matcherFound: " << matcherFound << endl;
		// If enough matches accept Loop
		int nTotalMatches = 0;
		for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); ++i)
			if (mvpCurrentMatchedPoints[i])
				nTotalMatches++;

		if (nTotalMatches >= 20)
		{
			cout << "=== Sim3 would accept this candidate (" << nTotalMatches << ") ===" << endl;
			for (int i = 0; i < nInitialCandidates; ++i)
				if (mvpEnoughConsistentCandidates[i] != mpMatchedKF)
					mvpEnoughConsistentCandidates[i]->SetErase();
			return true;
		}
		else
		{
			for (int i = 0; i < nInitialCandidates; ++i)
				mvpEnoughConsistentCandidates[i]->SetErase();
			mpCurrentKF->SetErase();
			return false;
		}

		return false;
	}

	void cLoopClosing::CorrectLoop()
	{

		cout << "======= IN LOOP CORRECTION ========" << endl;
		// Send a stop signal to Local Mapping
		// Avoid new MKFs are inserted while correcting the loop
		mpLocalMapper->RequestStop();

		// Wait until Local Mapping has effectively stopped
		while (!mpLocalMapper->isStopped())
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// Ensure current MKF is updated
		mpCurrentKF->UpdateConnections();

		// Retrive MKFs connected to the current MKF and compute corrected Sim3 pose by propagation
		mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
		mvpCurrentConnectedKFs.push_back(mpCurrentKF);

		KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
		CorrectedSim3[mpCurrentKF] = mg2oScw;
		cv::Matx44d Twc = mpCurrentKF->GetPose(); // pose in the camera frame


		//std::unique_lock<std::mutex>  lock(mpMap->mMutexMapUpdate);

		for (std::vector<cMultiKeyFrame*>::iterator vit =
			mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; ++vit)
		{
			cMultiKeyFrame* pKFi = *vit;

			cv::Matx44d Tiw = pKFi->GetPoseInverse(); // pose in camera frame

			if (pKFi != mpCurrentKF)
			{
				// evtl invertieren
				cv::Matx44d Tic = Tiw*Twc; // relative orientation between MKFs
				cv::Matx33d Ric = cConverter::Hom2R(Tic);
				cv::Vec3d tic = cConverter::Hom2T(Tic);
				g2o::Sim3 g2oSic(cConverter::toMatrix3d(Ric),
					cConverter::toVector3d(tic), 1.0);
				g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
				// Pose corrected with the Sim3 of the loop closure
				CorrectedSim3[pKFi] = g2oCorrectedSiw;
			}

			cv::Matx33d Riw = cConverter::Hom2R(Tiw);
			cv::Vec3d tiw = cConverter::Hom2T(Tiw);
			g2o::Sim3 g2oSiw(cConverter::toMatrix3d(Riw), cConverter::toVector3d(tiw), 1.0);
			//Pose without correction
			NonCorrectedSim3[pKFi] = g2oSiw;
		}

		// Correct all MapPoints observed by current keyframe and neighbors, so that they align with the other side of the loop
		for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++)
		{
			cMultiKeyFrame* pKFi = mit->first;
			g2o::Sim3 g2oCorrectedSiw = mit->second;
			g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

			g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

			std::vector<cMapPoint*> vpMPsi = pKFi->GetMapPointMatches();
			for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; ++iMP)
			{
				cMapPoint* pMPi = vpMPsi[iMP];
				if (!pMPi)
					continue;
				if (pMPi->isBad())
					continue;
				if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId)
					continue;

				// Project with non-corrected pose and project back with corrected pose
				cv::Vec3d P3Dw = pMPi->GetWorldPos();
				Eigen::Vector3d eigP3Dw = cConverter::toVector3d(P3Dw);
				Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

				cv::Vec3d cvCorrectedP3Dw = cConverter::toCvVec3d(eigCorrectedP3Dw);
				pMPi->SetWorldPos(cvCorrectedP3Dw);
				pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
				pMPi->mnCorrectedReference = pKFi->mnId;
				pMPi->UpdateNormalAndDepth();
			}

			// Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
			double scale = g2oCorrectedSiw.scale();
			Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
			Eigen::Vector3d eigt = g2oCorrectedSiw.translation() / scale;

			cv::Matx44d correctedTiw = cConverter::toCvSE3(eigR, eigt);
			// inverse!
			pKFi->SetPose(cConverter::invMat(correctedTiw));
			// Make sure connections are updated
			pKFi->UpdateConnections();
		}

		// Start Loop Fusion
		// Update matched map points and replace if duplicated
		for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); ++i)
		{
			if (mvpCurrentMatchedPoints[i])
			{
				cMapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
				cMapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
				if (pCurMP)
					pCurMP->Replace(pLoopMP);
				else
				{
					mpCurrentKF->AddMapPoint(pLoopMP, i);
					pLoopMP->AddObservation(mpCurrentKF, i);
					pLoopMP->ComputeDistinctiveDescriptors();
				}
			}
		}

		// Project MapPoints observed in the neighborhood of the loop keyframe
		// into the current keyframe and neighbors using corrected poses.
		// Fuse duplications.
		SearchAndFuse(CorrectedSim3);


		// After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
		std::map<cMultiKeyFrame*, std::set<cMultiKeyFrame*> > LoopConnections;

		for (std::vector<cMultiKeyFrame*>::iterator vit = mvpCurrentConnectedKFs.begin(),
			vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
		{
			cMultiKeyFrame* pKFi = *vit;
			std::vector<cMultiKeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

			// Update connections. Detect new links.
			pKFi->UpdateConnections();
			LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
			for (std::vector<cMultiKeyFrame*>::iterator vit_prev = vpPreviousNeighbors.begin(),
				vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++)
			{
				LoopConnections[pKFi].erase(*vit_prev);
			}
			for (std::vector<cMultiKeyFrame*>::iterator vit2 = mvpCurrentConnectedKFs.begin(),
				vend2 = mvpCurrentConnectedKFs.end(); vit2 != vend2; vit2++)
			{
				LoopConnections[pKFi].erase(*vit2);
			}
		}

		mpTracker->ForceRelocalisation();
		cout << "======= Starting Essential Graph Optimization ========" << endl;
		cOptimizer::OptimizeEssentialGraph(mpMap,
			mpMatchedKF, mpCurrentKF,
			mg2oScw,
			NonCorrectedSim3, CorrectedSim3,
			LoopConnections);

		//Add edge
		mpMatchedKF->AddLoopEdge(mpCurrentKF);
		mpCurrentKF->AddLoopEdge(mpMatchedKF);

		// Loop closed. Release Local Mapping.
		mpLocalMapper->Release();
		mpMap->SetFlagAfterBA();

		mLastLoopKFid = mpCurrentKF->mnId;

		cout << "Loop Closed!" << endl;
		mpMap->SetFlagAfterBA();
	}

	void cLoopClosing::SearchAndFuse(KeyFrameAndPose &CorrectedPosesMap)
	{
		cORBmatcher matcher(0.8, checkOrientation, descDim, havingMasks);
		for (KeyFrameAndPose::iterator mit =
			CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end(); mit != mend; ++mit)
		{
			cMultiKeyFrame* pKF = mit->first;

			g2o::Sim3 g2oScw = mit->second;
			cv::Matx44d cvScw = cConverter::toCvMat(g2oScw);

			matcher.Fuse(pKF, cvScw, mvpLoopMapPoints, 4);
		}
	}


	void cLoopClosing::RequestReset()
	{
		{
			std::unique_lock<std::mutex>  lock(mMutexReset);
			mbResetRequested = true;
		}

		while (1)
		{
			{
				std::unique_lock<std::mutex>  lock2(mMutexReset);
				if (!mbResetRequested)
					break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}

	void cLoopClosing::ResetIfRequested()
	{
		std::unique_lock<std::mutex> lock(mMutexReset);
		if (mbResetRequested)
		{
			mlpLoopKeyFrameQueue.clear();
			mLastLoopKFid = 0;
			mbResetRequested = false;
		}
	}

	void cLoopClosing::SetMatcherProperties(int _descDim, bool _havingMasks)
	{
		descDim = _descDim;
		havingMasks = _havingMasks;
	}


	void cLoopClosing::RequestFinish()
	{
		std::unique_lock<std::mutex> lock(mMutexFinish);
		mbFinishRequested = true;
	}

	bool cLoopClosing::CheckFinish()
	{
		std::unique_lock<std::mutex> lock(mMutexFinish);
		return mbFinishRequested;
	}

	void cLoopClosing::SetFinish()
	{
		std::unique_lock<std::mutex> lock(mMutexFinish);
		mbFinished = true;
	}

	bool cLoopClosing::isFinished()
	{
		std::unique_lock<std::mutex> lock(mMutexFinish);
		return mbFinished;
	}
}

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

#include "cOptimizer.h"


#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"

#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/types/types_seven_dof_expmap.h"

#include "g2o/core/jacobian_workspace.h"
#include "g2o/stuff/macros.h"

#include <Eigen/StdVector>

#include "cConverter.h"

#include "g2o_MultiCol_vertices_edges.h"
#include "g2o_MultiCol_sim3_expmap.h"

namespace MultiColSLAM
{
	double cOptimizer::stdRecon = 2.0;
	double cOptimizer::stdPose = 2.0;

	void cOptimizer::GlobalBundleAdjustment(cMap* pMap,
		bool poseOnly,
		int nIterations,
		bool *pbStopFlag)
	{
		std::vector<cMultiKeyFrame*> vpKFs = pMap->GetAllKeyFrames();
		std::vector<cMapPoint*> vpMP = pMap->GetAllMapPoints();
		BundleAdjustment(vpKFs, vpMP,
			poseOnly,
			nIterations, pbStopFlag);
	}

	void cOptimizer::BundleAdjustment(const std::vector<cMultiKeyFrame*> &vpKFs,
		const std::vector<cMapPoint*> &vpMP,
		bool poseOnly,
		int nIterations,
		bool *pbStopFlag)
	{
		int redundancy = 1;
		g2o::SparseOptimizer optimizer;
		g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

		linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

		g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

		g2o::OptimizationAlgorithmLevenberg* solver =
			new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		optimizer.setAlgorithm(solver);
		optimizer.setVerbose(false);

		g2o::SparseOptimizerTerminateAction* terminateAction = 0;
		terminateAction = new g2o::SparseOptimizerTerminateAction;
		terminateAction->setGainThreshold(1e-6);
		terminateAction->setMaxIterations(15);
		optimizer.addPostIterationAction(terminateAction);

		if (pbStopFlag)
			optimizer.setForceStopFlag(pbStopFlag);

		//long unsigned int maxKFid = 0;

		unsigned long currVertexIdx = 0;
		unsigned long maxKF = 0;
		unsigned long maxKFid = 0;
		unsigned long maxMcid = 0;
		unsigned long maxIOid = 0;

		// SET Mt KEYFRAME VERTICES
		for (size_t i = 0, iend = vpKFs.size(); i < iend; ++i)
		{
			cMultiKeyFrame* pKF = vpKFs[i];
			if (pKF->isBad())
				continue;

			VertexMt_cayley * vSE3 = new VertexMt_cayley();
			vSE3->setEstimate(hom2cayley(pKF->GetPose()));
			vSE3->setId(pKF->mnId);
			if (pKF->mnId == 0)
				vSE3->setFixed(true);
			else
			{
				vSE3->setFixed(false);
				redundancy -= vSE3->dimension();
			}
			optimizer.addVertex(vSE3);
			if (pKF->mnId > maxKFid)
				maxKF = pKF->mnId;

		}
		// because the keyframe ids are not continuous
		currVertexIdx = maxKF + 1;

		const int nrCams = vpKFs[0]->camSystem.GetNrCams();
		// SET Mc VERTICES 
		for (int c = 0; c < nrCams; ++c)
		{
			VertexMc_cayley* vMc = new VertexMc_cayley();
			vMc->setEstimate(vpKFs[0]->camSystem.Get_M_c_min(c));
			vMc->setId(currVertexIdx);
			vMc->setFixed(true);
			vMc->setMarginalized(false);
			optimizer.addVertex(vMc);
			currVertexIdx++;
		}
		maxMcid = currVertexIdx;
		// SET IO VERTICES 
		for (int c = 0; c < nrCams; ++c)
		{
			VertexOmniCameraParameters* vIO =
				new VertexOmniCameraParameters(vpKFs[0]->camSystem.GetCamModelObj(c));
			vIO->setEstimate(vpKFs[0]->camSystem.GetCamModelObj(c).toVector());
			vIO->setId(currVertexIdx);
			vIO->setFixed(true);
			vIO->setMarginalized(false);
			optimizer.addVertex(vIO);
			currVertexIdx++;
		}
		maxIOid = currVertexIdx; // to be able to recover the corresponding vertex

		const double thHuber = sqrt(5.991);

		std::unordered_map<int, int> mapPointId_to_cont_g2oId;
		// SET MAP POINT VERTICES
		for (size_t i = 0, iend = vpMP.size(); i < iend; ++i)
		{
			cMapPoint* pMP = vpMP[i];
			if (pMP->isBad())
				continue;

			VertexPointXYZ* vPoint = new VertexPointXYZ();
			vPoint->setEstimate(pMP->GetWorldPos());

			vPoint->setId(currVertexIdx);
			// for later save the map from the continous to map point idxs
			mapPointId_to_cont_g2oId[pMP->mnId] = currVertexIdx;

			vPoint->setFixed(poseOnly);
			if (!poseOnly)
				redundancy -= vPoint->dimension();
			vPoint->setMarginalized(true);
			optimizer.addVertex(vPoint);

			std::map<cMultiKeyFrame*, std::vector<size_t> > observations = pMP->GetObservations();

			// SET EDGES
			// in contrast to ORB_SLAM an additional layer of measurements has to be introduced
			// we also need to search for the camera in which the observation was made
			for (std::map<cMultiKeyFrame*, std::vector<size_t> >::iterator mit =
				observations.begin(), mend = observations.end();
				mit != mend; ++mit)
			{
				cMultiKeyFrame* pKF = mit->first;

				if (pKF->isBad())
					continue;
				// get all image points for this keyframe corresponding to one map point
				std::vector<size_t>& imagePoints = mit->second;
				// add all observations
				for (auto obsIdx : imagePoints)
				{
					int cam = pKF->keypoint_to_cam.find(obsIdx)->second;

					cv::KeyPoint kpUn = pKF->GetKeyPoint(obsIdx);
					cv::Vec2d obs(kpUn.pt.x, kpUn.pt.y);

					EdgeProjectXYZ2MCS* e = new EdgeProjectXYZ2MCS();
					e->setMeasurement(Eigen::Vector2d(kpUn.pt.x, kpUn.pt.y));
					e->setInformation(Eigen::Matrix2d::Identity());
					redundancy += 2;
					// Mt
					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
						optimizer.vertex(pKF->mnId)));
					// 3D point
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
						optimizer.vertex(currVertexIdx)));
					// Mc
					e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
						optimizer.vertex(maxMcid - nrCams + cam)));
					// IO
					e->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
						optimizer.vertex(maxIOid - nrCams + cam)));
					g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
					rk->setDelta(thHuber);
					e->setRobustKernel(rk);


					optimizer.addEdge(e);
					e->computeError();
				}

			}
			currVertexIdx++;
		}

		// Optimize!
		optimizer.initializeOptimization();
		optimizer.optimize(15);

		// Recover optimized data
		//Keyframes
		for (size_t i = 0, iend = vpKFs.size(); i < iend; ++i)
		{
			cMultiKeyFrame* pKF = vpKFs[i];
			VertexMt_cayley* vSE3 = static_cast<VertexMt_cayley*>(optimizer.vertex(pKF->mnId));
			cv::Matx61d mincayley = vSE3->estimate();
			pKF->SetPose(cayley2hom(mincayley));
			//pKF->SetPose(rodrigues2hom(mincayley));
		}

		//Points
		for (size_t i = 0, iend = vpMP.size(); i < iend; ++i)
		{
			cMapPoint* pMP = vpMP[i];
			int contId = mapPointId_to_cont_g2oId.find(pMP->mnId)->second;
			VertexPointXYZ* vPoint = static_cast<VertexPointXYZ*>(optimizer.vertex(contId));
			pMP->SetWorldPos(vPoint->estimate());
			pMP->UpdateNormalAndDepth();
		}

	}

	int cOptimizer::PoseOptimization(cMultiFrame *pFrame,
		double& inliers,
		const double& huberMultiplier)
	{
#ifdef VERBOSE
		cout << " ---OPTIMIZING POSE--- " << endl;
#endif
		int redundancy = 1; // m-n+1
		std::chrono::steady_clock::time_point begin;
		std::chrono::steady_clock::time_point end;

		begin = std::chrono::steady_clock::now();
		g2o::SparseOptimizer optimizer;
		g2o::BlockSolverX::LinearSolverType * linearSolver;

		linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

		g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

		g2o::OptimizationAlgorithmLevenberg* solver =
			new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

		optimizer.setAlgorithm(solver);
		optimizer.setVerbose(false);
		g2o::SparseOptimizerTerminateAction* terminateAction = 0;
		terminateAction = new g2o::SparseOptimizerTerminateAction;
		terminateAction->setGainThreshold(1e-6);
		terminateAction->setMaxIterations(15);
		optimizer.addPostIterationAction(terminateAction);

		// SET FRAME VERTEX
		VertexMt_cayley* vSE3 = new VertexMt_cayley();
		vSE3->setEstimate(pFrame->GetPoseMin());
		vSE3->setId(0); // index is 0
		vSE3->setFixed(false);
		optimizer.addVertex(vSE3);
		redundancy -= vSE3->dimension();

		unsigned long currVertexIdx = 1;
		unsigned long maxMcid = 0;
		unsigned long maxIOid = 0;

		// because the keyframe ids are not continous

		const int nrCams = pFrame->camSystem.GetNrCams();
		// SET Mc VERTICES 
		for (int c = 0; c < nrCams; ++c)
		{
			VertexMc_cayley* vMc = new VertexMc_cayley();
			vMc->setEstimate(pFrame->camSystem.Get_M_c_min(c));
			vMc->setId(currVertexIdx);
			vMc->setFixed(true);
			optimizer.addVertex(vMc);

			currVertexIdx++;
		}
		maxMcid = currVertexIdx;

		// SET IO VERTICES 
		for (int c = 0; c < nrCams; ++c)
		{
			VertexOmniCameraParameters* vIO = new
				VertexOmniCameraParameters(pFrame->camSystem.GetCamModelObj(c));
			vIO->setEstimate(pFrame->camSystem.GetCamModelObj(c).toVector());
			vIO->setId(currVertexIdx);
			vIO->setFixed(true);
			optimizer.addVertex(vIO);

			currVertexIdx++;
		}
		maxIOid = currVertexIdx; // to be able to recover the corresponding vertex

		const double thHuber = 1.345 * huberMultiplier;

		std::unordered_map<int, int> mapPointId_to_cont_g2oId;
		// SET MAP POINT VERTICES
		std::vector<EdgeProjectXYZ2MCS*> vpEdges;
		std::vector<VertexPointXYZ*> vVertices;
		std::vector<double> vInvSigmas2;
		std::vector<size_t> vnIndexEdge;

		const int N = pFrame->mvpMapPoints.size();
		vpEdges.reserve(N);
		vVertices.reserve(N);
		vInvSigmas2.reserve(N);
		vnIndexEdge.reserve(N);
		std::unordered_map<int, int> mapPt_2_obs_idx;
		int pointIdx = 0;
		int nInitialCorrespondences = 0;
		for (int i = 0; i < N; ++i)
		{
			cMapPoint* pMP = pFrame->mvpMapPoints[i];
			pFrame->mvbOutlier[i] = false;

			if (pMP)
			{
				if (mapPt_2_obs_idx.count(pMP->mnId) <= 0)
				{
					mapPt_2_obs_idx[pMP->mnId] = i;

					VertexPointXYZ* vPoint = new VertexPointXYZ();
					vPoint->setEstimate(pMP->GetWorldPos());
					vPoint->setId(currVertexIdx);
					vPoint->setFixed(true);

					optimizer.addVertex(vPoint);
					pointIdx = currVertexIdx;
					mapPointId_to_cont_g2oId[pMP->mnId] = currVertexIdx;
					currVertexIdx++;

				}
				else
					pointIdx = mapPointId_to_cont_g2oId.find(pMP->mnId)->second;

				int cam = pFrame->keypoint_to_cam.find(i)->second;

				cv::KeyPoint kpUn = pFrame->mvKeys[i];

				EdgeProjectXYZ2MCS* e = new EdgeProjectXYZ2MCS();
				e->setMeasurement(Eigen::Vector2d(kpUn.pt.x, kpUn.pt.y));
				const double invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
				e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
				redundancy += 2;
				// Mt
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
					optimizer.vertex(0)));
				// 3D point
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
					optimizer.vertex(pointIdx)));
				// Mc
				e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
					optimizer.vertex(maxMcid - nrCams + cam)));
				// IO
				e->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
					optimizer.vertex(maxIOid - nrCams + cam)));
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
				rk->setDelta(thHuber);
				e->setRobustKernel(rk);

				optimizer.addEdge(e);
				++nInitialCorrespondences;
				vpEdges.push_back(e);
				vnIndexEdge.push_back(i);
			}
		}

		// Optimize!
		optimizer.initializeOptimization();
		optimizer.optimize(10);

		double thHuber2 = thHuber*thHuber;
		int nBad = 0;
		// set outlier measurements
		for (size_t i = 0, iend = vpEdges.size(); i < iend; ++i)
		{
			EdgeProjectXYZ2MCS* e = vpEdges[i];

			const size_t idx = vnIndexEdge[i];

			if (e->chi2() > thHuber2)
			{
				vpEdges[i] = NULL;
				pFrame->mvbOutlier[idx] = true;
				e->setLevel(1);
				++nBad;
			}
			else
				pFrame->mvbOutlier[idx] = false;
		}
		// Optimize!
		optimizer.initializeOptimization(0);
		int result = optimizer.optimize(10);

		// set outlier measurements
		for (size_t i = 0, iend = vpEdges.size(); i < iend; ++i)
		{
			EdgeProjectXYZ2MCS* e = vpEdges[i];
			if (!vpEdges[i])
				continue;
			const size_t idx = vnIndexEdge[i];

			if (e->chi2() > thHuber2)
			{
				pFrame->mvbOutlier[idx] = true;
				++nBad;
			}
			else
				pFrame->mvbOutlier[idx] = false;
		}

		// Recover optimized pose and return number of inliers
		VertexMt_cayley* mincayley = static_cast<VertexMt_cayley*>(optimizer.vertex(0));
		cv::Matx61d SE3quat_recov = mincayley->estimate();
		pFrame->camSystem.Set_M_t_from_min(SE3quat_recov);

		if (nInitialCorrespondences > 0)
			inliers = static_cast<double>(nBad) / static_cast<double>(nInitialCorrespondences);
		else inliers = 0;

		return nInitialCorrespondences - nBad;
	}

	std::list<cMultiKeyFrame*> cOptimizer::LocalBundleAdjustment(
		cMultiKeyFrame *pKF,
		cMap* pMap,
		int nrIters,
		bool getCovMats,
		bool* pbStopFlag)
	{
#ifdef VERBOSE
		cout << " ---OPTIMIZING LOCAL MAP--- " << endl;
#endif
		int numUnknowns = 0;
		int numObservationsTotal = 0;
		// Local KeyFrames: First Breath Search from Current Keyframe
		std::list<cMultiKeyFrame*> lLocalKeyFrames;

		lLocalKeyFrames.push_back(pKF);
		pKF->mnBALocalForKF = pKF->mnId;

		std::vector<cMultiKeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
		//std::vector<cMultiKeyFrame*> vNeighKFs = pKF->GetBestCovisibilityKeyFrames(10);
		for (int i = 0, iend = vNeighKFs.size(); i < iend; ++i)
		{
			cMultiKeyFrame* pKFi = vNeighKFs[i];
			pKFi->mnBALocalForKF = pKF->mnId;
			if (!pKFi->isBad())
				lLocalKeyFrames.push_back(pKFi);
		}

		if (lLocalKeyFrames.size() <= 1)
			return std::list<cMultiKeyFrame*>();
		// Local MapPoints seen in Local KeyFrames
		std::list<cMapPoint*> lLocalMapPoints;
		for (std::list<cMultiKeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
			lit != lend; lit++)
		{
			std::vector<cMapPoint*> vpMPs = (*lit)->GetMapPointMatches();

			for (std::vector<cMapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; ++vit)
			{
				cMapPoint* pMP = *vit;
				if (pMP)
					if (!pMP->isBad())
						if (pMP->mnBALocalForKF != pKF->mnId)
						{
							lLocalMapPoints.push_back(pMP);
							pMP->mnBALocalForKF = pKF->mnId;
						}
			}
		}

		// Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
		std::list<cMultiKeyFrame*> lFixedCameras;
		for (std::list<cMapPoint*>::iterator lit = lLocalMapPoints.begin(),
			lend = lLocalMapPoints.end(); lit != lend; lit++)
		{
			std::map<cMultiKeyFrame*, std::vector<size_t> > observations = (*lit)->GetObservations();
			for (std::map<cMultiKeyFrame*, std::vector<size_t> >::iterator mit =
				observations.begin(), mend = observations.end(); mit != mend; ++mit)
			{
				cMultiKeyFrame* pKFi = mit->first;

				if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
				{
					pKFi->mnBAFixedForKF = pKF->mnId;
					if (!pKFi->isBad())
						lFixedCameras.push_back(pKFi);
				}
			}
		}

		// Setup optimizer
		g2o::SparseOptimizer optimizer;
		g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

		linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

		g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		optimizer.setAlgorithm(solver);
		optimizer.setVerbose(false);

		//solver_ptr->setSchur(false);
		//optimizer.setComputeBatchStatistics(true);
		if (pbStopFlag)
			optimizer.setForceStopFlag(pbStopFlag);

		g2o::SparseOptimizerTerminateAction* terminateAction = 0;
		terminateAction = new g2o::SparseOptimizerTerminateAction;
		terminateAction->setGainThreshold(1e-6);
		terminateAction->setMaxIterations(15);
		optimizer.addPostIterationAction(terminateAction);

		long unsigned int maxKFid = 0;

		// SET LOCAL KEYFRAME VERTICES
		bool oneFixed = false;
		for (std::list<cMultiKeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
			lit != lend; lit++)
		{
			cMultiKeyFrame* pKFi = *lit;
			VertexMt_cayley* vSE3 = new VertexMt_cayley();
			vSE3->setEstimate(pKFi->camSystem.Get_M_t_min());
			vSE3->setId(pKFi->mnId);
			oneFixed = pKFi->mnId == 0;
			vSE3->setFixed(oneFixed);
			optimizer.addVertex(vSE3);
			if (pKFi->mnId > maxKFid)
				maxKFid = pKFi->mnId;
			if (!vSE3->fixed())
				numUnknowns += 6;
		}

		// if no camera was fixed and also the fixed camera vector is empty
		// fix one camera
		if (!oneFixed && lFixedCameras.size() == 0)
		{
			std::list<cMultiKeyFrame*>::iterator lit = lLocalKeyFrames.begin();
			cMultiKeyFrame* pKFi = *lit;
			optimizer.vertex(pKFi->mnId)->setFixed(true);
			numUnknowns -= 6;
		}

		// SET FIXED KEYFRAME VERTICES
		for (std::list<cMultiKeyFrame*>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end();
			lit != lend; lit++)
		{
			cMultiKeyFrame* pKFi = *lit;
			VertexMt_cayley* vSE3 = new VertexMt_cayley();
			vSE3->setEstimate(pKFi->camSystem.Get_M_t_min());
			vSE3->setId(pKFi->mnId);
			vSE3->setFixed(true);
			optimizer.addVertex(vSE3);
			if (pKFi->mnId > maxKFid)
				maxKFid = pKFi->mnId;
		}
		unsigned long currVertexIdx = maxKFid + 1;

		unsigned long maxMcid = 0;
		unsigned long maxIOid = 0;

		// because the keyframe ids are not continous
		const int nrCams = pKF->camSystem.GetNrCams();
		// SET Mc VERTICES 
		for (int c = 0; c < nrCams; ++c)
		{
			VertexMc_cayley* vMc = new VertexMc_cayley();
			vMc->setEstimate(pKF->camSystem.Get_M_c_min(c));
			vMc->setId(currVertexIdx);
			vMc->setFixed(true);
			optimizer.addVertex(vMc);

			if (!vMc->fixed())
				numUnknowns += 6;
			currVertexIdx++;
		}
		maxMcid = currVertexIdx;

		// SET IO VERTICES 
		for (int c = 0; c < nrCams; ++c)
		{
			VertexOmniCameraParameters* vIO =
				new VertexOmniCameraParameters(pKF->camSystem.GetCamModelObj(c));
			vIO->setEstimate(pKF->camSystem.GetCamModelObj(c).toVector());
			vIO->setId(currVertexIdx);
			vIO->setFixed(true);
			optimizer.addVertex(vIO);
			if (!vIO->fixed())
				numUnknowns += 6;
			currVertexIdx++;
		}
		maxIOid = currVertexIdx; // to be able to recover the corresponding vertex

		// SET MAP POINT VERTICES
		const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size())*lLocalMapPoints.size();

		std::vector<EdgeProjectXYZ2MCS*> vpEdges;
		vpEdges.reserve(nExpectedSize);

		std::vector<cMultiKeyFrame*> vpEdgeKF;
		vpEdgeKF.reserve(nExpectedSize);

		std::vector<double> vSigmas2;
		vSigmas2.reserve(nExpectedSize);

		std::vector<cMapPoint*> vpMapPointEdge;
		vpMapPointEdge.reserve(nExpectedSize);

		std::vector<size_t> obsIndices;
		std::vector<size_t> cont_obsIndices;
		const double thHuber = 1.345 * stdRecon;

		std::unordered_map<int, int> mapPointId_to_cont_g2oId;
		std::unordered_map<EdgeProjectXYZ2MCS*, int> mapMapPt_to_edge;
		//cout << "Size local map points: " << lLocalMapPoints.size() << endl;
		for (std::list<cMapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end();
			lit != lend; lit++)
		{
			cMapPoint* pMP = *lit;
			if (pMP->isBad())
				continue;

			int nrObs = pMP->TotalNrObservations();
			//cout << pMP->GetMeanPtError() << endl;;

			VertexPointXYZ* vPoint = new VertexPointXYZ();
			vPoint->setEstimate(pMP->GetWorldPos());
			// for later save the map from the continous to map point idxs
			mapPointId_to_cont_g2oId[pMP->mnId] = currVertexIdx;
			vPoint->setId(currVertexIdx);

			vPoint->setFixed(false);
			vPoint->setMarginalized(true);

			if (!vPoint->fixed())
				numUnknowns += 3;

			optimizer.addVertex(vPoint);

			std::map<cMultiKeyFrame*, std::vector<size_t> > observations = pMP->GetObservations();

			// SET EDGES
			// in contrast to ORB_SLAM an additional layer of measurements has to be introduced
			// we also need to search for the camera in which the observation was made
			int obsCnt = 0;
			for (std::map<cMultiKeyFrame*, std::vector<size_t> >::iterator mit =
				observations.begin(), mend = observations.end();
				mit != mend; ++mit)
			{
				cMultiKeyFrame* pKF = mit->first;

				if (pKF->isBad())
					continue;
				// get all image points for this keyframe corresponding to one map point
				std::vector<size_t>& imagePoints = mit->second;

				// add all observations
				for (auto obsIdx : imagePoints)
				{
					int cam = pKF->keypoint_to_cam.find(obsIdx)->second;

					cv::KeyPoint kpUn = pKF->GetKeyPoint(obsIdx);
					cv::Vec2d obs(kpUn.pt.x, kpUn.pt.y);

					EdgeProjectXYZ2MCS* e = new EdgeProjectXYZ2MCS();
					e->setMeasurement(Eigen::Vector2d(kpUn.pt.x, kpUn.pt.y));
					const double invSigma2 = pKF->GetInvSigma2(kpUn.octave);
					e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
					// Mt
					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
						optimizer.vertex(pKF->mnId)));
					// 3D point
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
						optimizer.vertex(currVertexIdx)));
					// Mc
					e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
						optimizer.vertex(maxMcid - nrCams + cam)));
					// IO
					e->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
						optimizer.vertex(maxIOid - nrCams + cam)));
					g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
					rk->setDelta(thHuber);
					e->setRobustKernel(rk);

					++numObservationsTotal;

					optimizer.addEdge(e);
					vpEdges.push_back(e);
					vpEdgeKF.push_back(pKF);
					vpMapPointEdge.push_back(pMP);
					obsIndices.push_back(obsCnt);
					cont_obsIndices.push_back(obsIdx);
					mapMapPt_to_edge[e]++;
					++obsCnt;
				}
			}
			++currVertexIdx;
		}

		if (pbStopFlag)
			if (*pbStopFlag)
				return lLocalKeyFrames;

		size_t redundancy = 2 * numObservationsTotal - numUnknowns;

		///////////////////////////
		// optimize the first time
		///////////////////////////
		optimizer.initializeOptimization(0);
		int result = optimizer.optimize(10);
		if (result == g2o::OptimizationAlgorithm::Fail)
		{
			cout << "Optimization failed" << endl;
			return lLocalKeyFrames;
		}

		// not stopped yet? go on
		bool bDoMore = true;
		if (pbStopFlag)
			if (*pbStopFlag)
				bDoMore = false;

		if (bDoMore)
		{
			double huberK2 = thHuber*thHuber;
			// Check inlier observations
			for (size_t i = 0, iend = vpEdges.size(); i < iend; ++i)
			{
				EdgeProjectXYZ2MCS* e = vpEdges[i];
				cMapPoint* pMP = vpMapPointEdge[i];

				if (pMP->isBad())
					continue;

				if (e->chi2() > huberK2)
				{
					cMultiKeyFrame* pKFi = vpEdgeKF[i];
					pKFi->EraseMapPointMatch(cont_obsIndices[i]);
					pMP->EraseObservation(pKFi, cont_obsIndices[i]);
					e->setLevel(1);
					vpEdges[i] = NULL;
					vpMapPointEdge[i] = NULL;
				}
			}

			optimizer.initializeOptimization(0);
			result = optimizer.optimize(15);

			if (result == g2o::OptimizationAlgorithm::Fail)
			{
				cout << "Optimization failed" << endl;
				return lLocalKeyFrames;
			}
			else
			{
				for (size_t i = 0, iend = vpEdges.size(); i < iend; ++i)
				{

					if (!vpEdges[i])
						continue;
					EdgeProjectXYZ2MCS* e = vpEdges[i];
					cMapPoint* pMP = vpMapPointEdge[i];

					if (pMP->isBad())
						continue;

					if (e->chi2() > huberK2)
					{
						--numObservationsTotal;
						cMultiKeyFrame* pKFi = vpEdgeKF[i];
						pKFi->EraseMapPointMatch(cont_obsIndices[i]);
						pMP->EraseObservation(pKFi, cont_obsIndices[i]);
						vpEdges[i] = NULL;
						vpMapPointEdge[i] = NULL;
						e->setLevel(1);
					}
				}

				optimizer.initializeOptimization(0);

				// don't allow map change
				//std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

				std::vector<std::pair<int, int> > blockIndices;
				bool havingMasks = false;
				for (std::list<cMultiKeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
					lit != lend; ++lit)
				{
					cMultiKeyFrame* pKFl = *lit;
					if (pKFl->isBad())
						continue;

					VertexMt_cayley* vSE3 = static_cast<VertexMt_cayley*>(optimizer.vertex(pKFl->mnId));
					pKFl->camSystem.Set_M_t_from_min(vSE3->estimate());
					if (vSE3->hessianIndex() >= 0)
						blockIndices.push_back(make_pair(vSE3->hessianIndex(), vSE3->hessianIndex()));
				}

				double s0 = 1.0;
				//if (getCovMats)
				//	if (redundancy > 0 && !isnan(optimizer.chi2()) && !isinf(optimizer.chi2()))
				//		s0 = sqrt(optimizer.chi2() / (double)redundancy);

				std::vector<cMapPoint*> ptsForCov;
				std::vector<VertexPointXYZ*> vPoints;

				for (std::list<cMapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end();
					lit != lend; ++lit)
				{
					cMapPoint* pMP = *lit;
					if (pMP->isBad())
						continue;

					int cnt = mapPointId_to_cont_g2oId.count(pMP->mnId);
					if (cnt == 0)
						continue;
					if (pMP->TotalNrObservations() <= 1)
						continue;
					int contId = mapPointId_to_cont_g2oId.find(pMP->mnId)->second;

					VertexPointXYZ* vPoint = static_cast<VertexPointXYZ*>(optimizer.vertex(contId));
					// count number of edges

					if (vPoint->edges().size() >= 2 && !vPoint->fixed())
					{
						pMP->SetWorldPos(vPoint->estimate());
						pMP->UpdateNormalAndDepth();
						pMP->ComputeDistinctiveDescriptors();
					}

				}
			}
		}
		return lLocalKeyFrames;
	}

}
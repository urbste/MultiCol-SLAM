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
	double cOptimizer::stdSim = 4.0;


	int cOptimizer::OptimizeSim3(cMultiKeyFrame *pKF1,
		cMultiKeyFrame *pKF2,
		vector<cMapPoint *> &vpMatches1,
		g2o::Sim3 &g2oS12)
	{
		g2o::SparseOptimizer optimizer;
		g2o::BlockSolverX::LinearSolverType * linearSolver;

		linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

		g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		optimizer.setAlgorithm(solver);

		g2o::SparseOptimizerTerminateAction* terminateAction = 0;
		terminateAction = new g2o::SparseOptimizerTerminateAction;
		terminateAction->setGainThreshold(1e-6);
		terminateAction->setMaxIterations(15);
		optimizer.addPostIterationAction(terminateAction);
		optimizer.setVerbose(false);

		// Calibration
		cMultiCamSys_ camSys1 = pKF1->camSystem;
		cMultiCamSys_ camSys2 = pKF2->camSystem;

		// SET SIMILARITY VERTEX
		VertexSim3Expmap_Multi * vSim3 =
			new VertexSim3Expmap_Multi(
			pKF1->keypoint_to_cam,
			pKF2->keypoint_to_cam);

		vSim3->setEstimate(g2oS12);
		vSim3->setId(0);
		vSim3->setFixed(false);
		vSim3->camSys1 = &camSys1;
		vSim3->camSys2 = &camSys2;
		optimizer.addVertex(vSim3);

		// SET MAP POINT VERTICES
		const int N = vpMatches1.size();
		vector<cMapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
		vector<EdgeSim3ProjectXYZ_Multi*> vpEdges12;
		vector<EdgeInverseSim3ProjectXYZ_Multi*> vpEdges21;
		vector<float> vSigmas12, vSigmas21;
		vector<size_t> vnIndexEdge;

		vnIndexEdge.reserve(2 * N);
		vpEdges12.reserve(2 * N);
		vpEdges21.reserve(2 * N);

		const double deltaHuber = 1.345 * stdSim;
		const double deltaHuber2 = deltaHuber*deltaHuber;
		int nCorrespondences = 0;

		cv::Matx44d invM_t1 = cConverter::invMat(pKF1->camSystem.Get_M_t());
		cv::Matx44d invM_t2 = cConverter::invMat(pKF2->camSystem.Get_M_t());
		Eigen::Matrix2d I2x2 = Eigen::Matrix2d::Identity();

		for (int i = 0; i < N; i++)
		{
			if (!vpMatches1[i])
				continue;

			cMapPoint* pMP1 = vpMapPoints1[i];
			cMapPoint* pMP2 = vpMatches1[i];

			int id1 = 2 * i + 1;
			int id2 = 2 * (i + 1);

			int i2 = pMP2->GetIndexInKeyFrame(pKF2)[0];

			if (pMP1 && pMP2)
			{
				if (!pMP1->isBad() && !pMP2->isBad() && i2 >= 0)
				{
					VertexPointXYZ* vPoint1 = new VertexPointXYZ();
					cv::Vec4d P4D1w = cConverter::toVec4d(pMP1->GetWorldPos());
					cv::Vec4d rot1CamSys = invM_t1*P4D1w;
					vPoint1->setEstimate(cv::Vec3d(rot1CamSys(0), rot1CamSys(1), rot1CamSys(2)));
					vPoint1->setId(id1);
					vPoint1->setFixed(true);
					vPoint1->SetID(i);
					optimizer.addVertex(vPoint1);

					VertexPointXYZ* vPoint2 = new VertexPointXYZ();
					cv::Vec4d P4D2w = cConverter::toVec4d(pMP2->GetWorldPos());
					cv::Vec4d rot2CamSys = invM_t2*P4D2w;
					vPoint2->setEstimate(cv::Vec3d(rot2CamSys(0), rot2CamSys(1), rot2CamSys(2)));
					vPoint2->setId(id2);
					vPoint2->setFixed(true);
					vPoint2->SetID(i2);
					optimizer.addVertex(vPoint2);
				}
				else
					continue;
			}
			else
				continue;

			nCorrespondences++;

			// SET EDGE x1 = S12*X2, projection from 2 to 1
			Eigen::Matrix<double, 2, 1> obs1;
			cv::KeyPoint kpUn1 = pKF1->GetKeyPoint(i);
			obs1 << kpUn1.pt.x, kpUn1.pt.y;

			EdgeSim3ProjectXYZ_Multi* e12 =
				new EdgeSim3ProjectXYZ_Multi();
			e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2))); // point
			e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0))); // Sim3
			e12->setMeasurement(obs1);
			double invSigmaSquare1 = pKF1->GetInvSigma2(kpUn1.octave);
			e12->setInformation(I2x2*invSigmaSquare1);

			g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
			e12->setRobustKernel(rk1);
			rk1->setDelta(deltaHuber);
			optimizer.addEdge(e12);

			// SET EDGE x2 = S21*X1, projection from 1 to 2
			Eigen::Matrix<double, 2, 1> obs2;
			cv::KeyPoint kpUn2 = pKF2->GetKeyPoint(i2);
			obs2 << kpUn2.pt.x, kpUn2.pt.y;

			EdgeInverseSim3ProjectXYZ_Multi* e21 =
				new EdgeInverseSim3ProjectXYZ_Multi();
			e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1))); // point
			e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0))); // Sim3
			e21->setMeasurement(obs2);
			double invSigmaSquare2 = pKF2->GetSigma2(kpUn2.octave);
			e21->setInformation(I2x2*invSigmaSquare2);

			g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
			e21->setRobustKernel(rk2);
			rk2->setDelta(deltaHuber);
			optimizer.addEdge(e21);

			vpEdges12.push_back(e12);
			vpEdges21.push_back(e21);
			vnIndexEdge.push_back(i);
		}

		// Optimize

		optimizer.initializeOptimization();
		optimizer.optimize(5);

		// Check inliers
		int nBad = 0;
		for (size_t i = 0; i < vpEdges12.size(); i++)
		{
			EdgeSim3ProjectXYZ_Multi* e12 = vpEdges12[i];
			EdgeInverseSim3ProjectXYZ_Multi* e21 = vpEdges21[i];
			if (!e12 || !e21)
				continue;

			if (e12->chi2() > deltaHuber2 ||
				e21->chi2() > deltaHuber2)
			{
				size_t idx = vnIndexEdge[i];
				vpMatches1[idx] = NULL;
				optimizer.removeEdge(e12);
				optimizer.removeEdge(e21);
				vpEdges12[i] = NULL;
				vpEdges21[i] = NULL;
				nBad++;
			}
		}

		int nMoreIterations;
		if (nBad > 0)
			nMoreIterations = 10;
		else
			nMoreIterations = 5;

		if (nCorrespondences - nBad < 3)
			return 0;

		// Optimize again only with inliers
		optimizer.initializeOptimization();
		optimizer.optimize(nMoreIterations);

		int nIn = 0;
		for (size_t i = 0; i < vpEdges12.size(); i++)
		{
			EdgeSim3ProjectXYZ_Multi* e12 = vpEdges12[i];
			EdgeInverseSim3ProjectXYZ_Multi* e21 = vpEdges21[i];
			if (!e12 || !e21)
				continue;

			if (e12->chi2() > deltaHuber2 ||
				e21->chi2()>deltaHuber2)
			{
				size_t idx = vnIndexEdge[i];
				vpMatches1[idx] = NULL;
			}
			else
				nIn++;
		}

		// Recover optimized Sim3
		g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
		g2oS12 = vSim3_recov->estimate();

		return nIn;
	}


	void cOptimizer::OptimizeEssentialGraph(cMap* pMap,
		cMultiKeyFrame* pLoopMKF,
		cMultiKeyFrame* pCurMKF,
		g2o::Sim3 &Scurw,
		const cLoopClosing::KeyFrameAndPose &NonCorrectedSim3,
		const cLoopClosing::KeyFrameAndPose &CorrectedSim3,
		const std::map<cMultiKeyFrame*, std::set<cMultiKeyFrame*> > &LoopConnections)
	{
		g2o::SparseOptimizer optimizer;
		optimizer.setVerbose(true);
		g2o::BlockSolver_7_3::LinearSolverType* linearSolver =
			new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
		g2o::BlockSolver_7_3* solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
		g2o::OptimizationAlgorithmLevenberg* solver =
			new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		solver->setUserLambdaInit(1e-6);
		optimizer.setAlgorithm(solver);

		g2o::SparseOptimizerTerminateAction* terminateAction = 0;
		terminateAction = new g2o::SparseOptimizerTerminateAction;
		terminateAction->setGainThreshold(1e-6);
		terminateAction->setMaxIterations(15);
		optimizer.addPostIterationAction(terminateAction);

		// get all keyframes and map points
		unsigned int nMaxMKFid = pMap->GetMaxKFid();
		vector<cMultiKeyFrame*> vpMKFs = pMap->GetAllKeyFrames();
		vector<cMapPoint*> vpMPs = pMap->GetAllMapPoints();

		// create vectors for uncorrected and corrected similarity transformatons
		// between MKFs
		vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxMKFid + 1);
		vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw_corrected(nMaxMKFid + 1);
		vector<simpleVertexSim3Expmap*> vertices(nMaxMKFid + 1); // keeps the parameters to be estimated

		// limit for minimum weight between MKFs
		const int minNumFeat = 100;
		// now set the MKF vertices
		for (auto pMKF : vpMKFs)
		{
			if (pMKF->isBad())
				continue;

			simpleVertexSim3Expmap* VSim3 = new simpleVertexSim3Expmap();

			int MKFid = pMKF->mnId;

			// if this MKF was already corrected
			cLoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pMKF);
			if (it != CorrectedSim3.end())
			{
				vScw[MKFid] = it->second;
				VSim3->setEstimate(it->second); // set vertex estimate
			}
			else // convert rigid transformation to similarity
			{
				// we need the inverse transformation because, we store the world frame M_t,
				// instead of M_t^-1
				cv::Matx44d poseInv = pMKF->GetPoseInverse();
				Eigen::Matrix<double, 3, 3> R =
					cConverter::toMatrix3d(cConverter::Hom2R(poseInv));
				Eigen::Matrix<double, 3, 1> t =
					cConverter::toVector3d(cConverter::Hom2T(poseInv));
				g2o::Sim3 sim3(R, t, 1.0);
				vScw[MKFid] = sim3;
				VSim3->setEstimate(sim3); // set vertex estimate
			}
			// fix the transformation if the current MKF is the loop MKF
			if (pMKF == pLoopMKF)
				VSim3->setFixed(true);
			VSim3->setId(MKFid);
			VSim3->setMarginalized(false);
			optimizer.addVertex(VSim3);

			vertices[MKFid] = VSim3;
		}


		set<pair<long unsigned int, long unsigned int>> insertedEdges;
		const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

		// Now set all the loop edges
		for (map<cMultiKeyFrame*, set<cMultiKeyFrame*>>::const_iterator iter = LoopConnections.begin(),
			iterEnd = LoopConnections.end(); iter != iterEnd; ++iter)
		{
			cMultiKeyFrame* MKF = iter->first;
			const long unsigned int MKFid_i = MKF->mnId;
			const set<cMultiKeyFrame*>& connections = iter->second;
			const g2o::Sim3 Siw = vScw[MKFid_i];
			const g2o::Sim3 Swi = Siw.inverse();
			// now iterate through all connections
			for (set<cMultiKeyFrame*>::const_iterator connIter = connections.begin(),
				connIterEnd = connections.end(); connIter != connIterEnd; ++connIter)
			{
				const long unsigned int MKFid_j = (*connIter)->mnId;
				if ((MKFid_i != pCurMKF->mnId ||
					MKFid_j != pLoopMKF->mnId) &&
					MKF->GetWeight(*connIter) < minNumFeat)
					continue;

				const g2o::Sim3 Sjw = vScw[MKFid_j];
				edgeSim3* e = new edgeSim3();
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(MKFid_j)));
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(MKFid_i)));
				e->setMeasurement(Sjw * Swi);
				e->information() = matLambda;
				optimizer.addEdge(e);
				insertedEdges.insert(make_pair(min(MKFid_i, MKFid_j), max(MKFid_i, MKFid_j)));
			}
		}

		// Now set all normal edges
		for (size_t i = 0, iend = vpMKFs.size(); i < iend; ++i)
		{
			cMultiKeyFrame* pMKF = vpMKFs[i];
			const int MKFid_i = pMKF->mnId;

			g2o::Sim3 Swi;
			cLoopClosing::KeyFrameAndPose::const_iterator it_i = NonCorrectedSim3.find(pMKF);
			if (it_i != NonCorrectedSim3.end())
				Swi = (it_i->second).inverse();
			else
				Swi = vScw[MKFid_i].inverse();

			cMultiKeyFrame* parentMKF = pMKF->GetParent();
			/////////////////////
			// Spanning Tree Edges
			/////////////////////
			if (parentMKF)
			{
				int MKFid_j = parentMKF->mnId;
				g2o::Sim3 Sjw;
				cLoopClosing::KeyFrameAndPose::const_iterator it_j = NonCorrectedSim3.find(parentMKF);
				if (it_j != NonCorrectedSim3.end())
					Sjw = it_j->second;
				else
					Sjw = vScw[MKFid_j];

				edgeSim3* e = new edgeSim3();
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(MKFid_j)));
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(MKFid_i)));
				e->setMeasurement(Sjw * Swi);
				e->information() = matLambda;
				optimizer.addEdge(e);
			}
			/////////////////////
			// loop edges
			/////////////////////
			const set<cMultiKeyFrame*> loopEdges = pMKF->GetLoopEdges();
			for (set<cMultiKeyFrame*>::const_iterator itLoop = loopEdges.begin(),
				itLoopEnd = loopEdges.end(); itLoop != itLoopEnd; ++itLoop)
			{
				cMultiKeyFrame* loopMKF = *itLoop;
				const int MKFid_l = loopMKF->mnId;
				if (MKFid_l < pMKF->mnId)
				{
					g2o::Sim3 Slw;
					cLoopClosing::KeyFrameAndPose::const_iterator it_l = NonCorrectedSim3.find(loopMKF);
					if (it_l != NonCorrectedSim3.end())
						Slw = it_l->second;
					else
						Slw = vScw[MKFid_l];

					edgeSim3* e = new edgeSim3();
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(MKFid_l)));
					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(MKFid_i)));
					e->setMeasurement(Slw * Swi);
					e->information() = matLambda;
					optimizer.addEdge(e);
				}
			}
			/////////////////////
			// covisibility graph edges
			/////////////////////
			const vector<cMultiKeyFrame*> covConnectedMKFs = pMKF->GetCovisiblesByWeight(minNumFeat);
			for (vector<cMultiKeyFrame*>::const_iterator itCov = covConnectedMKFs.begin(),
				itCovEnd = covConnectedMKFs.end(); itCov != itCovEnd; ++itCov)
			{
				cMultiKeyFrame* covMKF = *itCov;
				const int MKFid_n = covMKF->mnId;
				if (MKFid_n < pMKF->mnId)
				{
					g2o::Sim3 Snw;
					cLoopClosing::KeyFrameAndPose::const_iterator it_c = NonCorrectedSim3.find(covMKF);
					if (it_c != NonCorrectedSim3.end())
						Snw = it_c->second;
					else
						Snw = vScw[MKFid_n];

					edgeSim3* e = new edgeSim3();
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(MKFid_n)));
					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(MKFid_i)));
					e->setMeasurement(Snw * Swi);
					e->information() = matLambda;
					optimizer.addEdge(e);
				}
			}
		}

		// optimize essential graph
		optimizer.initializeOptimization(0);
		optimizer.optimize(20);

		/////////////////////
		// recover rigid SE3 poses. Sim3: [sR t] -> SE3 [R t/s]
		/////////////////////
		for (auto& itMKF : vpMKFs)
		{
			const int MKFid_i = itMKF->mnId;
			simpleVertexSim3Expmap* VSim3 = static_cast<simpleVertexSim3Expmap*>(optimizer.vertex(MKFid_i));
			g2o::Sim3 CorrectedSiw = VSim3->estimate();
			vScw_corrected[MKFid_i] = CorrectedSiw.inverse();

			// set final pose
			double scale = CorrectedSiw.scale();
			Eigen::Matrix3d R = CorrectedSiw.rotation().toRotationMatrix();
			Eigen::Vector3d t = CorrectedSiw.translation() / scale;
			itMKF->SetPose(cConverter::invMat(cConverter::toCvSE3(R, t))); // inverse!!
		}

		/////////////////////
		// correct map points
		/////////////////////
		for (auto& itMPs : vpMPs)
		{
			if (itMPs->isBad())
				continue;
			int MP_ref_MKFid = 0;
			if (itMPs->mnCorrectedByKF == pCurMKF->mnId)
				MP_ref_MKFid = itMPs->mnCorrectedReference;
			else
			{
				cMultiKeyFrame* refMKF = itMPs->GetReferenceKeyFrame();
				MP_ref_MKFid = refMKF->mnId;
			}


			g2o::Sim3 Srw = vScw[MP_ref_MKFid];
			g2o::Sim3 corrected_Swr = vScw_corrected[MP_ref_MKFid];

			cv::Vec3d mp3D = itMPs->GetWorldPos();
			Eigen::Vector3d eig_mp3D =
				corrected_Swr.map(Srw.map(cConverter::toVector3d(mp3D)));
			itMPs->SetWorldPos(cConverter::toCvVec3d(eig_mp3D));
			itMPs->UpdateNormalAndDepth();
		}
	}

}
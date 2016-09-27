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

#include "cMultiInitializer.h"

#include "cOptimizer.h"
#include "cConverter.h"
#include <thread>
#include <random>
#include "DBoW2/DUtils/Random.h"
#include "misc.h"

namespace MultiColSLAM
{
	cMultiInitializer::cMultiInitializer(const cMultiFrame &ReferenceFrame,
		double sigma, int iterations)
	{
		camSystem = ReferenceFrame.camSystem;

		mvKeys1 = ReferenceFrame.mvKeys;
		mvKeysRays1 = ReferenceFrame.mvKeysRays;
		referenceFrame = ReferenceFrame;

		mSigma = sigma;
		mSigma2 = sigma * sigma;
		mMaxIterations = iterations;
	}

	bool cMultiInitializer::Initialize(cMultiFrame &currentFrame,
		const std::vector<int> &vMatches12,
		cv::Matx33d &R21,
		cv::Vec3d &t21,
		std::vector<cv::Vec3d> &vP3D,
		std::vector<bool> &vbTriangulated,
		int& bestCam)
	{
		// Fill structures with current keypoints and matches with reference frame
		// Reference Frame: 1, Current Frame: 2
		mvKeys2 = currentFrame.mvKeys;
		mvKeysRays2 = currentFrame.mvKeysRays;

		mvMatches12.clear();
		mvMatches12.reserve(mvKeys2.size());
		mvbMatched1.resize(mvKeys1.size());

		for (int i = 0, iend = vMatches12.size(); i < iend; i++)
		{
			if (vMatches12[i] >= 0)
			{
				mvMatches12.push_back(std::make_pair(i, vMatches12[i]));
				mvbMatched1[i] = true;
			}
			else
				mvbMatched1[i] = false;
		}

		const int N = mvMatches12.size();

		// Indices for minimum set selection
		std::vector<size_t> vAllIndices;
		vAllIndices.reserve(N);
		std::vector<size_t> vAvailableIndices;

		for (int i = 0; i < N; i++)
			vAllIndices.push_back(i);
		// pretty easy
		// eightpt ransac the problem, then test the median norm of the result
		// if it is big enough -> init the system
		int nrCams = currentFrame.camSystem.GetNrCams();
		std::vector<opengv::bearingVectors_t> bear1(nrCams);
		std::vector<opengv::bearingVectors_t> bear2(nrCams);
		std::vector<std::vector<int> > bear1_cont_indices(nrCams);
		std::vector<std::vector<int> > bear2_cont_indices(nrCams);

		// assign matches/bearing vectors to each camera
		for (int i = 0; i < N; ++i)
		{
			int idx1 = mvMatches12[i].first;
			int idx2 = mvMatches12[i].second;
			// get indices for the corresponding camera
			int camidx1 = referenceFrame.keypoint_to_cam.find(idx1)->second;
			int camidx2 = currentFrame.keypoint_to_cam.find(idx2)->second;
			// save which index belongs to which bearing vector, so that we can recover the
			// observations later
			bear1_cont_indices[camidx1].push_back(idx1);
			bear1[camidx1].push_back(opengv::bearingVector_t(mvKeysRays1[idx1](0),
				mvKeysRays1[idx1](1), mvKeysRays1[idx1](2)));

			bear2_cont_indices[camidx2].push_back(idx2);
			bear2[camidx2].push_back(opengv::bearingVector_t(mvKeysRays2[idx2](0),
				mvKeysRays2[idx2](1), mvKeysRays2[idx2](2)));
		}

		vector<int> nr_recon(nrCams);
		vector<cv::Matx33d> Rel_Rs(nrCams);
		vector<cv::Vec3d> Rel_ts(nrCams);
		vector<vector<bool> > triangulated(nrCams);
		vector<vector<cv::Vec3d> > vP3Ds(nrCams);
		vector<double> normsAll(nrCams);
		// calculate an essential matrix for each camera separately
		// ge, 17pt and 6pt are just too slow
		// then take the one with the most 
		// inliers? biggest norm? most reconstructed pts?
		for (int c = 0; c < nrCams; ++c)
		{
			nr_recon[c] = 0;
			normsAll[c] = 0;
			opengv::relative_pose::CentralRelativeAdapter adapter(bear1[c], bear2[c]);

			opengv::sac::Ransac<
				opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem > ransac;

			std::shared_ptr<
				opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem> relposeproblem_ptr(
				new opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem(
				adapter,
				opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem::STEWENIUS));

			ransac.sac_model_ = relposeproblem_ptr;
			ransac.threshold_ = 0.0001;
			ransac.max_iterations_ = 200;

			ransac.computeModel();
			Eigen::Matrix3d R = ransac.model_coefficients_.block<3, 3>(0, 0);
			Eigen::Vector3d t = ransac.model_coefficients_.block<3, 1>(0, 3);
			Rel_Rs[c] = cConverter::toCvMat(R);
			Rel_ts[c] = cConverter::toCvVec3d(t);
			vector<double> norms;
			vector<Match> vInlierMatches12;
			// recover inlier measurements
			for (int i = 0; i < ransac.inliers_.size(); ++i)
			{
				int idx = ransac.inliers_[i];
				int cam_matchidx1 = bear1_cont_indices[c][idx];
				int cam_matchidx2 = bear2_cont_indices[c][idx];
				vInlierMatches12.push_back(make_pair(cam_matchidx1,
					cam_matchidx2));
				cv::Vec3d bear1 = mvKeysRays1[cam_matchidx1];
				cv::Vec3d bear2 = mvKeysRays2[cam_matchidx2];
				cv::Vec3d res = bear1.cross(cConverter::toCvMat(R)*bear2);
				norms.push_back(cv::norm(res));
			}
			if (ransac.inliers_.size() <= 0)
				continue;

			normsAll[c] = median(norms);
			//if (normsAll[c] < 0.02)
			//	continue;
			nr_recon[c] = CheckRT(currentFrame, Rel_Rs[c], Rel_ts[c],
				mvKeys1, mvKeys2,
				mvKeysRays1, mvKeysRays2, vInlierMatches12,
				vP3Ds[c], 5, triangulated[c], 1.0, c);
		}

		// find cam with most reconstructed points
		bool init = false;
		for (int c = 0; c < nrCams; ++c)
		{
			if (nr_recon[c] > 60 &&
				normsAll[c] > 0.06)
			{
				init = true;
				bestCam = c;
				if (c > 0)
					if (normsAll[c] > normsAll[c - 1])
						bestCam = c;
			}
		}
		vbTriangulated = triangulated[bestCam];
		vP3D = vP3Ds[bestCam];
		R21 = Rel_Rs[bestCam];
		t21 = Rel_ts[bestCam];

		return init;
	}

	int cMultiInitializer::CheckRT(const cMultiFrame& CurrentFrame,
		const cv::Matx33d& R,
		const cv::Vec3d& t,
		const std::vector<cv::KeyPoint>& vKeys1,
		const std::vector<cv::KeyPoint>& vKeys2,
		const std::vector<cv::Vec3d>& vKeysRays1,
		const std::vector<cv::Vec3d>& vKeysRays2,
		const std::vector<Match>& vMatches12,
		std::vector<cv::Vec3d>& vP3D,
		double th2,
		std::vector<bool>& vbGood,
		double parallax,
		int currCam)
	{
		double cosThresh = cos(parallax / RHOd);
		vbGood = std::vector<bool>(vKeys1.size(), false);
		vP3D.resize(vKeys1.size());

		std::vector<double> vCosParallax;
		vCosParallax.reserve(vKeys1.size());

		cv::Vec3d O1(0, 0, 0);
		cv::Vec3d O2 = -R.t()*t;
		int nGood = 0;

		for (size_t i = 0, iend = vMatches12.size(); i < iend; ++i)
		{
			int currCam1 = CurrentFrame.keypoint_to_cam.find(vMatches12[i].second)->second;
			if (currCam1 != currCam)
				continue;
			const cv::Vec3d &kpRay1 = vKeysRays1[vMatches12[i].first];
			const cv::Vec3d &kpRay2 = vKeysRays2[vMatches12[i].second];

			const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
			const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

			cv::Vec3d p3dC1;
			p3dC1 = triangulate_point(t, R, kpRay1, kpRay2);

			if (!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2)))
			{
				vbGood[vMatches12[i].first] = false;
				continue;
			}

			// Check parallax
			cv::Vec3d normal1 = p3dC1 - O1;
			double dist1 = cv::norm(normal1);

			cv::Vec3d normal2 = p3dC1 - O2;
			double dist2 = cv::norm(normal2);

			double cosParallax = normal1.dot(normal2) / (dist1*dist2);

			if (p3dC1(2) <= 0 && cosParallax > cosThresh)
				continue;

			cv::Vec3d p3dC2 = R.t()*(p3dC1 - t);

			if (p3dC2(2) <= 0 && cosParallax > cosThresh)
				continue;

			// Check reprojection error in first image
			double u = 0.0;
			double v = 0.0;
			camSystem.GetCamModelObj(currCam).WorldToImg(
				p3dC1(0), p3dC1(1), p3dC1(2), u, v);

			double squareError1 = cv::pow(u - cv::saturate_cast<double>(kp1.pt.x), 2) +
				cv::pow(v - cv::saturate_cast<double>(kp1.pt.y), 2);

			if (squareError1 > th2)
				continue;

			// Check reprojection error in second image
			camSystem.GetCamModelObj(currCam).WorldToImg(
				p3dC2(0), p3dC2(1), p3dC2(2), u, v);

			double squareError2 = cv::pow(u - cv::saturate_cast<double>(kp2.pt.x), 2) +
				cv::pow(v - cv::saturate_cast<double>(kp2.pt.y), 2);

			if (squareError2 > th2)
				continue;

			if (cosParallax < cosThresh)
			{
				vbGood[vMatches12[i].first] = true;
				vCosParallax.push_back(cosParallax);
				vP3D[vMatches12[i].first] = p3dC1;
				++nGood;
			}
		}

		double minParallax = 0.0;
		if (nGood > 0)
		{
			std::vector<double>::iterator result =
				std::min_element(std::begin(vCosParallax), std::end(vCosParallax));

			minParallax = acos(*result) * 180 / CV_PI;
		}
		else
			parallax = 0;

		if (minParallax < parallax)
			nGood = 0;
		return nGood;
	}

}
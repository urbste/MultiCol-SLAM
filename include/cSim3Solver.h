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

#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "cMultiKeyFrame.h"

namespace MultiColSLAM
{
	class cSim3Solver
	{
	public:

		cSim3Solver(cMultiKeyFrame* pKF1,
			cMultiKeyFrame* pKF2,
			const std::vector<cMapPoint*> &vpMatched12,
			cMultiCamSys_* camSys);

		void SetRansacParameters(double probability = 0.99,
			int minInliers = 6,
			int maxIterations = 300);

		bool find(std::vector<bool> &vbInliers12,
			int &nInliers,
			cv::Matx44d& result);

		bool iterate(int nIterations,
			bool &bNoMore,
			std::vector<bool> &vbInliers,
			int &nInliers,
			cv::Matx44d& result);

		cv::Matx33d GetEstimatedRotation();
		cv::Vec3d GetEstimatedTranslation();
		double GetEstimatedScale();


	protected:

		bool Refine();

		void centroid(cv::Matx33d &P,
			cv::Matx33d &Pr,
			cv::Vec3d &C);

		void computeT(cv::Matx33d &P1,
			cv::Matx33d &P2);

		void CheckInliers();

	protected:

		// KeyFrames and matches
		cMultiKeyFrame* mpKF1;
		cMultiKeyFrame* mpKF2;

		std::vector<cv::Vec3d> mvX3Dc1;
		std::vector<cv::Vec3d> mvX3Dc2;
		std::vector<cMapPoint*> mvpMapPoints1;
		std::vector<cMapPoint*> mvpMapPoints2;
		std::vector<cMapPoint*> mvpMatches12;
		std::vector<size_t> mvnIndices1;
		std::vector<size_t> mvSigmaSquare1;
		std::vector<size_t> mvSigmaSquare2;
		std::vector<size_t> mvnMaxError1;
		std::vector<size_t> mvnMaxError2;
		std::vector<int> camIdx1;
		std::vector<int> camIdx2;

		int N;
		int mN1;

		// Current Estimation
		cv::Matx33d mR12i;
		cv::Vec3d mt12i;
		double ms12i;
		cv::Matx44d mT12i;
		cv::Matx44d mT21i;

		cv::Matx44d mT12i_rigid;
		cv::Matx44d mT21i_rigid;

		std::vector<bool> mvbInliersi;
		int mnInliersi;

		// Current Ransac State
		int mnIterations;
		std::vector<bool> mvbBestInliers;
		int mnBestInliers;
		cv::Matx44d mBestT12;
		cv::Matx33d mBestRotation;
		cv::Vec3d mBestTranslation;
		double mBestScale;

		// Refined
		cv::Mat mRefinedT12;
		std::vector<bool> mvbRefinedInliers;
		int mnRefinedInliers;

		// Indices for random selection
		std::vector<size_t> mvAllIndices;

		// Projections
		std::vector<cv::Vec2d> mvP1im1;
		std::vector<cv::Vec2d> mvP2im2;

		// RANSAC probability
		double mRansacProb;

		// RANSAC min inliers
		int mRansacMinInliers;

		// RANSAC max iterations
		int mRansacMaxIts;

		// Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
		double mTh;
		double mSigma2;

		cMultiCamSys_* camSysLocal;
	};
}
#endif

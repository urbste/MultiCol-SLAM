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

#include <vector>
#include <cmath>
#include <opencv/cv.h>
//#include <ros/ros.h>

#include "cSim3Solver.h"
#include "cMultiKeyFrame.h"
#include "cORBmatcher.h"
#include "cConverter.h"

#include <random>

#include "DBoW2/DUtils/Random.h"


namespace MultiColSLAM
{
	cSim3Solver::cSim3Solver(cMultiKeyFrame *pKF1,
		cMultiKeyFrame *pKF2,
		const std::vector<cMapPoint *> &vpMatched12,
		cMultiCamSys_* camSys) :
		mnIterations(0),
		mnBestInliers(0)
	{
		camSysLocal = camSys;
		mpKF1 = pKF1;
		mpKF2 = pKF2;

		std::vector<cMapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

		mN1 = vpMatched12.size();

		mvpMapPoints1.reserve(mN1);
		mvpMapPoints2.reserve(mN1);
		mvpMatches12 = vpMatched12;
		mvnIndices1.reserve(mN1);
		mvX3Dc1.reserve(mN1);
		mvX3Dc2.reserve(mN1);

		mvAllIndices.reserve(mN1);

		size_t idx = 0;
		cv::Matx44d hom1 = cConverter::invMat(pKF1->camSystem.Get_M_t());
		cv::Matx44d hom2 = cConverter::invMat(pKF2->camSystem.Get_M_t());
		for (int i1 = 0; i1 < mN1; ++i1)
		{
			if (vpMatched12[i1])
			{
				cMapPoint* pMP1 = vpKeyFrameMP1[i1];

				if (!pMP1)
					continue;

				cMapPoint* pMP2 = vpMatched12[i1];

				if (pMP1->isBad() || pMP2->isBad())
					continue;

				std::vector<size_t> idxs1 = pMP1->GetIndexInKeyFrame(pKF1);
				std::vector<size_t> idxs2 = pMP2->GetIndexInKeyFrame(pKF2);
				// TODO
				// only if the point was visible in multiple cameras (better scale?)
				//if (!(idxs1.size() > 1 || idxs2.size() > 1))
				//	continue;

				// TODO
				int indexKF1 = idxs1[0]; // naja can be more than 1
				int indexKF2 = idxs2[0]; // naja can be more than 1

				if (indexKF1 < 0 || indexKF2 < 0)
					continue;

				const cv::KeyPoint &kp1 = pKF1->GetKeyPoint(indexKF1);
				const cv::KeyPoint &kp2 = pKF2->GetKeyPoint(indexKF2);

				const double sigmaSquare1 = pKF1->GetSigma2(kp1.octave);
				const double sigmaSquare2 = pKF2->GetSigma2(kp2.octave);

				mvnMaxError1.push_back(9.210*sigmaSquare1);
				mvnMaxError2.push_back(9.210*sigmaSquare2);

				mvpMapPoints1.push_back(pMP1);
				mvpMapPoints2.push_back(pMP2);
				mvnIndices1.push_back(i1);

				int cam1 = pKF1->keypoint_to_cam.find(indexKF1)->second;
				cv::Vec3d X3D1w = pMP1->GetWorldPos();
				cv::Vec2d proj1(0.0, 0.0);
				pKF1->camSystem.WorldToCamHom_fast(cam1, X3D1w, proj1);
				// rotate it to the MCS frame!
				mvX3Dc1.push_back(cConverter::Hom2R(hom1)*X3D1w + cConverter::Hom2T(hom1));
				mvP1im1.push_back(proj1);
				camIdx1.push_back(cam1);

				int cam2 = pKF2->keypoint_to_cam.find(indexKF2)->second;
				cv::Vec3d X3D2w = pMP2->GetWorldPos();
				cv::Vec2d proj2(0.0, 0.0);
				pKF2->camSystem.WorldToCamHom_fast(cam2, X3D2w, proj2);

				mvX3Dc2.push_back(cConverter::Hom2R(hom2)*X3D2w + cConverter::Hom2T(hom2)); // rotate it to the MCS frame!
				mvP2im2.push_back(proj2);
				camIdx2.push_back(cam2);

				mvAllIndices.push_back(idx);
				++idx;
			}
		}

		SetRansacParameters();
	}

	void cSim3Solver::SetRansacParameters(double probability,
		int minInliers,
		int maxIterations)
	{
		mRansacProb = probability;
		mRansacMinInliers = minInliers;
		mRansacMaxIts = maxIterations;

		N = mvpMapPoints1.size(); // number of correspondences

		mvbInliersi.resize(N);

		// Adjust Parameters according to number of correspondences
		double epsilon = (double)mRansacMinInliers / N;

		// Set RANSAC iterations according to probability, epsilon, and max iterations
		int nIterations;

		if (mRansacMinInliers == N)
			nIterations = 1;
		else
			nIterations = ceil(log(1 - mRansacProb) / log(1 - pow(epsilon, 3)));

		mRansacMaxIts = std::max(1, std::min(nIterations, mRansacMaxIts));

		mnIterations = 0;
	}

	bool cSim3Solver::iterate(int nIterations,
		bool &bNoMore,
		std::vector<bool> &vbInliers,
		int &nInliers,
		cv::Matx44d& result)
	{
		bNoMore = false;
		vbInliers = std::vector<bool>(mN1, false);
		nInliers = 0;

		if (N < mRansacMinInliers)
		{

			bNoMore = true;
			return false;
		}

		std::vector<size_t> vAvailableIndices;

		cv::Matx33d P3Dc1i;
		cv::Matx33d P3Dc2i;
		std::vector<int> camIdx1Min(3);
		std::vector<int> camIdx2Min(3);
		int nCurrentIterations = 0;
		std::random_device rd;
		std::mt19937 gen(rd());
		//cout << " mvAllIndices.size()" << mvAllIndices.size() << endl;
		if (mvAllIndices.size() < mRansacMinInliers)
			return false;
		while (mnIterations < mRansacMaxIts &&
			nCurrentIterations < nIterations)
		{
			++nCurrentIterations;
			++mnIterations;

			vAvailableIndices = mvAllIndices;
			std::uniform_int_distribution<> dis(0, vAvailableIndices.size() - 1);
			// Get min set of points
			for (short i = 0; i < 3; ++i)
			{
				int randi = dis(gen);
				int idx = vAvailableIndices[randi];
				P3Dc1i(0, i) = mvX3Dc1[idx](0);
				P3Dc1i(1, i) = mvX3Dc1[idx](1);
				P3Dc1i(2, i) = mvX3Dc1[idx](2);

				P3Dc2i(0, i) = mvX3Dc2[idx](0);
				P3Dc2i(1, i) = mvX3Dc2[idx](1);
				P3Dc2i(2, i) = mvX3Dc2[idx](2);

				camIdx1Min[i] = camIdx1[idx];
				camIdx2Min[i] = camIdx2[idx];

				vAvailableIndices[idx] = vAvailableIndices.back();
				vAvailableIndices.pop_back();
			}

			computeT(P3Dc1i, P3Dc2i);

			CheckInliers();

			if (mnInliersi >= mnBestInliers)
			{
				mvbBestInliers = mvbInliersi;
				mnBestInliers = mnInliersi;
				mBestT12 = mT12i;
				mBestRotation = mR12i;
				mBestTranslation = mt12i;
				mBestScale = ms12i;

				if (mnInliersi > mRansacMinInliers)
				{
					nInliers = mnInliersi;
					for (int i = 0; i < N; ++i)
						if (mvbInliersi[i])
							vbInliers[mvnIndices1[i]] = true;
					result = mBestT12;

					return true;
				}
			}
		}

		if (mnIterations >= mRansacMaxIts)
			bNoMore = true;

		return false;
	}

	bool cSim3Solver::find(std::vector<bool> &vbInliers12,
		int &nInliers,
		cv::Matx44d& result)
	{
		bool bFlag;
		return iterate(mRansacMaxIts, bFlag, vbInliers12, nInliers, result);
	}

	void cSim3Solver::centroid(cv::Matx33d &P,
		cv::Matx33d &Pr,
		cv::Vec3d &C)
	{
		C = cv::Vec3d(0.0, 0.0, 0.0);
		for (int i = 0; i < 3; ++i)
		{
			C(0) += P(0, i);
			C(1) += P(1, i);
			C(2) += P(2, i);
		}
		C /= 3.0;

		Pr = cv::Matx33d::zeros();
		for (int i = 0; i < P.cols; ++i)
		{
			Pr(0, i) = P(0, i) - C(0);
			Pr(1, i) = P(1, i) - C(1);
			Pr(2, i) = P(2, i) - C(2);
		}
	}

	void cSim3Solver::computeT(cv::Matx33d &P1,
		cv::Matx33d &P2)
	{
		// Custom implementation of:
		// Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

		// Step 1: Centroid and relative coordinates

		cv::Matx33d Pr1; // Relative coordinates to centroid (set 1)
		cv::Matx33d Pr2; // Relative coordinates to centroid (set 2)
		cv::Vec3d O1; // Centroid of P1
		cv::Vec3d O2; // Centroid of P2

		centroid(P1, Pr1, O1);
		centroid(P2, Pr2, O2);

		// Step 2: Compute M matrix

		cv::Matx33d M = Pr2*Pr1.t();
		// Step 3: Compute N matrix
		double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

		cv::Mat N(4, 4, CV_64FC1);

		N11 = M(0, 0) + M(1, 1) + M(2, 2);
		N12 = M(1, 2) - M(2, 1);
		N13 = M(2, 0) - M(0, 2);
		N14 = M(0, 1) - M(1, 0);
		N22 = M(0, 0) - M(1, 1) - M(2, 2);
		N23 = M(0, 1) + M(1, 0);
		N24 = M(2, 0) + M(0, 2);
		N33 = -M(0, 0) + M(1, 1) - M(2, 2);
		N34 = M(1, 2) + M(2, 1);
		N44 = -M(0, 0) - M(1, 1) + M(2, 2);

		N = (cv::Mat_<double>(4, 4) << N11, N12, N13, N14,
			N12, N22, N23, N24,
			N13, N23, N33, N34,
			N14, N24, N34, N44);
		// Step 4: Eigenvector of the highest eigenvalue
		cv::Mat eval, evec;

		cv::eigen(N, eval, evec); //evec[0] is the quaternion of the desired rotation
		cv::Mat vec(1, 3, evec.type());
		(evec.row(0).colRange(1, 4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

		// Rotation angle. sin is the norm of the imaginary part, cos is the real part
		double ang = atan2(norm(vec), evec.ptr<double>(0)[0]);

		vec = 2 * ang*vec / norm(vec); //Angle-axis representation. quaternion angle is the half

		mR12i = cv::Matx33d::eye();

		cv::Rodrigues(vec, mR12i); // computes the rotation matrix from angle-axis

		// Step 5: Rotate set 2
		cv::Matx33d P3 = mR12i*Pr2;

		// Step 6: Scale

		double nom = Pr1.dot(P3);
		cv::Matx33d aux_P3;
		aux_P3 = P3;
		cv::pow(P3, 2, aux_P3);
		double den = 0;

		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				den += aux_P3(i, j);

		ms12i = nom / den;

		// Step 7: Translation
		mt12i = O1 - ms12i*mR12i*O2;

		// Step 8: Transformation

		// Step 8.1 T12
		cv::Matx33d sR = ms12i*mR12i;
		mT12i = cConverter::Rt2Hom(sR, mt12i);

		// Step 8.2 T21
		cv::Matx33d sRinv = (1.0 / ms12i) * mR12i.t();
		cv::Vec3d tinv = -sRinv*mt12i;
		mT21i = cConverter::Rt2Hom(sRinv, tinv);
	}


	void cSim3Solver::CheckInliers()
	{
		//std::vector<cv::Vec2d> vP1im2(mvP1im1.size());
		//std::vector<cv::Vec2d> vP2im1(mvP1im1.size());

		//cv::Matx44d relOri = mpKF1->GetPoseInverse()*mpKF2->GetPose();

		mnInliersi = 0;
		for (size_t i = 0; i < mvP1im1.size(); ++i)
		{
			cv::Vec2d vP2im1(0.0, 0.0), vP1im2(0.0, 0.0);
			// rotate point from 1 to 2 and vice versa
			// transformation sequence: M_c^-1 * mT12i *  M_t^-1 * Pt3
			// first to MCS frame then from MCS frame 1 to MCS frame 2 and then to the corresponding camera
			cv::Matx<double, 4, 1> pt2_in_1 = mT12i * cConverter::toVec4d(mvX3Dc2[i]);
			cv::Matx<double, 4, 1> pt1_in_2 = mT21i * cConverter::toVec4d(mvX3Dc1[i]);

			cv::Matx<double, 4, 1> ptRot_in_1 = cConverter::invMat(camSysLocal->Get_M_c(camIdx1[i])) * pt2_in_1;
			cv::Matx<double, 4, 1> ptRot_in_2 = cConverter::invMat(camSysLocal->Get_M_c(camIdx2[i])) * pt1_in_2;

			camSysLocal->GetCamModelObj(camIdx1[i]).WorldToImg(
				ptRot_in_1(0), ptRot_in_1(1), ptRot_in_1(2), vP2im1(0), vP2im1(1));

			camSysLocal->GetCamModelObj(camIdx2[i]).WorldToImg(
				ptRot_in_2(0), ptRot_in_2(1), ptRot_in_2(2), vP1im2(0), vP1im2(1));

			cv::Vec2d dist1 = mvP1im1[i] - vP2im1;
			cv::Vec2d dist2 = vP1im2 - mvP2im2[i];

			double err1 = dist1.dot(dist1);
			double err2 = dist2.dot(dist2);

			if (err1 < mvnMaxError1[i] &&
				err2 < mvnMaxError2[i])
			{
				mvbInliersi[i] = true;
				++mnInliersi;
			}
			else
				mvbInliersi[i] = false;
		}
	}


	cv::Matx33d cSim3Solver::GetEstimatedRotation()
	{
		return mBestRotation;
	}

	cv::Vec3d cSim3Solver::GetEstimatedTranslation()
	{
		return mBestTranslation;
	}

	double cSim3Solver::GetEstimatedScale()
	{
		return mBestScale;
	}

}
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

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "cMapPoint.h"
#include "cMultiKeyFrame.h"
#include "cMultiFrame.h"

namespace MultiColSLAM
{
	const bool checkOrientation = false;

	// faster using popcount64 on uint64_t, only desc_dimension/8 loop iterations
	int DescriptorDistance64(const uint64_t* descr_i,
		const uint64_t* descr_j,
		const int& dim);

	// masked distance
	int DescriptorDistance64Masked(const uint64_t* descr_i,
		const uint64_t* descr_j,
		const uint64_t* mask_i,
		const uint64_t* mask_j,
		const int& dim);

	class cORBmatcher
	{
	public:

		cORBmatcher(double nnratio = 0.6,
			bool checkOri = true,
			const int featDim = 32,
			bool havingMasks_ = false);

		// Computes the Hamming distance between two ORB descriptors
		static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

		// Search matches between Frame keypoints and projected MapPoints. Returns number of matches
		// Used to track the local map (Tracking)
		int SearchByProjection(cMultiFrame &F,
			const std::vector<cMapPoint*> &vpMapPoints,
			const double th = 3);

		// Project MapPoints tracked in last frame into the current frame and search matches.
		// Used to track from previous frame (Tracking)
		int SearchByProjection(cMultiFrame &CurrentFrame, const cMultiFrame &LastFrame, double th);

		// Project MapPoints seen in KeyFrame into the Frame and search matches.
		// Used in relocalisation (Tracking)
		int SearchByProjection(cMultiFrame &CurrentFrame, cMultiKeyFrame* pKF,
			const std::set<cMapPoint*> &sAlreadyFound, double th, int ORBdist);

		// Project MapPoints using a Similarity Transformation and search matches.
		// Used in loop detection (Loop Closing)
		int SearchByProjection(cMultiKeyFrame* pKF,
			cv::Matx44d Scw,
			const std::vector<cMapPoint*> &vpPoints,
			std::vector<cMapPoint*> &vpMatched,
			int th);

		// Search matches between MapPoints in a KeyFrame and ORB in a Frame.
		// Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
		// Used in Relocalisation and Loop Detection
		int SearchByBoW(cMultiKeyFrame *pKF, cMultiFrame &F,
			std::vector<cMapPoint*> &vpMapPointMatches);
		int SearchByBoW(cMultiKeyFrame *pKF1, cMultiKeyFrame* pKF2,
			std::vector<cMapPoint*> &vpMatches12);

		// Search MapPoints tracked in Frame1 in Frame2 in a window centered at their position in Frame1
		int WindowSearch(cMultiFrame &F1, cMultiFrame &F2, int windowSize,
			std::vector<cMapPoint *> &vpMapPointMatches2,
			int minOctave = -1, int maxOctave = INT_MAX);

		// Refined matching when we have a guess of Frame 2 pose
		int SearchByProjection(cMultiFrame &F1, cMultiFrame &F2,
			int windowSize, std::vector<cMapPoint *> &vpMapPointMatches2);

		// Matching for the Map Initialization
		int SearchForInitialization(cMultiFrame &F1, cMultiFrame &F2,
			std::vector<cv::Vec2d> &vbPrevMatched,
			std::vector<int> &vnMatches12, int windowSize = 10);

		int SearchForTriangulationRaw(cMultiKeyFrame *pKF1,
			cMultiKeyFrame *pKF2,
			std::vector<cv::KeyPoint> &vMatchedKeys1,
			std::vector<cv::Vec3d> &vMatchedKeysRays1,
			std::vector<cv::KeyPoint> &vMatchedKeys2,
			std::vector<cv::Vec3d> &vMatchedKeysRays2,
			std::vector<std::pair<size_t, size_t> > &vMatchedPairs);

		// Matching to triangulate new MapPoints. Check Epipolar Constraint
		int SearchForTriangulation(cMultiKeyFrame *pKF1, cMultiKeyFrame *pKF2,
			std::vector<cv::KeyPoint> &vMatchedKeys1,
			std::vector<cv::Vec3d> &vMatchedKeysRays1,
			std::vector<cv::KeyPoint> &vMatchedKeys2,
			std::vector<cv::Vec3d> &vMatchedKeysRays2,
			std::vector<std::pair<size_t, size_t> > &vMatchedPairs);

		int SearchForTriangulationBetweenCameras(cMultiKeyFrame *pKF1,
			const int cam1, const int cam2,
			std::vector<cv::KeyPoint> &vMatchedKeys1,
			std::vector<cv::Vec3d> &vMatchedKeysRays1,
			std::vector<cv::KeyPoint> &vMatchedKeys2,
			std::vector<cv::Vec3d> &vMatchedKeysRays2,
			std::vector<std::pair<size_t, size_t> > &vMatchedPairs);

		// Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
		int SearchBySim3(cMultiKeyFrame *pKF1, cMultiKeyFrame *pKF2,
			vector<cMapPoint*> &vpMatches12,
			const double &s12,
			const cv::Matx33d &R12,
			const cv::Vec3d &t12,
			double th);

		// Project MapPoints into KeyFrame and search for duplicated MapPoints.
		int Fuse(cMultiKeyFrame* pKF,
			cMultiKeyFrame* curKF,
			std::vector<cMapPoint *> &vpMapPoints,
			double th = 2.5);
		int Fuse(cMultiKeyFrame* curKF,
			std::vector<cMultiKeyFrame*> neighKFs,
			std::unordered_map<cMapPoint*, int> &vpMapPoints,
			double th = 2.5);
		int Fuse(cMultiKeyFrame* curKF,
			std::vector<cMapPoint*> &vpMapPoints,
			double th = 2.5);
		// Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
		int Fuse(cMultiKeyFrame* pKF, cv::Matx44d Scw,
			const std::vector<cMapPoint*> &vpPoints,
			double th = 2.5);

		static const int HISTO_LENGTH;
		int TH_HIGH_;
		int TH_LOW_;

	protected:

		double RadiusByViewingCos(const double &viewCos);

		void ComputeThreeMaxima(std::vector<int>* histo,
			const int L, int &ind1, int &ind2, int &ind3);

		double mfNNratio;
		bool mbCheckOrientation;
		bool havingMasks;
		int mbFeatDim;
	};

}
#endif // ORBMATCHER_H

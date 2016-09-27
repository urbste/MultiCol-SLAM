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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <opencv2/core/core.hpp>
#include "cMultiKeyFrame.h"
#include "cMap.h"

#include <mutex>
namespace MultiColSLAM
{

	class ImageFeature;
	class cMultiKeyFrame;
	class cMap;
	class cCamModelGeneral_;
	class cMultiCamSys_;

	class cMapPoint
	{
	public:
		cMapPoint(const cv::Vec3d &Pos, cMultiKeyFrame* pRefKF, cMap* pMap);

		void SetWorldPos(const cv::Vec3d &Pos);
		cv::Vec3d GetWorldPos();

		cv::Vec3d GetNormal();
		cMultiKeyFrame* GetReferenceKeyFrame();

		// each point can be oberved from multiple cameras in the multi cam system
		std::map<cMultiKeyFrame*, std::vector<size_t>> GetObservations();
		int Observations();
		int TotalNrObservations();

		void AddObservation(cMultiKeyFrame* pKF, const size_t& idx);
		void EraseObservation(cMultiKeyFrame* pKF, const size_t& idx);
		void EraseAllObservations(cMultiKeyFrame* pKF);

		std::vector<size_t> GetIndexInKeyFrame(cMultiKeyFrame* pKF);
		bool IsInKeyFrame(cMultiKeyFrame* pKF);

		void SetBadFlag();
		void SetScaledFlag();
		bool isScaled();
		bool isBad();

		void Replace(cMapPoint* pMP);

		void IncreaseVisible();
		void IncreaseFound();
		void IncreaseVisible(const int& val);
		void IncreaseFound(const int& val);
		double GetFoundRatio();

		void ComputeDistinctiveDescriptors(bool havingMasks = false);

		cv::Mat GetDescriptor();
		cv::Mat GetCurrentDescriptor();
		cv::Mat GetDescriptorMask();
		const uint64_t* GetDescriptorPtr();
		const uint64_t* GetDescriptorMaskPtr();
		const uint64_t* GetCurrentDescriptorPtr();
		void UpdateCurrentDescriptor(cv::Mat& currDesc);

		void UpdateNormalAndDepth();

		double GetMinDistanceInvariance();
		double GetMaxDistanceInvariance();

	public:
		long unsigned int mnId;
		static long unsigned int nNextId;
		long int mnFirstKFid;

		// Variables used by the tracking
		std::vector<double> mTrackProjX; // for each cam
		std::vector<double> mTrackProjY;
		std::vector<bool> mbTrackInView;
		std::vector<int> mnTrackScaleLevel;
		std::vector<double> mTrackViewCos;
		long unsigned int mnTrackReferenceForFrame;
		long unsigned int mnLastFrameSeen;

		// Variables used by local mapping
		long unsigned int mnBALocalForKF;
		long unsigned int mnFuseCandidateForKF;

		// Variables used by loop closing
		long unsigned int mnLoopPointForKF;
		long unsigned int mnCorrectedByKF;
		long unsigned int mnCorrectedReference;

	protected:

		// Position in absolute coordinates
		cv::Vec3d mWorldPos;
		cv::Vec3d mModelPos;
		// Keyframes observing the point and associated indeces in keyframe
		std::map<cMultiKeyFrame*, std::vector<size_t>> mObservations;

		// Mean viewing direction
		cv::Vec3d mNormalVector;
		// Best descriptor to fast matching
		cv::Mat mDescriptor;
		cv::Mat mCurrentDescriptor;
		cv::Mat mDescriptorMask; // learned mask of the descriptor

		double meanPtError;
		double sigmaX;
		double sigmaY;
		double sigmaZ;

		// Reference KeyFrame
		cMultiKeyFrame* mpRefKF;

		// Tracking counters
		int mnVisible;
		int mnFound;

		// Bad flag (we do not currently erase MapPoint from memory)
		bool mbBad;
		bool scaled;

		// Scale invariance distances
		double mfMinDistance;
		double mfMaxDistance;

		cMap* mpMap;

		std::mutex mMutexPos;
		std::mutex mMutexFeatures;


	};
}
#endif // MAPPOINT_H

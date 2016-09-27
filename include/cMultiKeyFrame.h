// The original version was released under the following license
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

#ifndef MULTIKEYFRAME_H
#define MULTIKEYFRAME_H


// external
#include <algorithm>
#include <unordered_map>
#include <thread>
#include <mutex>
// third party
#include "DBoW2/DBoW2/BowVector.h"
#include "DBoW2/DBoW2/FeatureVector.h"
// ówn includes
#include "cam_system_omni.h"
#include "misc.h"
#include "cMapPoint.h"
#include "cORBVocabulary.h"
#include "cMultiFrame.h"
#include "cMultiKeyFrameDatabase.h"

namespace MultiColSLAM
{
	class cMap;
	class cMapPoint;
	class cMultiFrame;
	class cMultiKeyFrameDatabase;
	class DatabaseResult;
	class cCamModelGeneral_;
	class cMultiCamSys_;

	class cMultiKeyFrame
	{
	public:
		cMultiKeyFrame(cMultiFrame &F,
			cMap* pMap,
			cMultiKeyFrameDatabase* pKFDB);

		// Pose functions
		void SetPose(const cv::Matx33d &Rcw,
			const cv::Vec3d &tcw);
		void SetPose(const cv::Matx44d &Tcw);
		void SetPose(const cv::Matx61d &Tcw_min_);
		cv::Matx44d GetPose();
		cv::Matx44d GetPoseInverse();
		cv::Vec3d GetCameraCenter();
		cv::Matx33d GetRotation();
		cv::Vec3d GetTranslation();

		// Calibration
		// Bag of Words Representation
		void ComputeBoW();
		DBoW2::FeatureVector GetFeatureVector();
		DBoW2::FeatureVector GetFeatureVector(int& c);
		DBoW2::BowVector GetBowVector();
		DBoW2::BowVector GetBowVector(int& c);

		// Covisibility graph functions
		void AddConnection(cMultiKeyFrame* pKF, const int &weight);
		void EraseConnection(cMultiKeyFrame* pKF);
		void UpdateConnections();
		void UpdateBestCovisibles();
		std::set<cMultiKeyFrame*> GetConnectedKeyFrames();
		std::vector<cMultiKeyFrame*> GetVectorCovisibleKeyFrames();
		std::vector<cMultiKeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
		std::vector<cMultiKeyFrame*> GetCovisiblesByWeight(const int &w);
		int GetWeight(cMultiKeyFrame* pKF);

		// Spanning tree functions
		void AddChild(cMultiKeyFrame* pKF);
		void EraseChild(cMultiKeyFrame* pKF);
		void ChangeParent(cMultiKeyFrame* pKF);
		std::set<cMultiKeyFrame*> GetChilds();
		cMultiKeyFrame* GetParent();
		bool hasChild(cMultiKeyFrame* pKF);

		// Loop Edges
		void AddLoopEdge(cMultiKeyFrame* pKF);
		std::set<cMultiKeyFrame*> GetLoopEdges();

		// MapPoint observation functions
		void AddMapPoint(cMapPoint* pMP, const size_t &idx);
		void EraseMapPointMatch(const size_t &idx);
		//void EraseMapPointMatch(cMapPoint* pMP,const size_t& idx);
		void ReplaceMapPointMatch(const size_t &idx, cMapPoint* pMP);
		std::set<cMapPoint*> GetMapPoints();
		std::vector<cMapPoint*> GetMapPointMatches();
		int TrackedMapPoints();
		cMapPoint* GetMapPoint(const size_t &idx);

		// KeyPoint functions
		cv::KeyPoint GetKeyPoint(const size_t &idx) const; // 2D Point
		cv::Vec3d GetKeyPointRay(const size_t &idx) const; // 3D ray

		// keypoint functions
		int GetKeyPointScaleLevel(const size_t &idx) const;
		std::vector<cv::KeyPoint>  GetKeyPoints() const;
		std::vector<cv::Vec3d> GetKeyPointsRays() const;

		// descriptor get functions
		cv::Mat GetDescriptor(const int& cam, const size_t &idx) const;
		const uint64_t* GetDescriptorRowPtr(const int& cam, const size_t &idx) const; // this is waaaayy faster
		std::vector<cv::Mat> GetAllDescriptors() const { return mDescriptors; }
		std::vector<cv::Mat> GetAllDescriptorMasks() const { return mDescriptorMasks; }
		cv::Mat GetDescriptors(int c) const { return mDescriptors[c]; }
		cv::Mat GetDescriptorMask(const int& cam, const size_t &idx) const;
		cv::Mat GetDescriptorsMasks(int c) const { return mDescriptorMasks[c]; }
		const uint64_t* GetDescriptorMaskRowPtr(const int& cam, const size_t &idx) const; // this is waaaayy faster

		std::vector<size_t> GetFeaturesInArea(const int& cam, const double &x,
			const double  &y, const double  &r) const;

		// Image
		cv::Mat GetImage(const int& cam);
		std::vector<cv::Mat> GetAllImages() { return images; }
		bool IsInImage(const int& cam, const double &x, const double &y) const;

		// Activate/deactivate erasable flags
		void SetNotErase();
		void SetErase();

		// Set/check erased
		void SetBadFlag();
		bool isBad();

		// Scale functions
		double inline GetScaleFactor(int nLevel = 1) const{
			return mvScaleFactors[nLevel];
		}
		std::vector<double> inline GetScaleFactors() const{
			return mvScaleFactors;
		}
		std::vector<double> inline GetVectorScaleSigma2() const{
			return mvLevelSigma2;
		}
		double inline GetSigma2(int nLevel = 1) const{
			return mvLevelSigma2[nLevel];
		}
		double inline GetInvSigma2(int nLevel = 1) const{
			return mvInvLevelSigma2[nLevel];
		}
		int inline GetScaleLevels() const{
			return mnScaleLevels;
		}

		// Median MapPoint depth
		double ComputeSceneMedianDepth(int q = 2);

		static long unsigned int nNextId;
		long unsigned int mnId;
		long unsigned int mnFrameId;

		double mTimeStamp;

		// Grid (to speed up feature matching)
		std::vector<int> mnGridCols;
		std::vector<int> mnGridRows;
		std::vector<double> mfGridElementWidthInv;
		std::vector<double> mfGridElementHeightInv;

		// Variables used by the tracking
		long unsigned int mnTrackReferenceForFrame;
		long unsigned int mnFuseTargetForKF;

		// Variables used by the local mapping
		long unsigned int mnBALocalForKF;
		long unsigned int mnBAFixedForKF;

		// Variables used by the keyframe database
		long unsigned int mnLoopQuery;
		int mnLoopWords;
		double mLoopScore;
		long unsigned int mnRelocQuery;
		int mnRelocWords;
		double mRelocScore;

		//BoW
		// for all cams combined
		DBoW2::BowVector mBowVec;
		// for each cam
		std::vector<DBoW2::BowVector> mBowVecs;

		static bool weightComp(int a, int b) { return a > b; }

		static bool lId(cMultiKeyFrame* pKF1, cMultiKeyFrame* pKF2){
			return pKF1->mnId < pKF2->mnId;
		}

		// Calibration, camera model, camera system
		// all poses are stored in this class
		cMultiCamSys_ camSystem;

		// this hashmap holds the mapping between keypoint ID and camera
		// it was observed in
		// [key_id : cam_id]
		std::unordered_map<size_t, int> keypoint_to_cam;
		// this hashmap holds the mapping between the continous indexing of all
		// descriptors and keypoints and the image wise indexes
		// it was observed in
		// [cont_id : local_image_id]
		std::unordered_map<size_t, int> cont_idx_to_local_cam_idx;

		// other infos/statistics
		size_t GetValidMapPointCnt();
		size_t GetNrKeypointsInFrame();

		// infos about the descriptors
		bool HavingMasks() { return masksLearned; }
		int DescDims() { return descDimension; }
		bool IsReference();
		void SetReference(const bool ref);
		bool IsLoopCandidate();
		void SetLoopCandidate(const bool ref);
		int imageId;

	protected:

		bool mdBRIEF;
		bool masksLearned;
		int descDimension;
		bool IamTheReference;
		bool IamLoopCandidate;
		bool havingEdgeMeasurements;

		// Original images
		std::vector<cv::Mat> images;

		// assign those boundaries for each invdividual image
		std::vector<int> mnMinX;
		std::vector<int> mnMinY;
		std::vector<int> mnMaxX;
		std::vector<int> mnMaxY;

		// KeyPoints, Descriptors, MapPoints vectors (all associated by an index)
		// keypoints are saved contiously, i.e. they are assigned to the corresponding camera
		// by an unordered_map
		std::vector<cv::KeyPoint> mvKeys;
		std::vector<cv::Vec3d> mvKeysRays;
		std::vector<cv::Mat> mDescriptors;
		std::vector<cv::Mat> mDescriptorMasks;
		std::vector<cMapPoint*> mvpMapPoints;

		// BoW
		cMultiKeyFrameDatabase* mpKeyFrameDB;
		ORBVocabulary* mpORBvocabulary;

		DBoW2::FeatureVector mFeatVec;
		std::vector<DBoW2::FeatureVector> mFeatVecs;

		// Grid over all images to speed up feature matching
		std::vector< std::vector< std::vector< std::vector< std::size_t > > > > mGrids;

		std::map<cMultiKeyFrame*, int> mConnectedKeyFrameWeights;
		std::vector<cMultiKeyFrame*> mvpOrderedConnectedKeyFrames;
		std::vector<int> mvOrderedWeights;

		// Spanning Tree and Loop Edges
		bool mbFirstConnection;
		cMultiKeyFrame* mpParent;
		std::set<cMultiKeyFrame*> mspChildrens;
		std::set<cMultiKeyFrame*> mspLoopEdges;

		// Erase flags
		bool mbNotErase;
		bool mbToBeErased;
		bool mbBad;

		// Scale
		int mnScaleLevels;
		std::vector<double> mvScaleFactors;
		std::vector<double> mvLevelSigma2;
		std::vector<double> mvInvLevelSigma2;

		cMap* mpMap;

		std::mutex mMutexPose;
		std::mutex mMutexConnections;
		std::mutex mMutexFeatures;
		std::mutex mMutexImage;
		std::mutex mMutexRenderedImages;
		std::mutex mMutexEdgeImages;
		std::mutex mMutexModelPts;
		std::mutex mMutexProperties;
	};

}
#endif // KEYFRAME_H

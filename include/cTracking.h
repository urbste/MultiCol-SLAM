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

#ifndef TRACKING_H
#define TRACKING_H


#include "cMultiFramePublisher.h"
#include "cMap.h"
#include "cLocalMapping.h"
#include "cLoopClosing.h"
#include "cMultiFrame.h"
#include "cORBVocabulary.h"
#include "cMultiKeyFrameDatabase.h"
#include "mdBRIEFextractorOct.h"
#include "cMultiInitializer.h"
#include "cMapPublisher.h"
#include "cSystem.h"
#include "cViewer.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

namespace MultiColSLAM
{
	class cMultiFramePublisher;
	class cMap;
	class cLocalMapping;
	class cLoopClosing;
	class cSystem;
	class cViewer;

	class cTracking
	{

	public:
		cTracking(cSystem* pSys,
			ORBVocabulary* pVoc,
			cMultiFramePublisher *pFramePublisher,
			cMapPublisher *pMapPublisher,
			cMap *pMap,
			cMultiKeyFrameDatabase* pKFDB,
			cMultiCamSys_ camSystem,
			std::string settingsPath_);

		enum eTrackingState
		{
			SYSTEM_NOT_READY = -1,
			NO_IMAGES_YET = 0,
			NOT_INITIALIZED = 1,
			INITIALIZING = 2,
			WORKING = 3,
			LOST = 4
		};

		void SetLocalMapper(cLocalMapping* pLocalMapper);
		void SetLoopClosing(cLoopClosing* pLoopClosing);
		void SetKeyFrameDatabase(cMultiKeyFrameDatabase* pKFDB);
		void SetViewer(cViewer* pViewer);

		cv::Matx44d GrabImageSet(const std::vector<cv::Mat>& imgSet,
			const double& timestamp);

		void ForceRelocalisation();

		eTrackingState mState;
		eTrackingState mLastProcessedState;

		// Current Frame
		cMultiFrame mCurrentFrame;

		// Initialization Variables
		std::vector<int> mvIniLastMatches;
		std::vector<int> mvIniMatches;
		std::vector<cv::Vec2d> mvbPrevMatched;
		std::vector<cv::Vec3d> mvIniP3D;
		cMultiFrame mInitialFrame;

		void CheckResetByPublishers();

		std::vector<cv::Matx61d> GetAllPoses() { return this->allPoses; }
		std::vector<bool> GetAllPosesBool() { return this->allPosesBool; }
		std::vector<int> GetNrTrackedPts() { return nrTrackedPts; }
		std::vector<double> GetInlierRatio() { return inlierRatio; }

		std::vector<double> timingFeatureExtraction;
		std::vector<double> timingTrackLocalMap;
		std::vector<double> timingInitalPoseEst;

		bool CheckFinished();
		void Reset();

		int GetNrCams();
	protected:

		std::vector<std::vector<std::string>> imagePaths;
		int nrImages2Track;
		std::string imgPath;
		int imgCounter;
		bool finished;
		bool grab;
		int numberCameras;

		cv::Matx44d initPose;
		double curBaseline2MKF;
		bool Track();

		void FirstInitialization();
		void Initialize();
		void CreateInitialMap(cv::Matx33d &Rcw,
			cv::Vec3d &tcw, int leadingCam);

		bool TrackPreviousFrame();
		bool TrackWithMotionModel();

		bool RelocalisationRequested();
		bool Relocalisation();

		void UpdateReference();
		void UpdateReferencePoints();
		void UpdateReferenceKeyFrames();

		bool TrackLocalMap();
		int SearchReferencePointsInFrustum();

		bool NeedNewKeyFrame();
		void CreateNewKeyFrame();
		void CountNumberTrackedPointsPerCam();
		std::vector<int> nbTrackedPtsInCam;
		std::vector<double> nbTrackedRatios;

		//Other Thread Pointers
		cLocalMapping* mpLocalMapper;
		cLoopClosing* mpLoopClosing;
		cSystem* mpSystem;

		// mdBRIEF with octree
		std::vector<mdBRIEFextractorOct*> mp_mdBRIEF_extractorOct;
		std::vector<mdBRIEFextractorOct*> mp_mdBRIEF_init_extractorOct;

		//BoW
		ORBVocabulary* mpORBVocabulary;
		cMultiKeyFrameDatabase* mpKeyFrameDB;

		// Initalization
		cMultiInitializer* mpInitializer;

		//Local Map
		cMultiKeyFrame* mpReferenceKF;
		std::vector<cMultiKeyFrame*> mvpLocalKeyFrames;
		std::vector<int> mvpLocalKeyFramesCovWeights;
		std::vector<double> mvpLocalKeyFramesDistance2Frame;
		std::vector<cMapPoint*> mvpLocalMapPoints;

		//Publishers
		cMultiFramePublisher* mpFramePublisher;
		cMapPublisher* mpMapPublisher;
		cViewer* mpViewer;

		//Map
		cMap* mpMap;

		// camera system class
		cMultiCamSys_ camSystem;

		//New KeyFrame rules (according to fps)
		int mMinFrames;
		int mMaxFrames;

		//Current matches in frame
		int mnMatchesInliers;

		//Last Frame, KeyFrame and Relocalisation Info
		cMultiKeyFrame* mpLastKeyFrame;
		cMultiFrame mLastFrame;
		unsigned int mnLastKeyFrameId;
		unsigned int mnLastRelocFrameId;

		//Mutex
		std::mutex mMutexTrack;
		std::mutex mMutexForceRelocalisation;

		//Reset
		bool mbPublisherStopped;
		bool mbReseting;
		std::mutex mMutexReset;

		//Is relocalisation requested by an external thread? (loop closing)
		bool mbForceRelocalisation;

		//Motion Model
		bool mbMotionModel;
		cv::Matx44d mVelocity;

		//Color order (true RGB, false BGR, ignored if grayscale)
		bool mbRGB;

		string settingsPath;

		// evaluation
		std::vector<cv::Matx61d> allPoses;
		std::vector<bool> allPosesBool;
		std::vector<int> nrTrackedPts;
		std::vector<double> inlierRatio;
		// features
		bool use_mdBRIEF;
		bool loopAndMapperSet;
	};
}
#endif // TRACKING_H

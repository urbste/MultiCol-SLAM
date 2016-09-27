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

#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "cTracking.h"
#include "cMultiFramePublisher.h"
#include "cMapPublisher.h"
#include "cMap.h"
#include "cLocalMapping.h"
#include "cLoopClosing.h"
#include "cMultiKeyFrameDatabase.h"
#include "cORBVocabulary.h"
#include "cam_system_omni.h"
#include "cViewer.h"

namespace MultiColSLAM
{
	class cMultiFramePublisher;
	class cMap;
	class cMapPublisher;
	class cTracking;
	class cLocalMapping;
	class cLoopClosing;
	class cMultiCamSys_;
	class cViewer;

	class cSystem
	{
	public:

		// Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
		cSystem(const string &strVocFile, const string &strSettingsFile,
			const string& path2MCScalibrationFiles, const bool bUseViewer = true);

		void LoadMCS(const string path2calibrations, cMultiCamSys_& camSystem);

		// Proccess the given frames. Images must be synchronized. The camera calibration needs to be known
		// Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
		// Returns the camera pose (empty if tracking fails).
		cv::Matx44d TrackMultiColSLAM(const std::vector<cv::Mat>& imgSet, const double &timestamp);

		// This stops local mapping thread (map building) and performs only camera tracking.
		void ActivateLocalizationMode();
		// This resumes local mapping thread and performs SLAM again.
		void DeactivateLocalizationMode();

		// Reset the system (clear map)
		void Reset();

		// All threads will be requested to finish.
		// It waits until all threads have finished.
		// This function must be called before saving the trajectory.
		void Shutdown();

		// Save camera trajectory in the.
		// Call first Shutdown()
		// See format details at: http://www.ipf.kit.edu/lafida.php
		void SaveMKFTrajectoryLAFIDA(const string &filename);

	private:

		// ORB vocabulary used for place recognition and feature matching.
		ORBVocabulary* mpVocabulary;

		// KeyFrame database for place recognition (relocalization and loop detection).
		cMultiKeyFrameDatabase* mpKeyFrameDatabase;

		// Map structure that stores the pointers to all KeyFrames and MapPoints.
		cMap* mpMap;

		// Tracker. It receives a frame and computes the associated camera pose.
		// It also decides when to insert a new keyframe, create some new MapPoints and
		// performs relocalization if tracking fails.
		cTracking* mpTracker;

		// Local Mapper. It manages the local map and performs local bundle adjustment.
		cLocalMapping* mpLocalMapper;

		// Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
		// a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
		cLoopClosing* mpLoopCloser;

		// The viewer draws the map and the current camera pose. It uses Pangolin.
		cViewer* mpViewer;
		cMapPublisher* mpMapPublisher;
		cMultiFramePublisher* mpMultiFramePublisher;

		// System threads: Local Mapping, Loop Closing, Viewer.
		// The Tracking thread "lives" in the main execution thread that creates the System object.
		std::thread* mptLocalMapping;
		std::thread* mptLoopClosing;
		std::thread* mptViewer;

		// Reset flag
		std::mutex mMutexReset;
		bool mbReset;

		// Change mode flags
		std::mutex mMutexMode;
		bool mbActivateLocalizationMode;
		bool mbDeactivateLocalizationMode;
	};


}
#endif // SYSTEM_H
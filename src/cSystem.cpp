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

#include "cSystem.h"
#include "cConverter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>


namespace MultiColSLAM
{
	using namespace std;

	cSystem::cSystem(const string &strVocFile, const string &strSettingsFile,
		const string& path2MCScalibrationFiles,
		const bool bUseViewer) : mbReset(false), mbActivateLocalizationMode(false),
		mbDeactivateLocalizationMode(false)
	{
		// Output welcome message
		cout << endl <<
			"MultiCol-SLAM Copyright (C) 2015-2016 Steffen Urban." << endl <<
			"This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
			"This is free software, and you are welcome to redistribute it" << endl <<
			"under certain conditions. See LICENSE.txt." << endl << endl;

		//Check settings file
		cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
		if (!fsSettings.isOpened())
		{
			cerr << "Failed to open settings file at: " << strSettingsFile << endl;
			exit(-1);
		}
		//Load ORB Vocabulary
		cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

		cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
		ORBVocabulary Vocabulary;
		Vocabulary.load(fsVoc);
		mpVocabulary = new ORBVocabulary(Vocabulary);
		//mpVocabulary->loadFromTextFile
		fsVoc.release();
		//bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
		if (mpVocabulary->size() <= 0)
		{
			cerr << "Wrong path to vocabulary. " << endl;
			cerr << "Falied to open at: " << strVocFile << endl;
			exit(-1);
		}
		cout << "Vocabulary loaded!" << endl << endl;

		// load MCS
		cout << endl << "Loading camera and MCS calibrations" << endl;
		cMultiCamSys_ camSystem;
		this->LoadMCS(path2MCScalibrationFiles, camSystem);

		//Create KeyFrame Database
		mpKeyFrameDatabase = new cMultiKeyFrameDatabase(*mpVocabulary);

		//Create the Map
		mpMap = new cMap();

		//Create Drawers. These are used by the Viewer
		mpMultiFramePublisher = new cMultiFramePublisher(mpMap);
		mpMapPublisher = new cMapPublisher(mpMap, strSettingsFile);

		//Initialize the Tracking thread
		//(it will live in the main thread of execution, the one that called this constructor)
		mpTracker = new cTracking(
			this, mpVocabulary, mpMultiFramePublisher, mpMapPublisher,
			mpMap, mpKeyFrameDatabase, camSystem, strSettingsFile);

		//Initialize the Local Mapping thread and launch
		mpLocalMapper = new cLocalMapping(mpMap);
		mptLocalMapping = new thread(&cLocalMapping::Run, mpLocalMapper);

		//Initialize the Loop Closing thread and launch
		mpLoopCloser = new cLoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary);
		mptLoopClosing = new thread(&cLoopClosing::Run, mpLoopCloser);

		//Initialize the Viewer thread and launch

		//Initialize the Viewer thread and launch
		mpViewer = new cViewer(this, mpMultiFramePublisher,
			mpMapPublisher, mpTracker, strSettingsFile);
		if (bUseViewer)
			mptViewer = new thread(&cViewer::Run, mpViewer);

		mpTracker->SetViewer(mpViewer);

		//Set pointers between threads
		mpTracker->SetLocalMapper(mpLocalMapper);
		mpTracker->SetLoopClosing(mpLoopCloser);

		mpLocalMapper->SetTracker(mpTracker);
		mpLocalMapper->SetLoopCloser(mpLoopCloser);

		mpLoopCloser->SetTracker(mpTracker);
		mpLoopCloser->SetLocalMapper(mpLocalMapper);
	}

	void cSystem::LoadMCS(const string path2calibrations,
		cMultiCamSys_& camSystem)
	{

		string mcs_settings = path2calibrations + "/MultiCamSys_Calibration.yaml";
		cv::FileStorage mcs_calib_data(mcs_settings, cv::FileStorage::READ);
		int nrCams = (int)mcs_calib_data["CameraSystem.nrCams"];
		std::vector<cv::Matx44d> M_c_s(nrCams);
		std::vector<cCamModelGeneral_> camModels(nrCams);
		for (int c = 0; c < nrCams; ++c)
		{
			// all M_c
			cv::Matx61d tmp;
			for (int p = 1; p < 7; ++p)
			{
				string param = "CameraSystem.cam" + to_string(c + 1) + "_" + to_string(p);
				tmp(p - 1) = mcs_calib_data[param];
			}
			M_c_s[c] = cayley2hom<double>(tmp);

			// Interior orientation
			string calib_data = path2calibrations + "/InteriorOrientationFisheye" + to_string(c) + ".yaml";
			cv::FileStorage fSettings(calib_data, cv::FileStorage::READ);
			int nrpol = (int)fSettings["Camera.nrpol"];
			int nrinvpol = (int)fSettings["Camera.nrinvpol"];
			cv::Mat_<double> poly = cv::Mat::zeros(5, 1, CV_64F);
			for (int i = 0; i < nrpol; ++i)
				poly.at<double>(i, 0) = fSettings["Camera.a" + to_string(i)];
			cv::Mat_<double>  invpoly = cv::Mat::zeros(12, 1, CV_64F);
			for (int i = 0; i < nrinvpol; ++i)
				invpoly.at<double>(i, 0) = fSettings["Camera.pol" + to_string(i)];

			int Iw = (int)fSettings["Camera.Iw"];
			int Ih = (int)fSettings["Camera.Ih"];

			double cdeu0v0[5] = { fSettings["Camera.c"], fSettings["Camera.d"], fSettings["Camera.e"],
				fSettings["Camera.u0"], fSettings["Camera.v0"] };

			cCamModelGeneral_ camModel = cCamModelGeneral_(cdeu0v0, poly, invpoly, Iw, Ih);
			vector<cv::Mat> mirrorMasks;
			CreateMirrorMask(camModel,
				4,
				mirrorMasks);
			camModel.SetMirrorMasks(mirrorMasks);
			camModels[c] = camModel;

		}
		// set up the cam system and initialize its first pose to identity
		camSystem = cMultiCamSys_(cv::Matx44d::eye(), M_c_s, camModels);
	}

	cv::Matx44d cSystem::TrackMultiColSLAM(
		const std::vector<cv::Mat>& imgSet, 
		const double &timestamp)
	{

		//	// Check mode change
		//{
		//	unique_lock<mutex> lock(mMutexMode);
		//	if (mbActivateLocalizationMode)
		//	{
		//		mpLocalMapper->RequestStop();
		//
		//		// Wait until Local Mapping has effectively stopped
		//		while (!mpLocalMapper->isStopped())
		//		{
		//			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		//		}
		//
		//		//mpTracker->InformOnlyTracking(true);
		//		mbActivateLocalizationMode = false;
		//	}
		//	if (mbDeactivateLocalizationMode)
		//	{
		//		//mpTracker->InformOnlyTracking(false);
		//		mpLocalMapper->Release();
		//		mbDeactivateLocalizationMode = false;
		//	}
		//}

		// Check reset
		{
			unique_lock<mutex> lock(mMutexReset);
			if (mbReset)
			{
				mpTracker->Reset();
				mbReset = false;
			}
		}

		return mpTracker->GrabImageSet(imgSet, timestamp);
	}

	void cSystem::ActivateLocalizationMode()
	{
		unique_lock<mutex> lock(mMutexMode);
		mbActivateLocalizationMode = true;
	}

	void cSystem::DeactivateLocalizationMode()
	{
		unique_lock<mutex> lock(mMutexMode);
		mbDeactivateLocalizationMode = true;
	}

	void cSystem::Reset()
	{
		unique_lock<mutex> lock(mMutexReset);
		mbReset = true;
	}

	void cSystem::Shutdown()
	{
		cout << "System shutdown" << endl;
		mpLocalMapper->RequestFinish();
		mpLoopCloser->RequestFinish();
		mpViewer->RequestFinish();

		cout << "Requested to finish local mapper, loop closer and map publisher" << endl;
		// Wait until all thread have effectively stopped
		while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
			!mpViewer->isFinished())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		cout << "All threads stopped..." << endl;
		pangolin::BindToContext("MultiCol-SLAM: Map Viewer");
	}

	void cSystem::SaveMKFTrajectoryLAFIDA(const string &filename)
	{
		cout << endl << "Saving MKF trajectory to " << filename << " ..." << endl;

		///////////////
		// Save keyframe poses at the end of the execution
		///////////////
		ofstream f;

		vector<cMultiKeyFrame*> vpMKFs = this->mpMap->GetAllKeyFrames();
		sort(vpMKFs.begin(), vpMKFs.end(), cMultiKeyFrame::lId);

		cout << endl << "Saving MultiKeyframe trajectory and all MCS poses with timestamps" << endl;
		f.open(filename.c_str());
		f << fixed;
		for (size_t i = 0; i < vpMKFs.size(); ++i)
		{
			cMultiKeyFrame* pKF = vpMKFs[i];

			if (pKF->isBad())
				continue;

			cv::Matx33d R = pKF->GetRotation().t();
			vector<double> q = cConverter::toQuaternion(R);
			cv::Vec3d t = pKF->GetCameraCenter();
			f << std::setprecision(10) << pKF->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
				<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

		}
		f.close();
	}
}
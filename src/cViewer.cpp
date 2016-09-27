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

#include "cViewer.h"
#ifndef _DEBUG
#include <pangolin/pangolin.h>
#endif
#include <mutex>

namespace MultiColSLAM
{

	cViewer::cViewer(
		cSystem* pSystem,
		cMultiFramePublisher* pFrameDrawer,
		cMapPublisher* pMapDrawer,
		cTracking *pTracking,
		const std::string &strSettingPath) :
		mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
		mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
	{
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

		float fps = fSettings["Camera.fps"];
		if (fps < 1)
			fps = 25;
		mT = 1e3 / fps;

		mImageWidth = fSettings["Camera.width"];
		mImageHeight = fSettings["Camera.height"];
		if (mImageWidth < 1 || mImageHeight < 1)
		{
			mImageWidth = 640;
			mImageHeight = 480;
		}

		mViewpointX = fSettings["Viewer.ViewpointX"];
		mViewpointY = fSettings["Viewer.ViewpointY"];
		mViewpointZ = fSettings["Viewer.ViewpointZ"];
		mViewpointF = fSettings["Viewer.ViewpointF"];

		drawNrCams = (int)fSettings["Viewer.DrawNrCams"];

		int nrCamsTracker = mpTracker->GetNrCams();
		if (drawNrCams > nrCamsTracker)
			drawNrCams = nrCamsTracker;
	}

	void cViewer::Run()
	{
		
		mbFinished = false;
#ifndef _DEBUG
		pangolin::CreateWindowAndBind("MultiCol-SLAM: Map Viewer", 1024, 768);

		// 3D Mouse handler requires depth testing to be enabled
		glEnable(GL_DEPTH_TEST);

		// Issue specific OpenGl we might need
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
		pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
		pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
		pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", false, true);
		pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
		pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
		pangolin::Var<bool> menuReset("menu.Reset", false, false);

		// Define Camera Render Object (for view / scene browsing)
		pangolin::OpenGlRenderState s_cam(
			pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
			pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
			);

		// Add named OpenGL viewport to window and provide 3D Handler
		pangolin::View& d_cam = pangolin::CreateDisplay()
			.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
			.SetHandler(new pangolin::Handler3D(s_cam));

		pangolin::OpenGlMatrix Twc;
		Twc.SetIdentity();
#endif
		for (int c = 0; c < drawNrCams; ++c)
			cv::namedWindow("MultiCol-SLAM: Current Frame: "+to_string(c));

		bool bFollow = true;
		bool bLocalizationMode = false;

		while (1)
		{
#ifndef _DEBUG
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			mpMapDrawer->GetCurrentOpenGLMCSPose(Twc);

			if (menuFollowCamera && bFollow)
			{
				s_cam.Follow(Twc);
			}
			else if (menuFollowCamera && !bFollow)
			{
				s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
				s_cam.Follow(Twc);
				bFollow = true;
			}
			else if (!menuFollowCamera && bFollow)
			{
				bFollow = false;
			}

			d_cam.Activate(s_cam);
			glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
			mpMapDrawer->PublishCurrentCamera(Twc);
			if (menuShowKeyFrames || menuShowGraph)
				mpMapDrawer->PublishMultiKeyFrames(menuShowKeyFrames, menuShowGraph);
			if (menuShowPoints)
				mpMapDrawer->PublishMapPoints();

			pangolin::FinishFrame();
#endif
			std::vector<cv::Mat> imgs;
			mpFrameDrawer->DrawMultiFrame(imgs);
			for (int c = 0; c < drawNrCams; ++c)
			{
				cv::imshow("MultiCol-SLAM: Current Frame: " + to_string(c), imgs[c]);
				cv::waitKey(1);
			}
			cv::waitKey(mT);
#ifndef _DEBUG
			if (menuReset)
			{
				menuShowGraph = true;
				menuShowKeyFrames = true;
				menuShowPoints = true;
				menuLocalizationMode = false;
				if (bLocalizationMode)
					mpSystem->DeactivateLocalizationMode();
				bLocalizationMode = false;
				bFollow = true;
				menuFollowCamera = true;
				mpSystem->Reset();
				menuReset = false;
			}
#endif
			if (Stop())
				while (isStopped())
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			if (CheckFinish())
				break;
		}

		SetFinish();
#ifndef _DEBUG
		pangolin::BindToContext("MultiCam SLAM: Map Viewer");
		pangolin::Quit();
#endif
	}

	void cViewer::RequestFinish()
	{
		unique_lock<mutex> lock(mMutexFinish);
		mbFinishRequested = true;
	}

	bool cViewer::CheckFinish()
	{
		unique_lock<mutex> lock(mMutexFinish);
		return mbFinishRequested;
	}

	void cViewer::SetFinish()
	{
		unique_lock<mutex> lock(mMutexFinish);
		mbFinished = true;
	}

	bool cViewer::isFinished()
	{
		unique_lock<mutex> lock(mMutexFinish);
		return mbFinished;
	}

	void cViewer::RequestStop()
	{
		unique_lock<mutex> lock(mMutexStop);
		if (!mbStopped)
			mbStopRequested = true;
	}

	bool cViewer::isStopped()
	{
		unique_lock<mutex> lock(mMutexStop);
		return mbStopped;
	}

	bool cViewer::Stop()
	{
		unique_lock<mutex> lock(mMutexStop);
		unique_lock<mutex> lock2(mMutexFinish);

		if (mbFinishRequested)
			return false;
		else if (mbStopRequested)
		{
			mbStopped = true;
			mbStopRequested = false;
			return true;
		}

		return false;

	}

	void cViewer::Release()
	{
		unique_lock<mutex> lock(mMutexStop);
		mbStopped = false;
	}

}
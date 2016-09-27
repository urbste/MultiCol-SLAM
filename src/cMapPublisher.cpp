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

#include "cMapPublisher.h"
#include "cMapPoint.h"
#include "cMultiKeyFrame.h"
#include "cConverter.h"

#include <thread>
#include <mutex>

namespace MultiColSLAM
{
	cMapPublisher::cMapPublisher(cMap* pMap,
		const string &strSettingPath) :
		mpMap(pMap),
		mbCameraUpdated(false),
		cnter(0),
		mbFinishRequested(false),
		mbFinished(false),
		laufSave(0),
		modelPointsSet(false),
		mbStopped(false),
		mbStopRequested(false)
	{
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		mMultiKeyFrameSize = fSettings["Viewer.MultiKeyFrameSize"];
		mMultiKeyFrameLineWidth = fSettings["Viewer.MultiKeyFrameLineWidth"];
		mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
		mPointSize = fSettings["Viewer.PointSize"];
		mCameraSize = fSettings["Viewer.CameraSize"];
		mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

	}

	void cMapPublisher::PublishMapPoints()
	{
#ifndef _DEBUG
		vector<cMapPoint*> vpMPs = mpMap->GetAllMapPoints();
		vector<cMapPoint*> vpRefMPs = mpMap->GetReferenceMapPoints();
		set<cMapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

		if (vpMPs.empty())
			return;

		glPointSize(mPointSize);
		glBegin(GL_POINTS);
		glColor3f(0.0, 0.0, 0.0);
		for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
		{
			if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
				continue;
			cv::Vec3d pos = vpMPs[i]->GetWorldPos();
			glVertex3f(pos(0), pos(1), pos(2));
		}
		glEnd();

		glPointSize(mPointSize);
		glBegin(GL_POINTS);
		glColor3f(1.0, 0.0, 0.0);

		for (set<cMapPoint*>::iterator sit = spRefMPs.begin(),
			send = spRefMPs.end(); sit != send; sit++)
		{
			if ((*sit)->isBad())
				continue;
			cv::Vec3d pos = (*sit)->GetWorldPos();

			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(pos(0), pos(1), pos(2));
		}

		glEnd();
#endif
	}

	void cMapPublisher::PublishMultiKeyFrames(const bool bDrawKF,
		const bool bDrawGraph)
	{
#ifndef _DEBUG
		const float &w = mMultiKeyFrameSize;
		const float h = w*0.75;
		const float z = w*0.6;

		const vector<cMultiKeyFrame*> vpMKFs = mpMap->GetAllKeyFrames();

		if (bDrawKF)
		{
			for (size_t i = 0; i < vpMKFs.size(); ++i)
			{
				cMultiKeyFrame* pMKF = vpMKFs[i];
				cv::Mat MKF = cConverter::toMat(pMKF->GetPose().t());

				glPushMatrix();

				glMultMatrixd((GLdouble*)MKF.data);

				glLineWidth(mMultiKeyFrameLineWidth);
				if (pMKF->IsLoopCandidate())
					glColor3f(1.0f, 0.0f, 0.0f);
				else
					glColor3f(0.0f, 0.5f, 0.5f);

				glBegin(GL_LINES);
				glVertex3f(0, 0, 0);
				glVertex3f(w, h, z);
				glVertex3f(0, 0, 0);
				glVertex3f(w, -h, z);
				glVertex3f(0, 0, 0);
				glVertex3f(-w, -h, z);
				glVertex3f(0, 0, 0);
				glVertex3f(-w, h, z);

				glVertex3f(w, h, z);
				glVertex3f(w, -h, z);

				glVertex3f(-w, h, z);
				glVertex3f(-w, -h, z);

				glVertex3f(-w, h, z);
				glVertex3f(w, h, z);

				glVertex3f(-w, -h, z);
				glVertex3f(w, -h, z);
				glEnd();

				glPopMatrix();
			}
		}

		if (bDrawGraph)
		{
			glLineWidth(mGraphLineWidth);
			glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
			glBegin(GL_LINES);

			for (size_t i = 0; i < vpMKFs.size(); i++)
			{
				// Covisibility Graph
				const vector<cMultiKeyFrame*> vCovKFs = vpMKFs[i]->GetCovisiblesByWeight(100);
				cv::Vec3d Ow = cv::Vec3f(vpMKFs[i]->GetCameraCenter());
				if (!vCovKFs.empty())
				{
					for (vector<cMultiKeyFrame*>::const_iterator vit = vCovKFs.begin(),
						vend = vCovKFs.end(); vit != vend; vit++)
					{
						if ((*vit)->mnId < vpMKFs[i]->mnId)
							continue;
						cv::Vec3f Ow2 = cv::Vec3f((*vit)->GetCameraCenter());
						glVertex3f(Ow(0), Ow(1), Ow(2));
						glVertex3f(Ow2(0), Ow2(1), Ow2(2));
					}
				}

				// Spanning tree
				cMultiKeyFrame* pParent = vpMKFs[i]->GetParent();
				if (pParent)
				{
					cv::Vec3f Owp = cv::Vec3f(pParent->GetCameraCenter());
					glVertex3f(Ow(0), Ow(1), Ow(2));
					glVertex3f(Owp(0), Owp(1), Owp(2));
				}

				// Loops
				std::set<cMultiKeyFrame*> sLoopKFs = vpMKFs[i]->GetLoopEdges();
				for (set<cMultiKeyFrame*>::iterator sit = sLoopKFs.begin(),
					send = sLoopKFs.end(); sit != send; sit++)
				{
					if ((*sit)->mnId < vpMKFs[i]->mnId)
						continue;
					cv::Vec3f Owl = cv::Vec3f((*sit)->GetCameraCenter());
					glVertex3f(Ow(0), Ow(1), Ow(2));
					glVertex3f(Owl(0), Owl(1), Owl(2));
				}
			}

			glEnd();
		}
#endif
	}

#ifndef _DEBUG
	void cMapPublisher::GetCurrentOpenGLMCSPose(
		pangolin::OpenGlMatrix& _Tcw)
	{
		_Tcw.SetIdentity();
		cv::Matx33d Rwc;
		cv::Matx31d twc;
		{
			unique_lock<mutex> lock(mMutexCamera);
			Rwc = mCameraPose.get_minor<3, 3>(0, 0);
			twc = mCameraPose.get_minor<3, 1>(0, 3);
		}

		_Tcw.m[0] = Rwc(0, 0);
		_Tcw.m[1] = Rwc(1, 0);
		_Tcw.m[2] = Rwc(2, 0);
		_Tcw.m[3] = 0.0;

		_Tcw.m[4] = Rwc(0, 1);
		_Tcw.m[5] = Rwc(1, 1);
		_Tcw.m[6] = Rwc(2, 1);
		_Tcw.m[7] = 0.0;

		_Tcw.m[8] = Rwc(0, 2);
		_Tcw.m[9] = Rwc(1, 2);
		_Tcw.m[10] = Rwc(2, 2);
		_Tcw.m[11] = 0.0;

		_Tcw.m[12] = twc(0, 0);
		_Tcw.m[13] = twc(1, 0);
		_Tcw.m[14] = twc(2, 0);
		_Tcw.m[15] = 1.0;

	}

	void cMapPublisher::PublishCurrentCamera(
		const pangolin::OpenGlMatrix& Tcw)
	{

		const float &w = mCameraSize;
		const float h = w*0.75;
		const float z = w*0.6;

		glPushMatrix();

		//glMultMatrixd(camPose.ptr<GLdouble>(0));
		glMultMatrixd(Tcw.m);

		glLineWidth(mCameraLineWidth);
		glColor3f(0.0f, 1.0f, 0.0f);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(w, h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(w, -h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(-w, -h, z);
		glVertex3f(0, 0, 0);
		glVertex3f(-w, h, z);

		glVertex3f(w, h, z);
		glVertex3f(w, -h, z);

		glVertex3f(-w, h, z);
		glVertex3f(-w, -h, z);

		glVertex3f(-w, h, z);
		glVertex3f(w, h, z);

		glVertex3f(-w, -h, z);
		glVertex3f(w, -h, z);
		glEnd();

		glPopMatrix();

		// to opencv mat
		cv::Matx44d ocvTcw = cv::Matx44d::eye();
		ocvTcw(0, 0) = Tcw.m[0];
		ocvTcw(1, 0) = Tcw.m[1];
		ocvTcw(2, 0) = Tcw.m[2];

		ocvTcw(0, 1) = Tcw.m[4];
		ocvTcw(1, 1) = Tcw.m[5];
		ocvTcw(2, 1) = Tcw.m[6];

		ocvTcw(0, 2) = Tcw.m[8];
		ocvTcw(1, 2) = Tcw.m[9];
		ocvTcw(2, 2) = Tcw.m[10];

		ocvTcw(0, 3) = Tcw.m[12];
		ocvTcw(1, 3) = Tcw.m[13];
		ocvTcw(2, 3) = Tcw.m[14];

		// publish all cameras
		for (int c = 0; c < mM_c_s.size(); ++c)
		{
			cv::Mat camEO = cConverter::toMat((ocvTcw * mM_c_s[c]).t());

			glPushMatrix();
			glMultMatrixd(camEO.ptr<GLdouble>(0));
			glLineWidth(mCameraLineWidth);
			glColor3f(0.0f, 0.5f, 0.5f);
			glBegin(GL_LINES);
			glVertex3f(0, 0, 0);
			glVertex3f(w, h, z);
			glVertex3f(0, 0, 0);
			glVertex3f(w, -h, z);
			glVertex3f(0, 0, 0);
			glVertex3f(-w, -h, z);
			glVertex3f(0, 0, 0);
			glVertex3f(-w, h, z);

			glVertex3f(w, h, z);
			glVertex3f(w, -h, z);

			glVertex3f(-w, h, z);
			glVertex3f(-w, -h, z);

			glVertex3f(-w, h, z);
			glVertex3f(w, h, z);

			glVertex3f(-w, -h, z);
			glVertex3f(w, -h, z);
			glEnd();

			glPopMatrix();
		}
	}
#endif

	void cMapPublisher::SetMCS(const std::vector<cv::Matx44d> M_c_s)
	{
		mM_c_s = M_c_s;
	}

	void cMapPublisher::SetCurrentCameraPose(const cv::Matx44d Tcw)
	{
		std::unique_lock<std::mutex> lock(mMutexCamera);
		mCameraPose = Tcw;
		mbCameraUpdated = true;
	}

	cv::Matx44d cMapPublisher::GetCurrentCameraPose()
	{
		std::unique_lock<std::mutex> lock(mMutexCamera);
		return mCameraPose;
	}

	bool cMapPublisher::isCamUpdated()
	{
		std::unique_lock<std::mutex> lock(mMutexCamera);
		return mbCameraUpdated;
	}

	void cMapPublisher::ResetCamFlag()
	{
		std::unique_lock<std::mutex> lock(mMutexCamera);
		mbCameraUpdated = false;
	}

	void cMapPublisher::RequestFinish()
	{
		std::unique_lock<std::mutex> lock(mMutexFinish);
		mbFinishRequested = true;
	}

	bool cMapPublisher::CheckFinish()
	{
		std::unique_lock<std::mutex> lock(mMutexFinish);
		return mbFinishRequested;
	}

	void cMapPublisher::SetFinish()
	{
		std::unique_lock<std::mutex> lock(mMutexFinish);
		mbFinished = true;
	}

	bool cMapPublisher::isFinished()
	{
		std::unique_lock<std::mutex> lock(mMutexFinish);
		return mbFinished;
	}

	void cMapPublisher::RequestStop()
	{
		unique_lock<mutex> lock(mMutexStop);
		if (!mbStopped)
			mbStopRequested = true;
	}

	bool cMapPublisher::isStopped()
	{
		unique_lock<mutex> lock(mMutexStop);
		return mbStopped;
	}

	bool cMapPublisher::Stop()
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

	void cMapPublisher::Release()
	{
		unique_lock<mutex> lock(mMutexStop);
		mbStopped = false;
	}
}
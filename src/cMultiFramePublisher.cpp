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

#include "cMultiFramePublisher.h"
#include "cTracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>

namespace MultiColSLAM
{
	cMultiFramePublisher::cMultiFramePublisher(cMap *pMap) :
		nrCams(1), mcsSet(false), cntWrite(0), mpMap()
	{
		mImages.push_back(cv::Mat::zeros(480, 640, CV_8UC1));
		mState = cTracking::SYSTEM_NOT_READY;

		mbUpdated = false;
	}

	bool cMultiFramePublisher::isMapUpdated()
	{
		std::unique_lock<std::mutex> lock(mMutex);
		return mbUpdated;
	}

	void cMultiFramePublisher::SetMCS(cMultiCamSys_* tmpSys)
	{
		nrCams = tmpSys->GetNrCams();
		mImages.clear();
		mImages.resize(nrCams);
		for (int c = 0; c < nrCams; ++c)
			mImages[c] = cv::Mat::zeros(tmpSys->GetCamModelObj(c).GetHeight(),
			tmpSys->GetCamModelObj(c).GetWidth(), CV_8UC3);

		mcsSet = true;
	}

	void cMultiFramePublisher::SetMap(cMap *pMap)
	{
		mpMap = pMap;
	}

	void cMultiFramePublisher::DrawMultiFrame(std::vector<cv::Mat>& imgs)
	{
		cv::Mat imWithInfo = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
		if (mcsSet)
		{
			std::vector<cv::Mat> ims(nrCams);
			std::vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
			std::vector<int> vMatches; // Initialization: correspondeces with reference keypoints
			std::vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
			std::vector<cMapPoint*> vMatchedMapPoints; // Tracked MapPoints in current frame
			std::unordered_map<size_t, int> lkeyp_to_cam;
			int state; // Tracking state

			//Copy variable to be used within scoped mutex
			{
				std::unique_lock<std::mutex> lock(mMutex);
				state = mState;
				if (mState == cTracking::SYSTEM_NOT_READY)
					mState = cTracking::NO_IMAGES_YET;

				if (mState == cTracking::NOT_INITIALIZED)
				{
					vCurrentKeys = mvCurrentKeys;
					vIniKeys = mvIniKeys;
					vMatches = mvIniMatches;
					lkeyp_to_cam = keyp_to_cam;
				}
				else if (mState == cTracking::INITIALIZING)
				{
					vCurrentKeys = mvCurrentKeys;
					vIniKeys = mvIniKeys;
					vMatches = mvIniMatches;
					lkeyp_to_cam = keyp_to_cam;
				}
				else if (mState == cTracking::WORKING)
				{
					vCurrentKeys = mvCurrentKeys;
					vMatchedMapPoints = mvpMatchedMapPoints;
					lkeyp_to_cam = keyp_to_cam;
				}
				else if (mState == cTracking::LOST)
				{
					vCurrentKeys = mvCurrentKeys;
					lkeyp_to_cam = keyp_to_cam;
				}
			} // destroy scoped mutex -> release

			for (int c = 0; c < nrCams; ++c)
			{
				ims[c] = mImages[c];
				if (ims[c].channels() < 3)
					cvtColor(ims[c], ims[c], CV_GRAY2BGR);

				//Draw
				if (state == cTracking::INITIALIZING) //INITIALIZING
				{
					for (unsigned int i = 0; i < vMatches.size(); ++i)
					{

						if (vMatches[i] >= 0)
						{
							int camIdx = lkeyp_to_cam.find(vMatches[i])->second;
							cv::line(ims[camIdx], vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt,
								cv::Scalar(0, 255, 0));
						}
					}
				}
				// TRACKING
				else if (state == cTracking::WORKING)
				{
					mnTracked = 0;
					const float r = 5;
					for (unsigned int i = 0; i < vMatchedMapPoints.size(); ++i)
					{
						if (vMatchedMapPoints[i] || mvbOutliers[i])
						{
							int camIdx = lkeyp_to_cam.find(i)->second;

							cv::Point2f pt1, pt2;
							pt1.x = vCurrentKeys[i].pt.x - r;
							pt1.y = vCurrentKeys[i].pt.y - r;
							pt2.x = vCurrentKeys[i].pt.x + r;
							pt2.y = vCurrentKeys[i].pt.y + r;
							if (!mvbOutliers[i])
							{
								int ptIdx = vMatchedMapPoints[i]->mnId;

								cv::rectangle(ims[camIdx], pt1, pt2, cv::Scalar(255, 0, 0));

								cv::circle(ims[camIdx], vCurrentKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
								//cv::putText(ims[camIdx], to_string(ptIdx), pt1 + cv::Point2f(2, 2), 1, 0.75, cv::Scalar(0, 0, 255));
								++mnTracked;
							}
						}
					}
				}
				//if (c==0)
				//	DrawTextInfo(ims[c], state, imWithInfo);
				//cv::imwrite("C:/trajectories/traj8loop2/video/img"+to_string(c)+"_" + to_string(cntWrite) + ".jpg", ims[c]);
			}
			//cv::Mat allImgs = cv::Mat::zeros(cv::Size(mImages[0].cols, 3 * mImages[0].rows), CV_8UC3);
			//ims[0].copyTo(allImgs(cv::Rect(r*, 0, mImages[cn].cols, mImages[cn].rows)));
			//ims[1].copyTo(allImgs(cv::Rect(0, mImages[1].rows, mImages[1].cols, mImages[1].rows)));
			//ims[2].copyTo(allImgs(cv::Rect(0, mImages[2].rows * 2, mImages[2].cols, mImages[2].rows)));
			DrawTextInfo(ims[0], state, imWithInfo);
			imgs.push_back(imWithInfo);
			for (int c = 1; c < nrCams;++c)
				imgs.push_back(ims[c]);
		}
		//cv::Mat imWithInfoR;
		//cv::resize(imWithInfo, imWithInfoR, cv::Size(), 0.7, 0.7, cv::INTER_LINEAR);
		//cv::imwrite("imWithInfoR" + to_string(cntWrite) + ".jpg", imWithInfo);
		//++cntWrite;
	}

	void cMultiFramePublisher::DrawTextInfo(cv::Mat &im,
		int nState, cv::Mat &imText)
	{
		std::stringstream s;
		if (nState == cTracking::NO_IMAGES_YET)
			s << "WAITING FOR IMAGES. (Topic: /camera/image_raw)";
		else if (nState == cTracking::NOT_INITIALIZED)
			s << " NOT INITIALIZED ";
		else if (nState == cTracking::INITIALIZING)
			s << " TRYING TO INITIALIZE ";
		else if (nState == cTracking::WORKING)
		{
			s << " TRACKING ";
			//int nKFs = mpMap->KeyFramesInMap();
			//int nMPs = mpMap->MapPointsInMap();
			// s << " - KFs: " << nKFs << " , MPs: " << nMPs << " , Tracked: " << mnTracked;
		}
		else if (nState == cTracking::LOST)
		{
			s << " TRACK LOST. TRYING TO RELOCALIZE ";
		}
		else if (nState == cTracking::SYSTEM_NOT_READY)
		{
			s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
		}

		int baseline = 0;
		cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

		imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
		im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
		imText.rowRange(im.rows, imText.rows) = cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
		cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
	}

	void cMultiFramePublisher::Update(cTracking *pTracker)
	{
		std::unique_lock<std::mutex> lock(mMutex);

		mImages = pTracker->mCurrentFrame.images;
		mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
		mvpMatchedMapPoints = pTracker->mCurrentFrame.mvpMapPoints;
		mvbOutliers = pTracker->mCurrentFrame.mvbOutlier;
		keyp_to_cam = pTracker->mCurrentFrame.keypoint_to_cam;

		if (pTracker->mLastProcessedState == cTracking::INITIALIZING)
		{
			mvIniKeys = pTracker->mInitialFrame.mvKeys;
			mvIniMatches = pTracker->mvIniMatches;
		}
		mState = static_cast<int>(pTracker->mLastProcessedState);

		mbUpdated = true;
	}
}
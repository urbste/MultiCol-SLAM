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

#include <opencv2/opencv.hpp>


#include "cTracking.h"


#include "cORBmatcher.h"
#include "cMultiFramePublisher.h"
#include "cConverter.h"
#include "cMap.h"
#include "cMultiInitializer.h"
#include "cOptimizer.h"
#include "cSim3Solver.h"

#include "misc.h"
#ifdef _WIN32
#include <conio.h>
#endif
#include <memory>


namespace MultiColSLAM
{
using namespace std;

cTracking::cTracking(
	cSystem* pSys,
	ORBVocabulary* pVoc,
	cMultiFramePublisher *pFramePublisher,
	cMapPublisher *pMapPublisher,
	cMap *pMap,
	cMultiKeyFrameDatabase* pKFDB,
	cMultiCamSys_ camSystem_,
	std::string settingsPath_) :
	mState(NO_IMAGES_YET),
	mpORBVocabulary(pVoc),
	mpSystem(pSys),
	mpFramePublisher(pFramePublisher),
	mpMapPublisher(pMapPublisher),
	mpKeyFrameDB(pKFDB),
	mpMap(pMap),
	mnLastRelocFrameId(0),
	mbPublisherStopped(false),
	mbReseting(false),
	mbForceRelocalisation(false),
	mbMotionModel(false),
	settingsPath(settingsPath_),
	camSystem(camSystem_),
	curBaseline2MKF(0.0),
	finished(false),
	grab(true),
	loopAndMapperSet(false)
{

	pFramePublisher->SetMCS(&camSystem);
	mpMapPublisher->SetMCS(camSystem.Get_All_M_c());
	numberCameras = static_cast<int>(camSystem.Get_All_M_c().size());
	/////////////
	// load slam settings
	/////////////
	cv::FileStorage slamSettings(settingsPath, cv::FileStorage::READ);
	double fps = slamSettings["Camera.fps"];
    if (fps==0)
        fps = 25;

    // Max/Min Frames to insert keyframes and to check relocalisation
	mMinFrames = cvRound(fps / 3);
    mMaxFrames = cvRound(2*fps / 3);

    std::cout << "Camera Parameters: " << endl;
	std::cout << "- fps: " << fps << endl;


	int nRGB = slamSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
		std::cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
		std::cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters
	int featDim = (int)slamSettings["extractor.descSize"];
	int nFeatures = (int)slamSettings["extractor.nFeatures"];
	float fScaleFactor = slamSettings["extractor.scaleFactor"];
	int nLevels = (int)slamSettings["extractor.nLevels"];
	int fastTh = (int)slamSettings["extractor.fastTh"];
	int Score = (int)slamSettings["extractor.nScoreType"];

	assert(Score == 1 || Score == 0);

	this->use_mdBRIEF = false;
	bool learnMasks = false;

	int usemd = (int)slamSettings["extractor.usemdBRIEF"];
	this->use_mdBRIEF = static_cast<bool>(usemd);
	int masksL = (int)slamSettings["extractor.masks"];
	learnMasks = static_cast<bool>(masksL);

	mp_mdBRIEF_extractorOct.resize(numberCameras);
	mp_mdBRIEF_init_extractorOct.resize(numberCameras);

	int useAgast = (int)slamSettings["extractor.useAgast"];
	int fastAgastType = (int)slamSettings["extractor.fastAgastType"];
	int descSize = (int)slamSettings["extractor.descSize"];

	assert(descSize == 16 || descSize == 32 || descSize == 64);

	std::cout << endl << "Extractor Parameters: " << endl;
	std::cout << "- Number of Features: " << nFeatures << endl;
	std::cout << "- Scale Levels: " << nLevels << endl;
	std::cout << "- Scale Factor: " << fScaleFactor << endl;
	std::cout << "- Fast Threshold: " << fastTh << endl;
	std::cout << "- Learn Masks: " << learnMasks << endl;
	std::cout << "- Descriptor Size (byte): " << descSize << endl;
	std::cout << "- Use AGAST: " << useAgast << endl;
	std::cout << "- FAST/AGAST Type: " << fastAgastType << endl;

	if (Score == 0)
		std::cout << "- Score: HARRIS" << endl;
	else
		std::cout << "- Score: FAST" << endl;

	for (int c = 0; c < numberCameras; ++c)
	{
		mp_mdBRIEF_extractorOct[c] = new mdBRIEFextractorOct(nFeatures,
			fScaleFactor, nLevels, 25, 0, Score,
			32, fastTh, (bool)useAgast, fastAgastType,this->use_mdBRIEF, learnMasks, descSize);

		mp_mdBRIEF_init_extractorOct[c] = new mdBRIEFextractorOct(2 * nFeatures,
			fScaleFactor, nLevels, 25, 0, Score,
			32, 5, (bool)useAgast, fastAgastType, this->use_mdBRIEF, learnMasks, descSize);
	}

	
    // ORB extractor for initialization
    // Initialization uses only points from the finest scale level
	int nMotion = slamSettings["UseMotionModel"];
    mbMotionModel = nMotion;

    if(mbMotionModel)
    {
        mVelocity = cv::Matx44d::eye();
		std::cout << endl << "Motion Model: Enabled" << endl << endl;
    }
    else
		std::cout << endl << "Motion Model: Disabled (not recommended, change settings UseMotionModel: 1)" << endl << endl;

	//allPoses = std::vector<cv::Matx61d>(nrImages2Track);
	//allPosesBool = std::vector<bool>(nrImages2Track);
	//nrTrackedPts = std::vector<int>(nrImages2Track);
	//inlierRatio = std::vector<double>(nrImages2Track);
}

void cTracking::SetLocalMapper(cLocalMapping *pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void cTracking::SetLoopClosing(cLoopClosing *pLoopClosing)
{
    mpLoopClosing = pLoopClosing;
}

void cTracking::SetKeyFrameDatabase(cMultiKeyFrameDatabase *pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

void cTracking::SetViewer(cViewer *pViewer)
{
	mpViewer = pViewer;
}

int cTracking::GetNrCams()
{
	return this->numberCameras;
}

cv::Matx44d cTracking::GrabImageSet(const std::vector<cv::Mat>& imgSet,
	const double& timestamp)
{
	std::chrono::system_clock Time;

	std::vector<cv::Mat> convertedImages(imgSet.size());
	convertedImages = imgSet;

	if (mState == WORKING || mState == LOST)
		mCurrentFrame = cMultiFrame(convertedImages,
		timestamp, mp_mdBRIEF_extractorOct, mpORBVocabulary, 
		camSystem, imgCounter - 1);
	else
		mCurrentFrame = cMultiFrame(convertedImages,
		timestamp, mp_mdBRIEF_init_extractorOct, mpORBVocabulary, 
		camSystem, imgCounter - 1);

	if (!loopAndMapperSet)
	{
		mpLocalMapper->SetMatcherProperties(mCurrentFrame.DescDims(),
			mCurrentFrame.HavingMasks());
		mpLoopClosing->SetMatcherProperties(mCurrentFrame.DescDims(),
			mCurrentFrame.HavingMasks());
		loopAndMapperSet = true;
	}

	Track();

	return mCurrentFrame.GetPose();
}

bool cTracking::Track()
{
    // Depending on the state of the Tracker we perform different tasks
    if (mState == NO_IMAGES_YET)
        mState = NOT_INITIALIZED;

    mLastProcessedState = mState;

    if (mState == NOT_INITIALIZED)
    {
        FirstInitialization();
    }
    else if (mState == INITIALIZING)
    {
        Initialize();
    }
    else
    {
        // System is initialized. Track Frame.
		bool bOK = false;
		//if (_kbhit())
		//{
		//	int ch;
		//	ch = _getch();
		//	if (ch == 114)
		//	{
		//		ForceRelocalisation();
		//	}
		//}
        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        if (mState == WORKING && !RelocalisationRequested())
        {
			if (!mbMotionModel || 
				 mpMap->KeyFramesInMap() < 2  ||
				 mCurrentFrame.mnId < mnLastRelocFrameId + 2)
			{
				bOK = TrackPreviousFrame();
			}
            else
            {
				bOK = TrackWithMotionModel();
				if (!bOK)
					bOK = TrackPreviousFrame();			
            }
        }
        else
        {
            bOK = Relocalisation();
        }
		// If we have an initial estimation of the camera pose and matching. Track the local map.
		if (bOK)
		{
			bOK = TrackLocalMap();
		}

        // If tracking were good, check if we insert a keyframe
        if (bOK)
        {
            mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.GetPose());
			// count tracked points in each cam
			//CountNumberTrackedPointsPerCam();
			if (NeedNewKeyFrame())
				CreateNewKeyFrame();
            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(size_t i=0; i < mCurrentFrame.mvbOutlier.size();i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i] = NULL;
            }
        }

        if(bOK)
            mState = WORKING;
        else
            mState = LOST;

        // Reset if the camera get lost soon after initialization
        if (mState == LOST)
        {
            if (mpMap->KeyFramesInMap() <= 3)
            {
                Reset();
                return true;
            }
        }

        // Update motion model
        if (mbMotionModel)
        {
			if (bOK)
            {
				cv::Matx44d LastTwc = cConverter::invMat(mLastFrame.GetPose());
				//cv::Matx44d current = mCurrentFrame.GetPose().inv();
				//mVelocity = current * LastTwc;
				mVelocity = LastTwc*mCurrentFrame.GetPose();
            }
            else
                mVelocity = cv::Matx44d::eye();
        }

        mLastFrame = cMultiFrame(mCurrentFrame);
     }       

    // Update drawer
    mpFramePublisher->Update(this);
	return true;
}

void cTracking::CountNumberTrackedPointsPerCam()
{
	int nrCams = camSystem.GetNrCams();
	nbTrackedPtsInCam = std::vector<int>(nrCams);
	nbTrackedRatios = std::vector<double>();
	for (int c = 0; c < nrCams; ++c)
		nbTrackedPtsInCam[c] = 0;

	for (size_t i = 0; i < mCurrentFrame.mvpMapPoints.size(); ++i)
		if (mCurrentFrame.mvpMapPoints[i])
			++nbTrackedPtsInCam[mCurrentFrame.keypoint_to_cam.find(i)->second];

	// calc ratios
	for (int c1 = 0; c1 < nrCams; ++c1)
	{
		for (int c2 = c1 + 1; c2 < nrCams; ++c2)
		{
			if ((double)nbTrackedPtsInCam[c2] > 0)
				nbTrackedRatios.push_back((double)
					nbTrackedPtsInCam[c1] / (double)nbTrackedPtsInCam[c2]);
			if ((double)nbTrackedPtsInCam[c1] > 0)
				nbTrackedRatios.push_back((double)
					nbTrackedPtsInCam[c2] / (double)nbTrackedPtsInCam[c1]);
		}
	}
}

void cTracking::FirstInitialization()
{
    //We ensure a minimum ORB features to continue, otherwise discard frame
    if (mCurrentFrame.mvKeys.size() > 100)
    {
		fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
		mInitialFrame = cMultiFrame(mCurrentFrame);
		mLastFrame = cMultiFrame(mCurrentFrame);
        mvbPrevMatched.resize(mCurrentFrame.mvKeys.size());
		for (size_t i = 0; i < mCurrentFrame.mvKeys.size(); ++i)
            mvbPrevMatched[i] = 
				cv::Vec2d(mCurrentFrame.mvKeys[i].pt.x, mCurrentFrame.mvKeys[i].pt.y);

		mpInitializer = new cMultiInitializer(mCurrentFrame, 1.0, 200);
		mState = INITIALIZING;   
    }
}

void cTracking::Initialize()
{
    // Check if current frame has enough keypoints, otherwise reset initialization process

	if (mCurrentFrame.mvKeys.size() <= 100)
    {
        fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
        mState = NOT_INITIALIZED;
        return;
    }    

    // Find correspondences
	cORBmatcher matcher(0.9, checkOrientation, mCurrentFrame.DescDims(), mCurrentFrame.HavingMasks());
    int nmatches = matcher.SearchForInitialization(mInitialFrame,
		mCurrentFrame,
		mvbPrevMatched,
		mvIniMatches,50);

    // Check if there are enough correspondences
    if (nmatches < 100)
    {
        mState = NOT_INITIALIZED;
        return;
    }  

    cv::Matx33d Rcw; // Current Camera Rotation
    cv::Vec3d tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
	int leadingCam = 0;
	if (mpInitializer->
		Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated, leadingCam))
    {
		for (size_t i = 0, iend = mvIniMatches.size(); i < iend; ++i)
        {
            if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
            {
                mvIniMatches[i] = -1;
				--nmatches;
            }           
        }

		CreateInitialMap(Rcw, tcw, leadingCam);
    }

}

void cTracking::CreateInitialMapStereo()
{
	mInitialFrame.SetPose(cv::Matx44d::eye());
	// Create KeyFrames
	cMultiKeyFrame* pKFini = new cMultiKeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
	pKFini->imageId = mInitialFrame.GetImgCnt();

	pKFini->ComputeBoW();
	mpMap->AddKeyFrame(pKFini);

	// Create MapPoints and asscoiate to keyframes
	for (size_t i = 0; i < matchedPairsStereo.size(); ++i)
	{
		if (mvIniMatches[i] < 0)
			continue;

		//Create MapPoint.
		cv::Vec3d worldPos(mvIniP3D[i]);

		cMapPoint* pMP = new cMapPoint(worldPos, pKFcur, mpMap);
		// assign mappoint to keyframes
		pKFini->AddMapPoint(pMP, i);
		// add observation to mappoints
		pMP->AddObservation(pKFini, matchedPairsStereo[i].first);
		pMP->AddObservation(pKFini, matchedPairsStereo[i].second);
		// compute some statistics about the mappoint
		pMP->ComputeDistinctiveDescriptors(pKFini->HavingMasks());
		cv::Mat desc = pMP->GetDescriptor();
		pMP->UpdateCurrentDescriptor(desc);
		pMP->UpdateNormalAndDepth();

		//Fill Current Frame structure
		mCurrentFrame.mvpMapPoints[matchedPairsStereo[i].first] = pMP;
		mCurrentFrame.mvpMapPoints[matchedPairsStereo[i].second] = pMP;
		//Add to Map
		mpMap->AddMapPoint(pMP);

	}
	pKFini->UpdateConnections();
	mpLocalMapper->InsertMultiKeyFrame(pKFini);

	cv::Matx44d iniPose = pKFini->GetPose();
	mInitialFrame.SetPose(iniPose);
	cv::Matx44d curPose = pKFini->GetPose();
	mCurrentFrame.SetPose(curPose);

	mLastFrame = cMultiFrame(mCurrentFrame);
	mnLastKeyFrameId = mCurrentFrame.mnId;
	mpLastKeyFrame = pKFini;

	// add local keyframes for the tracker
	mvpLocalKeyFrames.push_back(pKFini);
	mvpLocalMapPoints = mpMap->GetAllMapPoints();
	mpReferenceKF = pKFini;

	mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

	mpMapPublisher->SetCurrentCameraPose(pKFini->GetPose());

	mState = WORKING;
}

void cTracking::CreateInitialMap(cv::Matx33d &Rcw, cv::Vec3d &tcw, int leadingCam)
{
	// Set Frame Poses
	// we have to calculate the multi cam sys poses from the single camera poses
	cv::Matx44d invMc = mInitialFrame.camSystem.Get_M_t() *
		cConverter::invMat(mInitialFrame.camSystem.Get_M_c(leadingCam));
	cv::Matx44d Mc = mInitialFrame.camSystem.Get_M_c(leadingCam);
	//cv::Matx44d invMc = Mc.inv();
	mInitialFrame.SetPose(invMc);
	cv::Matx44d invCurr = cConverter::Rt2Hom(Rcw, tcw)*invMc;
	mCurrentFrame.SetPose(invCurr); // inverse!

	// Create KeyFrames
	cMultiKeyFrame* pKFini = new cMultiKeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
	pKFini->imageId = mInitialFrame.GetImgCnt();
	cMultiKeyFrame* pKFcur = new cMultiKeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
	pKFcur->imageId = mCurrentFrame.GetImgCnt();

	pKFini->ComputeBoW();
	pKFcur->ComputeBoW();

	// Insert KFs in the map
	mpMap->AddKeyFrame(pKFini);
	mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
	for (size_t i = 0; i < mvIniMatches.size(); ++i)
    {
        if (mvIniMatches[i] < 0)
            continue;

        //Create MapPoint.
        cv::Vec3d worldPos(mvIniP3D[i]);

        cMapPoint* pMP = new cMapPoint(worldPos,pKFcur,mpMap);
		// assign mappoint to keyframes
        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);
		// add observation to mappoints
        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);
		// compute some statistics about the mappoint
		pMP->ComputeDistinctiveDescriptors(pKFcur->HavingMasks());
		cv::Mat desc = pMP->GetDescriptor();
		pMP->UpdateCurrentDescriptor(desc);

        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;

        //Add to Map
        mpMap->AddMapPoint(pMP);

    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    if(pKFcur->TrackedMapPoints() < 15)
    {
        Reset();
        return;
    }

	vector<cMapPoint*> vpAllMapPoints1 = pKFini->GetMapPointMatches();
	vector<cMapPoint*> vpAllMapPoints2 = pKFcur->GetMapPointMatches();

	cOptimizer::GlobalBundleAdjustment(mpMap, true);

	cORBmatcher tempMatcher(0.8, checkOrientation, 
		pKFcur->DescDims(), pKFcur->HavingMasks());
	vector<double> scales;
	vector<cv::Vec3d> ptsBefore;
	vector<cv::Vec3d> ptsAfter;
	for (int c = 0; c < pKFcur->camSystem.GetNrCams(); ++c)
	{
		// skip the same cam
		// we just want to search in all other cams
		if (c == leadingCam)
			continue;

		cv::Matx44d relOri = Mc * cConverter::invMat(mInitialFrame.camSystem.Get_M_c(c));
		cv::Matx33d Rrel = cConverter::Hom2R(relOri);
		cv::Vec3d trel = cConverter::Hom2T(relOri);
		// essential matrix for epipolar test
		cv::Matx33d E12 = ComputeE(relOri);

		// loop through map points
		for (size_t iMP = 0; iMP < vpAllMapPoints1.size(); ++iMP)
		{
			cMapPoint* pMP = vpAllMapPoints1[iMP];

			cv::KeyPoint ptMatch1 = pKFini->GetKeyPoint(iMP);

			if (!vpAllMapPoints1[iMP])
				continue;

			cv::Vec2d uv;
			cv::Vec3d wp = pMP->GetWorldPos();
			cv::Vec4d wp4 = cConverter::toVec4d(wp);
			pKFini->camSystem.WorldToCamHom_fast(c, wp4, uv);
			// test if the point even projects into the mirror mask
			if (!pKFini->camSystem.GetCamModelObj(c).isPointInMirrorMask(uv(0), uv(1), 0))
				continue;

			// if yes, then get all the features in that area
			vector<size_t> vIndices =
				pKFini->GetFeaturesInArea(c, uv(0), uv(1), 50);
			if (vIndices.empty())
				continue;

			// get descriptor of point in the leading cam
			int idxDescLast = pKFini->cont_idx_to_local_cam_idx.find(iMP)->second;
			//cv::Mat descMP1 = pKFini->GetDescriptor(leadingCam, idxDescLast);
			const uint64_t* descMP = pKFini->GetDescriptorRowPtr(leadingCam, idxDescLast);
			const uint64_t* descMP_mask = 0;
			if (pKFini->HavingMasks())
				descMP_mask = pKFini->GetDescriptorMaskRowPtr(leadingCam, idxDescLast);

			// match to descriptors in area
			int bestDist = INT_MAX;
			int bestIdx2 = -1;
			// match the descriptor of the current image point to all points in the area
			for (vector<size_t>::iterator vit = vIndices.begin(), vend = vIndices.end();
				vit != vend; ++vit)
			{
				size_t i2 = *vit;
				int idxDescCurr = pKFini->cont_idx_to_local_cam_idx.find(i2)->second;
				//cv::Mat d = pKFini->GetDescriptor(c, idxDescCurr);
				const uint64_t* d = pKFini->GetDescriptorRowPtr(c, idxDescCurr);

				int dist = 0;
				if (pKFini->HavingMasks())
				{
					const uint64_t* d_mask = pKFini->GetDescriptorRowPtr(c, idxDescCurr);
					dist = DescriptorDistance64Masked(descMP, d, descMP_mask, d_mask, pKFini->DescDims());
				}
				else dist = DescriptorDistance64(descMP, d, pKFini->DescDims());

				if (dist < bestDist)
				{
					bestDist = dist;
					bestIdx2 = i2;
				}
			}
			cv::KeyPoint ptMatch2 = pKFini->GetKeyPoint(bestIdx2);

			cv::Vec3d Xl1 = pKFini->GetKeyPointRay(iMP);
			cv::Vec3d Xl2 = pKFini->GetKeyPointRay(bestIdx2);
			if (bestDist <= tempMatcher.TH_HIGH_ && CheckDistEpipolarLine(Xl1, Xl2, E12, 1e-2))
			{
				cv::Vec3d wp = pMP->GetWorldPos();

				pMP->AddObservation(pKFini, bestIdx2);
				// because we can have multiple observations per mappoint,
				pKFini->AddMapPoint(pMP, bestIdx2);
				mInitialFrame.mvpMapPoints[bestIdx2] = pMP;
			}
		}
	}

	cOptimizer::GlobalBundleAdjustment(mpMap, false);

	vpAllMapPoints1 = pKFini->GetMapPointMatches();

	// test reprojection
	for (int c = 0; c < pKFcur->camSystem.GetNrCams(); ++c)
	{
		cv::Matx44d relOri = Mc * cConverter::invMat(mCurrentFrame.camSystem.Get_M_c(c));
		cv::Matx33d Rrel = cConverter::Hom2R(relOri);
		cv::Vec3d trel = cConverter::Hom2T(relOri);
		// essential matrix for epipolar test
		cv::Matx33d E12 = ComputeE(relOri);

		// loop through map points
		for (size_t iMP = 0; iMP < vpAllMapPoints2.size(); ++iMP)
		{
			cMapPoint* pMP = vpAllMapPoints2[iMP];

			if (!pMP)
				continue;

			cv::KeyPoint ptMatch1 = pKFcur->GetKeyPoint(iMP);
			cv::Vec3d pos = pMP->GetWorldPos();
			if (vpAllMapPoints2[iMP])
			{
				if (!(c == leadingCam))
				{
					cv::Vec2d uv;
					cv::Vec3d wp = pMP->GetWorldPos();
					cv::Vec4d wp4 = cv::Vec4d(wp(0), wp(1), wp(2), 1.0);
					pKFcur->camSystem.WorldToCamHom_fast(c, wp4, uv);
					// test if the point even projects into the mirror mask
					if (!pKFcur->camSystem.GetCamModelObj(c).isPointInMirrorMask(uv(0), uv(1), 0))
						continue;

					// if yes, then get all the features in that area
					vector<size_t> vIndices =
						pKFcur->GetFeaturesInArea(c, uv(0), uv(1), 50);
					if (vIndices.empty())
						continue;
					// get descriptor of point in the leading cam
					int idxDescLast = pKFcur->cont_idx_to_local_cam_idx.find(iMP)->second;
					//cv::Mat descMP1 = pKFcur->GetDescriptor(leadingCam, idxDescLast);
					const uint64_t* descMP = pKFcur->GetDescriptorRowPtr(leadingCam, idxDescLast);
					const uint64_t* descMP_mask = 0;
					if (pKFcur->HavingMasks())
						descMP_mask = pKFcur->GetDescriptorMaskRowPtr(leadingCam, idxDescLast);

					// match to descriptors in area
					int bestDist = INT_MAX;
					int bestIdx2 = -1;
					// match the descriptor of the current image point to all points in the area
					for (vector<size_t>::iterator vit = vIndices.begin(), vend = vIndices.end();
						vit != vend; ++vit)
					{
						size_t i2 = *vit;
						int idxDescCurr = pKFcur->cont_idx_to_local_cam_idx.find(i2)->second;
						//cv::Mat d = pKFcur->GetDescriptor(c, idxDescCurr);
						const uint64_t* d = pKFcur->GetDescriptorRowPtr(c, idxDescCurr);
						int dist = 0;
						if (pKFcur->HavingMasks())
						{
							const uint64_t* d_mask = pKFcur->GetDescriptorMaskRowPtr(c, idxDescCurr);
							dist = DescriptorDistance64Masked(descMP, d, descMP_mask, d_mask, pKFcur->DescDims());
						}
						else
							dist = DescriptorDistance64(descMP, d, pKFcur->DescDims());

						if (dist < bestDist)
						{
							bestDist = dist;
							bestIdx2 = i2;
						}
					}
					cv::KeyPoint ptMatch2 = pKFcur->GetKeyPoint(bestIdx2);

					cv::Vec3d Xl1 = pKFcur->GetKeyPointRay(iMP);
					cv::Vec3d Xl2 = pKFcur->GetKeyPointRay(bestIdx2);
					bool epiDist = CheckDistEpipolarLine(Xl1, Xl2, E12, 1e-2);

					if (bestDist <= tempMatcher.TH_HIGH_ && epiDist)
					{
						// add observation
						pMP->AddObservation(pKFcur, bestIdx2);
						pKFcur->AddMapPoint(pMP, bestIdx2);
						mCurrentFrame.mvpMapPoints[bestIdx2] = pMP;
						pMP->ComputeDistinctiveDescriptors(pKFcur->HavingMasks());
					}
				}
			}

		}
	}

	cOptimizer::GlobalBundleAdjustment(mpMap, false, 5);

    mpLocalMapper->InsertMultiKeyFrame(pKFini);
	mpLocalMapper->InsertMultiKeyFrame(pKFcur);

	cv::Matx44d iniPose = pKFini->GetPose();
	mInitialFrame.SetPose(iniPose);
	cv::Matx44d curPose = pKFini->GetPose();
	mCurrentFrame.SetPose(curPose);
	mCurrentFrame.mvpMapPoints = pKFcur->GetMapPointMatches();

    mLastFrame = cMultiFrame(mCurrentFrame);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

	// add local keyframes for the tracker
    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapPublisher->SetCurrentCameraPose(pKFcur->GetPose());

    mState = WORKING;
}

bool cTracking::TrackPreviousFrame()
{
	cORBmatcher matcher(0.8, checkOrientation, 
		mCurrentFrame.DescDims(), mCurrentFrame.HavingMasks());
    vector<cMapPoint*> vpMapPointMatches;

    // Search first points at coarse scale levels to get a rough initial estimate
    int minOctave = 0;
    int maxOctave = mCurrentFrame.mvScaleFactors.size() - 1;
    if (mpMap->KeyFramesInMap() > 5)
		minOctave = maxOctave / 2 + 1;
	cv::Matx44d pose = mLastFrame.GetPose();
	mCurrentFrame.SetPose(pose);
    int nmatches = matcher.WindowSearch(mLastFrame, mCurrentFrame,
		60, vpMapPointMatches, minOctave);

    // If not enough matches, search again without scale constraint
    if (nmatches < 10)
    {
        nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,50,vpMapPointMatches,0);
        if (nmatches < 10)
        {
            vpMapPointMatches = vector<cMapPoint*>
				(mCurrentFrame.mvpMapPoints.size(), static_cast<cMapPoint*>(NULL));
            nmatches = 0;
        }
    }
	mCurrentFrame.mvpMapPoints = vpMapPointMatches;
	double inliers = 0.0;
	cOptimizer::PoseOptimization(&mCurrentFrame, inliers);

	// Discard outliers
	for (size_t i = 0; i < mCurrentFrame.mvbOutlier.size(); ++i)
	{
		if (mCurrentFrame.mvbOutlier[i])
		{
			mCurrentFrame.mvpMapPoints[i] = NULL;
			mCurrentFrame.mvbOutlier[i] = false;
			--nmatches;
		}
	}

	nmatches +=
		matcher.SearchByProjection(mLastFrame, mCurrentFrame, 40, vpMapPointMatches);

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;

    if (nmatches < 10)
        return false;

   // Optimize pose again with all correspondences
	cOptimizer::PoseOptimization(&mCurrentFrame, inliers);

    // Discard outliers
	for (size_t i = 0; i < mCurrentFrame.mvbOutlier.size(); ++i)
	{
		if (mCurrentFrame.mvbOutlier[i])
		{
			mCurrentFrame.mvpMapPoints[i] = NULL;
			mCurrentFrame.mvbOutlier[i] = false;
			--nmatches;
		}
	}
    return nmatches >= 6;
}

bool cTracking::TrackWithMotionModel()
{
	std::chrono::steady_clock::time_point begin;
	std::chrono::steady_clock::time_point end;
	cORBmatcher matcher(0.8, checkOrientation, 
		mCurrentFrame.DescDims(), mCurrentFrame.HavingMasks());
    vector<cMapPoint*> vpMapPointMatches;

    // Compute current pose by motion model	
	//mCurrentFrame.SetPose(mVelocity*mLastFrame.GetPose());
	cv::Matx44d pose = mLastFrame.GetPose()*mVelocity;
	mCurrentFrame.SetPose(pose);
	//mCurrentFrame.SetPose(mLastFrame.GetPose()*mVelocity);

    fill(mCurrentFrame.mvpMapPoints.begin(),
		mCurrentFrame.mvpMapPoints.end(),static_cast<cMapPoint*>(NULL));

	begin = std::chrono::steady_clock::now();
    // Project points seen in previous frame
	int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 50);
	end = std::chrono::steady_clock::now();

    if (nmatches < 10)
       return false;

	double inliers= 0.0;
    cOptimizer::PoseOptimization(&mCurrentFrame, inliers);
    // Discard outliers
	for (size_t i = 0; i < mCurrentFrame.mvpMapPoints.size(); ++i)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (mCurrentFrame.mvbOutlier[i])
            {

                mCurrentFrame.mvpMapPoints[i] = NULL;
                mCurrentFrame.mvbOutlier[i] = false;
				--nmatches;
            }
        }
    }
    return nmatches >= 6;
}

bool cTracking::TrackLocalMap()
{
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    // Update Local Map
    UpdateReference();

    // Search Local MapPoints
	int nrPoints = SearchReferencePointsInFrustum();

    // Optimize Pose
	double inliers = 0.0;
	mnMatchesInliers = cOptimizer::PoseOptimization(&mCurrentFrame, inliers);

    // Update MapPoints Statistics
	for (size_t i = 0; i < mCurrentFrame.mvpMapPoints.size(); ++i)
	{
		if (mCurrentFrame.mvpMapPoints[i])
		{
			if (mCurrentFrame.mvbOutlier[i])
			{
				--nrPoints;
			}
			else
			{
				mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
				int idxC = mCurrentFrame.keypoint_to_cam.find(i)->second;
				int descIdx = mCurrentFrame.cont_idx_to_local_cam_idx.find(i)->second;
				cv::Mat desc = mCurrentFrame.mDescriptors[idxC].row(descIdx);
				mCurrentFrame.mvpMapPoints[i]->UpdateCurrentDescriptor(desc);
			}				
		}
	}
	curBaseline2MKF = cv::norm(cConverter::Hom2T(mCurrentFrame.GetPose()) -
		cConverter::Hom2T(mpReferenceKF->GetPose()));

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
	if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 15)
        return false;


	if (mnMatchesInliers < 15)
        return false;
	else
	{
		allPoses.push_back(hom2cayley(mCurrentFrame.GetPose()));
		allPosesBool.push_back(true);
		nrTrackedPts.push_back(mnMatchesInliers);
		inlierRatio.push_back(inliers);
		return true;
	}
}

bool cTracking::NeedNewKeyFrame()
{
	// If Local Mapping is freezed by a Loop Closure do not insert keyframes
	if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
		return false;

	// Not insert keyframes if not enough frames from last relocalisation have passed
	if (mCurrentFrame.mnId<mnLastRelocFrameId + mMaxFrames && mpMap->KeyFramesInMap()>mMaxFrames)
		return false;

	// Reference KeyFrame MapPoints
	int nRefMatches = mpReferenceKF->TrackedMapPoints();

	// Local Mapping accept keyframes?
	bool bLocalMappingIdle = mpLocalMapper->AcceptMultiKeyFrames();

	// Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
	const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
	// Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
	const bool c1b = mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle;
	// Condition 2: Less than 90% of points than reference keyframe and enough inliers
	const bool c2 = mnMatchesInliers < (nRefMatches * 0.9) && mnMatchesInliers > 25;
	// Condition 3: if there is a camera which tracks a lot less keypoints than another
	//const bool c3 = true;
	//const bool c3 = (*std::min_element(this->nbTrackedPtsInCam.begin(), this->nbTrackedPtsInCam.end()) < 15 ||
	//	*std::min_element(this->nbTrackedRatios.begin(), this->nbTrackedRatios.end()) < 0.15) && 
	//	mnMatchesInliers > 15 && bLocalMappingIdle;
	//double ratio = *std::min_element(this->nbTrackedRatios.begin(), this->nbTrackedRatios.end());
	//cout << "ratio: " << ratio << endl;
	// if we track from an initialized model pose we wait until the baseline is big enough

	if (((c1a || c1b) && c2) && (curBaseline2MKF > 0.2))
	//if ((c1a || c1b) && c2)
	{
		// If the mapping accepts keyframes insert, otherwise send a signal to interrupt BA, but not insert yet
		if (bLocalMappingIdle)
		{
			return true;
		}
		else
		{
			mpLocalMapper->InterruptBA();
			return false;
		}
	}
	else
		return false;

}

void cTracking::CreateNewKeyFrame()
{
	const int nrCams = mCurrentFrame.camSystem.GetNrCams();

	cMultiKeyFrame* pKF = new cMultiKeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
	//pKF->SetRenderedImages(worldCoords, normalImages, depthFullImages);
	pKF->imageId = mCurrentFrame.GetImgCnt();
	mpLocalMapper->InsertMultiKeyFrame(pKF);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

int cTracking::SearchReferencePointsInFrustum()
{
    // Do not search map points already matched
	int nrMatches = 0;
	for (int i = 0; i < mCurrentFrame.mvpMapPoints.size(); ++i)
    {
		cMapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP)
        {
            if(pMP->isBad())
            {
				mCurrentFrame.mvpMapPoints[i] = NULL;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;

				int cam = mCurrentFrame.keypoint_to_cam.find(i)->second;
				pMP->mbTrackInView[cam] = false;
				++nrMatches;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<cMapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end();
		vit != vend; ++vit)
    {
        cMapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;        
        // Project (this fills MapPoint variables for matching)
		// project it in each camera
		for (int c = 0; c < mCurrentFrame.camSystem.GetNrCams(); ++c)
		{
			if (mCurrentFrame.isInFrustum(c, pMP, 0.3))
			{
				pMP->IncreaseVisible();
				++nToMatch;
			}
		}
	}    

    if (nToMatch > 0)
    {
		cORBmatcher matcher(0.8, checkOrientation, mCurrentFrame.DescDims(), mCurrentFrame.HavingMasks());
        int th = 3;
        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame.mnId < mnLastRelocFrameId+2)
            th = 3;
        nrMatches += matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
    }

	return nrMatches;
}

void cTracking::UpdateReference()
{    
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateReferenceKeyFrames();
    UpdateReferencePoints();
}

void cTracking::UpdateReferencePoints()
{
    mvpLocalMapPoints.clear();

	for (vector<cMultiKeyFrame*>::iterator itKF = mvpLocalKeyFrames.begin(),
		itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; ++itKF)
    {
		cMultiKeyFrame* pKF = *itKF;
        vector<cMapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<cMapPoint*>::iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); 
			itMP != itEndMP; ++itMP)
        {
            cMapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            }
        }
    }
}

void cTracking::UpdateReferenceKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
	// each map point that was found in the the current frame
	map<cMultiKeyFrame*, int> keyframeCounter;
	for (size_t i = 0, iend = mCurrentFrame.mvpMapPoints.size(); i<iend; ++i)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            cMapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if (!pMP->isBad())
            {
				map<cMultiKeyFrame*, std::vector<size_t>> observations = pMP->GetObservations();
				for (map<cMultiKeyFrame*, std::vector<size_t>>::iterator it = observations.begin(),
					itend = observations.end(); it != itend; it++)
				{
					//keyframeCounter[it->first] += static_cast<int>(it->second.size());
					keyframeCounter[it->first]++;
				}

            }
            else
            {
                mCurrentFrame.mvpMapPoints[i] = NULL;
            }
        }
    }

    int max = 0;
	cMultiKeyFrame* pKFmax = NULL;

    mvpLocalKeyFrames.clear();
	mvpLocalKeyFramesCovWeights.clear();
	mvpLocalKeyFramesDistance2Frame.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. 
	// Also check which keyframe shares most points
	for (map<cMultiKeyFrame*, int>::iterator it = keyframeCounter.begin(), 
		itEnd = keyframeCounter.end();it != itEnd; ++it)
    {
		cMultiKeyFrame* pKF = it->first;
		pKF->SetReference(false); // set reference to false, only to display reference
		// if the keyframe counter is above 2
		// this effectively controls how fast the tracker "forgets" 
		// map points that are probably not visible any more
		// the higher the threshold the smaller the local map
		if (it->second > 4)
		{
			if (pKF->isBad())
				continue;

			if (it->second > max)
			{
				max = it->second;
				pKFmax = pKF;
			}
			double 	curBaseline2MKF = cv::norm(cConverter::Hom2T(mCurrentFrame.GetPose()) -
				cConverter::Hom2T(pKF->GetPose()));
			//cout << "kf id: "<<pKF->mnId<<" kf weight: " << it->second << " dist: " << curBaseline2MKF << endl;

			mvpLocalKeyFramesCovWeights.push_back(it->second);

			mvpLocalKeyFrames.push_back(it->first);
			mvpLocalKeyFramesDistance2Frame.push_back(curBaseline2MKF);
		
			pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
		}
    }

	mpReferenceKF->SetReference(true);
    mpReferenceKF = pKFmax;
}

bool cTracking::Relocalisation()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalisation is performed when tracking is lost and forced at some stages during loop closing
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<cMultiKeyFrame*> vpCandidateKFs;
	if (!RelocalisationRequested())
        vpCandidateKFs = mpKeyFrameDB->DetectRelocalisationCandidates(&mCurrentFrame);
    else // Forced Relocalisation: Relocate against local window around last keyframe
    {
		std::unique_lock<std::mutex> lock(mMutexForceRelocalisation);
        mbForceRelocalisation = false;
        vpCandidateKFs.reserve(10);
        vpCandidateKFs = mpLastKeyFrame->GetBestCovisibilityKeyFrames(5);
        vpCandidateKFs.push_back(mpLastKeyFrame);
		
    }

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();
    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
	cORBmatcher matcher(0.9, checkOrientation, mCurrentFrame.DescDims(), mCurrentFrame.HavingMasks());

	vector<opengv::bearingVectors_t> matchedBearingVecs(nKFs);
	vector<opengv::points_t> points3D(nKFs);
	opengv::translations_t camOffsets = camSystem.Get_All_t_c_ogv();
	opengv::rotations_t camRotations = camSystem.Get_All_R_c_ogv();
	std::vector<std::vector<int> > camCorrespondences(nKFs);

	vector<int> indices_ransac_to_mp(nKFs);
	vector<vector<int>> mvKeyPointIndices(nKFs, vector<int>());

    vector<vector<cMapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

	for (size_t i = 0; i < vpCandidateKFs.size(); ++i)
    {
        cMultiKeyFrame* pKF = vpCandidateKFs[i];
        if (pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if (nmatches < 15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
				// bearing vectors
				opengv::bearingVectors_t mvP2D; 
				// 3D Points
				opengv::points_t mvP3Dw;
				// get points
				int idx = 0;
				for (size_t j = 0, iend = vvpMapPointMatches[i].size(); j < iend; ++j)
				{
					cMapPoint* pMP = vvpMapPointMatches[i][j];

					if (pMP)
					{
						if (!pMP->isBad())
						{
							const cv::Vec3d &kpRay = mCurrentFrame.mvKeysRays[j];
							mvP2D.push_back(opengv::bearingVector_t(kpRay(0), kpRay(1), kpRay(2)));

							cv::Vec3d Pos = pMP->GetWorldPos();
							mvP3Dw.push_back(opengv::point_t(Pos(0), Pos(1), Pos(2)));
							mvKeyPointIndices[i].push_back(j);
							int cam = mCurrentFrame.keypoint_to_cam.find(j)->second;
							camCorrespondences[i].push_back(cam);
							++idx;
						}
					}
				}
				matchedBearingVecs[i] = mvP2D;
				points3D[i] = mvP3Dw;
				// setup an adapter for each keyframe we are trying

				++nCandidates;
            }
        }        
    }

    bool bMatch = false;
	cORBmatcher matcher2(0.9, checkOrientation, mCurrentFrame.DescDims(), mCurrentFrame.HavingMasks());

	for (size_t i = 0; i < vpCandidateKFs.size(); i++)
	{
		if (vbDiscarded[i])
			continue;

		vector<bool> vbInliers;
		vector<int> inliers;

		int nInliers;
		bool bNoMore;

		opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(
			matchedBearingVecs[i],
			camCorrespondences[i],
			points3D[i],
			camOffsets,
			camRotations);
#undef max
#undef min
		opengv::sac::Ransac < opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem >
			ransac;
		std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
			absposeproblem_ptr(
			new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
			adapter,
			opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P));
		ransac.sac_model_ = absposeproblem_ptr;
		ransac.threshold_ = 0.0001;
		ransac.max_iterations_ = 150;

		ransac.computeModel();
		inliers = ransac.inliers_;
		opengv::transformation_t trafo = ransac.model_coefficients_;
		

		// If Ransac reaches max. iterations discard keyframe
		if (ransac.iterations_ >= ransac.max_iterations_)
		{
			vbDiscarded[i] = true;
			--nCandidates;
		}
		else
		{
			trafo = opengv::absolute_pose::gpnp(adapter, ransac.inliers_);

			cv::Matx44d trafoOut = cConverter::ogv2ocv(trafo);

			mCurrentFrame.SetPose(trafoOut);
			set<cMapPoint*> sFound;

			for (int ii = 0; ii < mCurrentFrame.mvpMapPoints.size(); ++ii)
				mCurrentFrame.mvpMapPoints[ii] = NULL;

			for (size_t j = 0; j < inliers.size(); ++j)
			{
				mCurrentFrame.mvpMapPoints[mvKeyPointIndices[i][inliers[j]]] =
					vvpMapPointMatches[i][mvKeyPointIndices[i][inliers[j]]];
				sFound.insert(vvpMapPointMatches[i][mvKeyPointIndices[i][inliers[j]]]);
			}
			double inliers = 0.0;
			int nGood = cOptimizer::PoseOptimization(&mCurrentFrame, inliers);

			if (nGood < 10)
				continue;

			for (size_t io = 0, ioend = mCurrentFrame.mvbOutlier.size(); io < ioend; ++io)
				if (mCurrentFrame.mvbOutlier[io])
					mCurrentFrame.mvpMapPoints[io] = NULL;

			// If the pose is supported by enough inliers stop ransacs and continue
			if (nGood >= 10)
			{
				bMatch = true;
				break;
			}
		}

	}

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void cTracking::ForceRelocalisation()
{
    std::unique_lock<std::mutex> lock(mMutexForceRelocalisation);
    mbForceRelocalisation = true;
    mnLastRelocFrameId = mCurrentFrame.mnId;
}

bool cTracking::RelocalisationRequested()
{
    std::unique_lock<std::mutex> lock(mMutexForceRelocalisation);
    return mbForceRelocalisation;
}

void cTracking::Reset()
{
	this->mpViewer->RequestStop();

	cout << "System Reseting" << endl;
	while (!mpViewer->isStopped())
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	// Reset Local Mapping
	cout << "Reseting Local Mapper...";
	mpLocalMapper->RequestReset();
	cout << " done" << endl;

	// Reset Loop Closing
	cout << "Reseting Loop Closing...";
	mpLoopClosing->RequestReset();
	cout << " done" << endl;

	// Clear BoW Database
	cout << "Reseting Database...";
	mpKeyFrameDB->clear();
	cout << " done" << endl;

	// Clear Map (this erase MapPoints and KeyFrames)
	cout << "Clearing map...";
	mpMap->clear();
	cout << " done" << endl;
	cMultiKeyFrame::nNextId = 0;
	cMultiFrame::nNextId = 0;
	mState = NO_IMAGES_YET;

	if (mpInitializer)
	{
		delete mpInitializer;
		mpInitializer = static_cast<cMultiInitializer*>(NULL);
	}

	mpViewer->Release();

	// Reset statistics otherwise false poses will be evaluated
	allPosesBool = std::vector<bool>(allPosesBool.size(), false);
	allPoses.clear();
	allPosesBool.clear();
	nrTrackedPts.clear();
	inlierRatio.clear();
	timingFeatureExtraction.clear();
	timingTrackLocalMap.clear();
	timingInitalPoseEst.clear();

}

void cTracking::CheckResetByPublishers()
{
    bool bReseting = false;

    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        bReseting = mbReseting;
    }

    if(bReseting)
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mbPublisherStopped = true;
    }

    // Hold until reset is finished
    while(1)
    {
        {
			std::unique_lock<std::mutex> lock(mMutexReset);
            if(!mbReseting)
            {
                mbPublisherStopped=false;
                break;
            }
        }
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

bool cTracking::CheckFinished()
{
	return finished;
}
}
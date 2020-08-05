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
* Ra�l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
*/

#include "cMultiFrame.h"
#include "cConverter.h"


namespace MultiColSLAM
{
	long unsigned int cMultiFrame::nNextId = 0;
	bool cMultiFrame::mbInitialComputations = true;
	std::vector<int> cMultiFrame::mnMinX, cMultiFrame::mnMinY;
	std::vector<int> cMultiFrame::mnMaxX, cMultiFrame::mnMaxY;

	cMultiFrame::cMultiFrame()
	{}

	//Copy Constructor
	cMultiFrame::cMultiFrame(const cMultiFrame& mframe)
		:
		mpORBvocabulary(mframe.mpORBvocabulary),
		images(mframe.images),
		mTimeStamp(mframe.mTimeStamp),
		camSystem(mframe.camSystem),
		N(mframe.N),
		totalN(mframe.totalN),
		mvKeys(mframe.mvKeys),
		mvKeysRays(mframe.mvKeysRays),
		mBowVec(mframe.mBowVec),
		mFeatVec(mframe.mFeatVec),
		mDescriptors(mframe.mDescriptors),
		mDescriptorMasks(mframe.mDescriptorMasks),
		mvpMapPoints(mframe.mvpMapPoints),
		mvbOutlier(mframe.mvbOutlier),
		mfGridElementWidthInv(mframe.mfGridElementWidthInv),
		mfGridElementHeightInv(mframe.mfGridElementHeightInv),
		mnId(mframe.mnId),
		mpReferenceKF(mframe.mpReferenceKF),
		mnScaleLevels(mframe.mnScaleLevels),
		mfScaleFactor(mframe.mfScaleFactor),
		mvScaleFactors(mframe.mvScaleFactors),
		mvLevelSigma2(mframe.mvLevelSigma2),
		mvInvLevelSigma2(mframe.mvInvLevelSigma2),
		keypoint_to_cam(mframe.keypoint_to_cam),
		cont_idx_to_local_cam_idx(mframe.cont_idx_to_local_cam_idx),
		mdBRIEF(mframe.mdBRIEF),
		masksLearned(mframe.masksLearned),
		descDimension(mframe.descDimension),
		imgCnt(mframe.imgCnt),
		mp_mdBRIEF_extractorOct(mframe.mp_mdBRIEF_extractorOct)
	{
		int nrCams = camSystem.GetNrCams();
		mGrids.resize(nrCams);
		mnMinX.resize(nrCams);
		mnMaxX.resize(nrCams);
		mnMinY.resize(nrCams);
		mnMaxY.resize(nrCams);
		for (int c = 0; c < camSystem.GetNrCams(); ++c)
		{
			mGrids[c] = mframe.mGrids[c];

			mnMinX[c] = 0;
			mnMaxX[c] = camSystem.GetCamModelObj(c).GetWidth();
			mnMinY[c] = 0;
			mnMaxY[c] = camSystem.GetCamModelObj(c).GetHeight();
		}
	}

	cMultiFrame::cMultiFrame(const std::vector<cv::Mat>& images_,
		const double &timeStamp,
		std::vector<mdBRIEFextractorOct*> extractor,
		ORBVocabulary* voc,
		cMultiCamSys_& camSystem_,
		int _imgCnt)
		:
		mp_mdBRIEF_extractorOct(extractor),
		mpORBvocabulary(voc),
		images(images_),
		mTimeStamp(timeStamp),
		camSystem(camSystem_),
		mdBRIEF(true),
		imgCnt(_imgCnt)
	{
		HResClk::time_point begin = HResClk::now();

		int nrCams = camSystem.GetNrCams();
		mDescriptors.resize(nrCams);
		mDescriptorMasks.resize(nrCams);
		N.resize(nrCams);
		mnMinX.resize(nrCams);
		mnMaxX.resize(nrCams);
		mnMinY.resize(nrCams);
		mnMaxY.resize(nrCams);
		mfGridElementWidthInv.resize(nrCams);
		mfGridElementHeightInv.resize(nrCams);

		mGrids = std::vector<std::vector<std::vector<std::vector<size_t> > > >(nrCams);

		totalN = 0;

		std::vector<std::vector<cv::KeyPoint>> keyPtsTemp(nrCams);
		std::vector<std::vector<cv::Vec3d>> keyRaysTemp(nrCams);
		int laufIdx = 0;

#pragma omp parallel for num_threads(nrCams)
		for (int c = 0; c < nrCams; ++c)
		{
			cCamModelGeneral_ camModel = camSystem.GetCamModelObj(c);
			mnMinX[c] = 0;
			mnMaxX[c] = camModel.GetWidth();
			mnMinY[c] = 0;
			mnMaxY[c] = camModel.GetHeight();

			// First step feature extraction ORB in the mirror mask
			(*mp_mdBRIEF_extractorOct[c])(images[c], camModel.GetMirrorMask(0),
				keyPtsTemp[c], camModel, mDescriptors[c], mDescriptorMasks[c]);

			N[c] = (int)keyPtsTemp[c].size();

			// calculate rays as observations
			double x = 0.0, y = 0.0, z = 0.0;
			keyRaysTemp[c].resize(keyPtsTemp[c].size());
			for (unsigned int i = 0; i < keyPtsTemp[c].size(); i++)
			{
				camModel.ImgToWorld(x, y, z,
					static_cast<double>(keyPtsTemp[c][i].pt.x),
					static_cast<double>(keyPtsTemp[c][i].pt.y));
				keyRaysTemp[c][i] = cv::Vec3d(x, y, z);
			}

			mfGridElementWidthInv[c] = static_cast<double>(FRAME_GRID_COLS) /
				static_cast<double>(mnMaxX[c] - mnMinX[c]);
			mfGridElementHeightInv[c] = static_cast<double>(FRAME_GRID_ROWS) /
				static_cast<double>(mnMaxY[c] - mnMinY[c]);


			// Assign Features to Grid Cells
			mGrids[c] = std::vector<std::vector<std::vector<size_t> > >(FRAME_GRID_COLS);
			for (unsigned int j = 0; j < FRAME_GRID_COLS; ++j)
				mGrids[c][j] = std::vector<std::vector<size_t> >(FRAME_GRID_ROWS);
		}

		// now save the image points in an order and fill the keypoints_to_cam variable
		int currPtIdx = 0;
		for (int c = 0; c < nrCams; ++c)
		{
			totalN += N[c];
			for (int i = 0; i < keyRaysTemp[c].size(); ++i)
			{
				mvKeys.push_back(keyPtsTemp[c][i]);
				mvKeysRays.push_back(keyRaysTemp[c][i]);
				keypoint_to_cam[currPtIdx] = c;
				cont_idx_to_local_cam_idx[currPtIdx] = i;
				cv::KeyPoint &kp = keyPtsTemp[c][i];
				// fill grids
				int nGridPosX, nGridPosY;
				if (PosInGrid(c, kp, nGridPosX, nGridPosY))
					mGrids[c][nGridPosX][nGridPosY].push_back(currPtIdx);
				++currPtIdx;
			}
		}

		mvbOutlier = std::vector<bool>(totalN, false);
		// this must not be done for each image, only for each multi keyframe
		mvpMapPoints = std::vector<cMapPoint*>(totalN, static_cast<cMapPoint*>(NULL));
		// next id
		mnId = nNextId++;

					//Scale Levels Info
		mnScaleLevels = mp_mdBRIEF_extractorOct[0]->GetLevels();
		mfScaleFactor = mp_mdBRIEF_extractorOct[0]->GetScaleFactor();

		mvScaleFactors.resize(mnScaleLevels);
		mvLevelSigma2.resize(mnScaleLevels);
		mvScaleFactors[0] = 1.0;
		mvLevelSigma2[0] = 1.0;

		for (int i = 1; i < mnScaleLevels; ++i)
		{
			mvScaleFactors[i] = mvScaleFactors[i - 1] * mfScaleFactor;
			mvLevelSigma2[i] = mvScaleFactors[i] * mvScaleFactors[i];
		}

		mvInvLevelSigma2.resize(mvLevelSigma2.size());
		for (int i = 0; i < mnScaleLevels; ++i)
			mvInvLevelSigma2[i] = 1 / mvLevelSigma2[i];

		this->masksLearned = extractor[0]->GetMasksLearned();
		this->descDimension = extractor[0]->GetDescriptorSize();

		HResClk::time_point end = HResClk::now();
		cout << "---Feature Extraction (" << T_in_ms(begin, end) << "ms) - ImageId: " << mnId << "---" << endl;
	}

	bool cMultiFrame::isInFrustum(int cam, cMapPoint *pMP, double viewingCosLimit)
	{
		pMP->mbTrackInView[cam] = false;

		// 3D in absolute coordinates
		cv::Vec3d P = pMP->GetWorldPos();

		// Project in image and check it is not outside
		cv::Vec2d uv(0.0, 0.0);
		cv::Vec4d pt3 = cv::Vec4d(P(0), P(1), P(2), 1.0);
		camSystem.WorldToCamHom_fast(cam, pt3, uv);
		// check mirror border
		if (!camSystem.GetCamModelObj(cam).isPointInMirrorMask(uv(0), uv(1), 0))
			return false;

		// Check distance is in the scale invariance region of the MapPoint
		const double maxDistance = pMP->GetMaxDistanceInvariance();
		const double minDistance = pMP->GetMinDistanceInvariance();

		cv::Matx44d Tcw = camSystem.Get_MtMc(cam);
		const cv::Vec3d PO = P - cv::Vec3d(Tcw(0, 3), Tcw(1, 3), Tcw(2, 3));
		const double dist = cv::norm(PO);

		if (dist < minDistance || dist > maxDistance)
			return false;

		// Check viewing angle
		cv::Vec3d Pn = pMP->GetNormal();

		double viewCos = PO.dot(Pn) / dist;

		//if(viewCos < viewingCosLimit)
		//	return false;

		// Predict scale level according to the distance
		double ratio = dist / minDistance;

		std::vector<double>::iterator it = std::lower_bound(mvScaleFactors.begin(),
			mvScaleFactors.end(), ratio);
		int nPredictedLevel = it - mvScaleFactors.begin();

		if (nPredictedLevel >= mnScaleLevels)
			nPredictedLevel = mnScaleLevels - 1;

		// Data used by the tracking
		pMP->mbTrackInView[cam] = true;
		pMP->mTrackProjX[cam] = uv(0);
		pMP->mTrackProjY[cam] = uv(1);
		pMP->mnTrackScaleLevel[cam] = nPredictedLevel;
		pMP->mTrackViewCos[cam] = viewCos;

		return true;
	}

	std::vector<size_t> cMultiFrame::GetFeaturesInArea(const int& cam,
		const double &x,
		const double &y,
		const double &r,
		int minLevel, int maxLevel) const
	{
		std::vector<size_t> vIndices;

		int nMinCellX = floor((x - mnMinX[cam] - r)*mfGridElementWidthInv[cam]);
		nMinCellX = std::max(0, nMinCellX);
		if (nMinCellX >= FRAME_GRID_COLS)
			return vIndices;

		int nMaxCellX = ceil((x - mnMinX[cam] + r)*mfGridElementWidthInv[cam]);
		nMaxCellX = std::min(FRAME_GRID_COLS - 1, nMaxCellX);
		if (nMaxCellX < 0)
			return vIndices;

		int nMinCellY = floor((y - mnMinY[cam] - r)*mfGridElementHeightInv[cam]);
		nMinCellY = std::max(0, nMinCellY);
		if (nMinCellY >= FRAME_GRID_ROWS)
			return vIndices;

		int nMaxCellY = ceil((y - mnMinY[cam] + r)*mfGridElementHeightInv[cam]);
		nMaxCellY = std::min(FRAME_GRID_ROWS - 1, nMaxCellY);
		if (nMaxCellY < 0)
			return vIndices;

		bool bCheckLevels = true;
		bool bSameLevel = false;
		if (minLevel == -1 && maxLevel == -1)
			bCheckLevels = false;
		else
			if (minLevel == maxLevel)
				bSameLevel = true;

		for (int ix = nMinCellX; ix <= nMaxCellX; ++ix)
		{
			for (int iy = nMinCellY; iy <= nMaxCellY; ++iy)
			{
				const std::vector<size_t>& vCell = mGrids[cam][ix][iy];
				if (vCell.empty())
					continue;

				for (size_t j = 0, jend = vCell.size(); j < jend; ++j)
				{
					const cv::KeyPoint &kpUn = mvKeys[vCell[j]];
					if (bCheckLevels && !bSameLevel)
					{
						if (kpUn.octave < minLevel || kpUn.octave > maxLevel)
							continue;
					}
					else if (bSameLevel)
					{
						if (kpUn.octave != minLevel)
							continue;
					}

					if (abs(kpUn.pt.x - x) > r || abs(kpUn.pt.y - y) > r)
						continue;

					vIndices.push_back(vCell[j]);
				}
			}
		}

		return vIndices;

	}

	bool cMultiFrame::PosInGrid(const int& cam,
		cv::KeyPoint &kp, int &posX, int &posY)
	{
		posX = cvRound((kp.pt.x - mnMinX[cam])*mfGridElementWidthInv[cam]);
		posY = cvRound((kp.pt.y - mnMinY[cam])*mfGridElementHeightInv[cam]);

		//Keypoint's coordinates are undistorted, which could cause to go out of the image
		if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
			return false;

		return true;
	}


	void cMultiFrame::ComputeBoW()
	{
		if (mBowVec.empty())
		{
			std::vector<cv::Mat> vCurrentDesc = cConverter::toDescriptorVector(mDescriptors);
			mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
		}
	}
}
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

#include "cORBmatcher.h"

#include <limits.h>

//#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "DBoW2/DBoW2/FeatureVector.h"
#include "cConverter.h"

namespace MultiColSLAM
{

using namespace std;


const int cORBmatcher::HISTO_LENGTH = 30;

cORBmatcher::cORBmatcher(double nnratio, 
	bool checkOri, const int featDim, bool havingMasks_) : 
mfNNratio(nnratio), 
mbCheckOrientation(checkOri), 
mbFeatDim(featDim), 
havingMasks(havingMasks_)
{
	// change thresholds if we are using masks, as the hamming distance distribution
	// for matching and non-matching points differs
	if (havingMasks_)
	{
		TH_HIGH_ = floor(1.5*featDim);
		TH_LOW_ = floor(featDim);
	}
	else
	{
		TH_HIGH_ = 3 * featDim;
		TH_LOW_ = 2 * featDim;
	}
}

int cORBmatcher::SearchByProjection(cMultiFrame &F,
	const vector<cMapPoint*> &vpMapPoints, 
	const double th)
{
    int nmatches=0;

    const bool bFactor = th != 1.0;

	for (size_t iMP = 0; iMP < vpMapPoints.size(); ++iMP)
	{
		cMapPoint* pMP = vpMapPoints[iMP];
		// if point is bad skip
		if (pMP->isBad())
			continue;
		for (int cam = 0; cam < F.camSystem.GetNrCams(); ++cam)
		{
			// if this point was not projected to this cam, skip
			if (!pMP->mbTrackInView[cam])
				continue;

			const int &nPredictedLevel = pMP->mnTrackScaleLevel[cam];

			// The size of the window will depend on the viewing direction
			double r = RadiusByViewingCos(pMP->mTrackViewCos[cam]);

			if (bFactor)
				r *= th;

			vector<size_t> vNearIndices =
				F.GetFeaturesInArea(cam, pMP->mTrackProjX[cam], pMP->mTrackProjY[cam],
				r*F.mvScaleFactors[nPredictedLevel], nPredictedLevel - 1,
				nPredictedLevel);
			//vector<size_t> vNearIndices =
			//	F.GetFeaturesInArea(cam, pMP->mTrackProjX[cam], pMP->mTrackProjY[cam], r*10);
			if (vNearIndices.empty())
				continue;

			const uint64_t* ptrMPdesc = pMP->GetDescriptorPtr();
			const uint64_t* ptrMPdesc_mask = 0;
			if (havingMasks)
				ptrMPdesc_mask = pMP->GetDescriptorMaskPtr();

			int bestDist = INT_MAX;
			int bestLevel = -1;
			int bestDist2 = INT_MAX;
			int bestLevel2 = -1;
			int bestIdx = -1;

			// Get best and second matches with near keypoints
			for (vector<size_t>::iterator vit = vNearIndices.begin(), vend = vNearIndices.end();
				vit != vend; vit++)
			{
				size_t idx = *vit;
				// do we have a point assigned already?
				if (F.mvpMapPoints[idx])
					continue;

				int descIdx = F.cont_idx_to_local_cam_idx.find(idx)->second;
				const uint64_t* d1 = F.mDescriptors[cam].ptr<uint64_t>(descIdx);
				
				int dist = 0;
				// masked distance
				if (havingMasks)
				{
					const uint64_t* d1_mask = F.mDescriptorMasks[cam].ptr<uint64_t>(descIdx);
					dist = DescriptorDistance64Masked(ptrMPdesc, d1, 
						ptrMPdesc_mask, d1_mask, mbFeatDim);
				}
				else
					dist = DescriptorDistance64(ptrMPdesc, d1, mbFeatDim);

				if (dist < bestDist)
				{
					bestDist2 = bestDist;
					bestDist = dist;
					bestLevel2 = bestLevel;
					bestLevel = F.mvKeys[idx].octave;
					bestIdx = idx;
				}
				else if (dist < bestDist2)
				{
					bestLevel2 = F.mvKeys[idx].octave;
					bestDist2 = dist;
				}
			}

			// Apply ratio to second match (only if best and second are in the same scale level)
			if (bestDist <= TH_HIGH_)
			{
				if (bestLevel == bestLevel2 && bestDist > mfNNratio*bestDist2)
					continue;

				F.mvpMapPoints[bestIdx] = pMP;
				++nmatches;
			}
		}
	}

    return nmatches;
}

// HERE - DO THIS WITH COVARIANCE INFORMAION
double cORBmatcher::RadiusByViewingCos(const double &viewCos)
{
    if (viewCos > 0.998)
        return 2.5;
    else
        return 4.0;
}



int cORBmatcher::SearchByBoW(cMultiKeyFrame* pKF, 
	cMultiFrame &F,
	vector<cMapPoint*> &vpMapPointMatches)
{
    vector<cMapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

    vpMapPointMatches = vector<cMapPoint*>(F.mvpMapPoints.size(),static_cast<cMapPoint*>(NULL));

    DBoW2::FeatureVector vFeatVecKF = pKF->GetFeatureVector();

    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];
    for(int i = 0;i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::iterator Fend = F.mFeatVec.end();

	// over all cameras
	while (KFit != KFend && Fit != Fend)
	{
		if (KFit->first == Fit->first)
		{
			vector<unsigned int> vIndicesKF = KFit->second;
			vector<unsigned int> vIndicesF = Fit->second;

			for (size_t iKF = 0, iendKF = vIndicesKF.size(); iKF < iendKF; ++iKF)
			{
				const unsigned int realIdxKF = vIndicesKF[iKF];

				cMapPoint* pMP = vpMapPointsKF[realIdxKF];

				if (!pMP)
					continue;

				if (pMP->isBad())
					continue;
				int descIdx1 = pKF->cont_idx_to_local_cam_idx.find(realIdxKF)->second;
				int camIdx1 = pKF->keypoint_to_cam.find(realIdxKF)->second;
				const uint64_t* dKF = pKF->GetDescriptorRowPtr(camIdx1, descIdx1);
				const uint64_t* dKF_mask = 0;
				if (havingMasks)
					dKF_mask = pKF->GetDescriptorMaskRowPtr(camIdx1, descIdx1);


				int bestDist1 = INT_MAX;
				int bestIdxF = -1;
				int bestDist2 = INT_MAX;

				for (size_t iF = 0, iendF = vIndicesF.size(); iF < iendF; ++iF)
				{
					const unsigned int realIdxF = vIndicesF[iF];

					if (vpMapPointMatches[realIdxF])
						continue;
					int descIdx2 = F.cont_idx_to_local_cam_idx.find(realIdxF)->second;
					int camIdx2 = F.keypoint_to_cam.find(realIdxF)->second;
					const uint64_t* dF = F.mDescriptors[camIdx2].ptr<uint64_t>(descIdx2);
					int dist = 0;
					if (havingMasks)
					{
						const uint64_t* dF_mask = F.mDescriptorMasks[camIdx2].ptr<uint64_t>(descIdx2);
						dist = DescriptorDistance64Masked(dKF, dF, 
							dKF_mask, dF_mask, mbFeatDim);
					}
					else
						dist = DescriptorDistance64(dKF, dF, mbFeatDim);

					if (dist < bestDist1)
					{
						bestDist2 = bestDist1;
						bestDist1 = dist;
						bestIdxF = realIdxF;
					}
					else if (dist < bestDist2)
					{
						bestDist2 = dist;
					}
				}

				if (bestDist1 <= TH_LOW_)
				{
					if (static_cast<double>(bestDist1) < mfNNratio*static_cast<double>(bestDist2))
					{
						vpMapPointMatches[bestIdxF] = pMP;

						cv::KeyPoint kp = pKF->GetKeyPoint(realIdxKF);

						if (mbCheckOrientation)
						{
							float rot = kp.angle - F.mvKeys[bestIdxF].angle;
							if (rot < 0.0)
								rot += 360.0f;
							int bin = cvRound(rot*factor);
							if (bin == HISTO_LENGTH)
								bin = 0;
							//ROS_ASSERT(bin>=0 && bin<HISTO_LENGTH);
							rotHist[bin].push_back(bestIdxF);
						}
						++nmatches;
					}
				}

			}

			++KFit;
			++Fit;
		}
		else if (KFit->first < Fit->first)
		{
			KFit = vFeatVecKF.lower_bound(Fit->first);
		}
		else
		{
			Fit = F.mFeatVec.lower_bound(KFit->first);
		}
	}


	if (mbCheckOrientation)
	{
		int ind1 = -1;
		int ind2 = -1;
		int ind3 = -1;

		ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

		for (int i = 0; i < HISTO_LENGTH; i++)
		{
			if (i == ind1 || i == ind2 || i == ind3)
				continue;
			for (size_t j = 0, jend = rotHist[i].size(); j < jend; ++j)
			{
				vpMapPointMatches[rotHist[i][j]] = NULL;
				--nmatches;
			}
		}
	}
    return nmatches;
}


int cORBmatcher::WindowSearch(cMultiFrame &F1, cMultiFrame &F2,
	int windowSize, 
	vector<cMapPoint *> &vpMapPointMatches2, 
	int minScaleLevel, 
	int maxScaleLevel)
{
    int nmatches=0;
    vpMapPointMatches2 = vector<cMapPoint*>(F2.mvpMapPoints.size(),static_cast<cMapPoint*>(NULL));
    vector<int> vnMatches21 = vector<int>(F2.mvKeys.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
	for (int i = 0; i < HISTO_LENGTH; ++i)
        rotHist[i].reserve(500);
	const double factor = 1.0f / HISTO_LENGTH;

    const bool bMinLevel = minScaleLevel > 0;
    const bool bMaxLevel = maxScaleLevel < INT_MAX;

	for (size_t i1 = 0, iend1 = F1.mvpMapPoints.size(); i1 < iend1; ++i1)
    {
        cMapPoint* pMP1 = F1.mvpMapPoints[i1];

        if(!pMP1)
            continue;

        if(pMP1->isBad())
            continue;

        const cv::KeyPoint &kp1 = F1.mvKeys[i1];
        int level1 = kp1.octave;

        if (bMinLevel)
            if (level1 < minScaleLevel)
                continue;

        if (bMaxLevel)
            if (level1 > maxScaleLevel)
                continue;

		int camIdx1 = F1.keypoint_to_cam.find(i1)->second;
        vector<size_t> vIndices2 = 
			F2.GetFeaturesInArea(camIdx1, kp1.pt.x, kp1.pt.y, 
			windowSize);

        if (vIndices2.empty())
            continue;

		int descIdx = F1.cont_idx_to_local_cam_idx.find(i1)->second;
		const uint64_t* d1 = F1.mDescriptors[camIdx1].ptr<uint64_t>(descIdx);
		const uint64_t* d1_mask = 0;
		if (havingMasks)
			d1_mask = F1.mDescriptorMasks[camIdx1].ptr<uint64_t>(descIdx);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;
		int camIdx2 = 0;
		for (vector<size_t>::iterator vit = vIndices2.begin(), vend = vIndices2.end(); 
			vit != vend; ++vit)
        {
            size_t i2 = *vit;

            if (vpMapPointMatches2[i2])
                continue;

			int descIdx2 = F2.cont_idx_to_local_cam_idx.find(i2)->second;
			camIdx2 = F2.keypoint_to_cam.find(i2)->second;

			const uint64_t* d2 = F2.mDescriptors[camIdx2].ptr<uint64_t>(descIdx2);
			int dist = 0;
			if (havingMasks)
			{
				const uint64_t* d2_mask = F2.mDescriptorMasks[camIdx2].ptr<uint64_t>(descIdx2);
				dist = DescriptorDistance64Masked(d1, d2, d1_mask, d2_mask, mbFeatDim);
			}
			else
				dist = DescriptorDistance64(d1, d2, mbFeatDim);
            //int dist = DescriptorDistance(d1,d2);
			
            if (dist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = dist;
                bestIdx2 = i2;
            } else if (dist < bestDist2)
            {
                bestDist2 = dist;
            }
        }

        if (bestDist <= bestDist2*mfNNratio && bestDist <= TH_HIGH_)
        {
			//cv::Vec2d uv1,uv2;
			//F1.camSystem.WorldToCamHom_fast(camIdx1, pMP1->GetWorldPos(), uv1);
			//F2.camSystem.WorldToCamHom_fast(camIdx2, pMP1->GetWorldPos(), uv2);
			//if (!F1.camSystem.GetCamModelObj(camIdx1).isPointInMirrorMask(uv1(0), uv1(1), 0))
			//	continue;
			//cv::circle(imsRes1[camIdx1], cv::Point(uv1), 3, cv::Scalar(0.0, 0.0, 250.0));
			//if (!F2.camSystem.GetCamModelObj(camIdx2).isPointInMirrorMask(uv2(0), uv2(1), 0))
			//	continue;
			//cv::circle(imsRes2[camIdx2], cv::Point(uv2), 3, cv::Scalar(0.0, 0.0, 250.0));

            vpMapPointMatches2[bestIdx2] = pMP1;
            vnMatches21[bestIdx2] = i1;
            nmatches++;

            float rot = F1.mvKeys[i1].angle - F2.mvKeys[bestIdx2].angle;
            if (rot < 0.0)
                rot += 360.0f;
            int bin = cvRound(rot*factor);
            if (bin == HISTO_LENGTH)
                bin = 0;
            rotHist[bin].push_back(bestIdx2);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

		ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

		for (int i = 0; i<HISTO_LENGTH; ++i)
        {
			if (i != ind1 && i != ind2 && i != ind3)
            {
				for (size_t j = 0, jend = rotHist[i].size(); j<jend; ++j)
                {
					vpMapPointMatches2[rotHist[i][j]] = NULL;
					vnMatches21[rotHist[i][j]] = -1;
					--nmatches;
                }
            }
        }
    }

	//cv::Mat imL1 = imsRes1[0];
	//cv::Mat imL2 = imsRes1[1];
	//cv::Mat imL3 = imsRes1[2];

	//cv::Mat imR1 = imsRes2[0];
	//cv::Mat imR2 = imsRes2[1];
	//cv::Mat imR3 = imsRes2[2];

    return nmatches;
}


int cORBmatcher::SearchByProjection(cMultiFrame &F1, 
	cMultiFrame &F2,
	int windowSize, 
	vector<cMapPoint *> &vpMapPointMatches2)
{
    vpMapPointMatches2 = F2.mvpMapPoints;
    set<cMapPoint*> spMapPointsAlreadyFound(vpMapPointMatches2.begin(),
		vpMapPointMatches2.end());

    int nmatches = 0;

	std::unordered_map<cMapPoint*, int> mapPt_2_obs_idx;
	for (size_t i1 = 0, iend1 = F1.mvpMapPoints.size(); i1 < iend1; ++i1)
    {
        cMapPoint* pMP1 = F1.mvpMapPoints[i1];

        if (!pMP1)
            continue;
        if (pMP1->isBad() || spMapPointsAlreadyFound.count(pMP1))
            continue;
		// if we worked with this mappoint already
		if (mapPt_2_obs_idx.count(pMP1) > 0)
			continue;

		mapPt_2_obs_idx[pMP1] = i1;

        cv::KeyPoint kp1 = F1.mvKeys[i1];
        int level1 = kp1.octave;

		int camIdxFeat = F1.keypoint_to_cam.find(i1)->second;

        cv::Vec3d x3Dw = pMP1->GetWorldPos();
		// project the point in each camera
		for (int c = 0; c < F1.camSystem.GetNrCams(); ++c)
		{
			cv::Vec2d uv;
			F2.camSystem.WorldToCamHom_fast(c, x3Dw, uv);
			if (F2.camSystem.GetCamModelObj(c).isPointInMirrorMask(uv(0), uv(1), 0))
			{
				vector<size_t> vIndices2 = F2.GetFeaturesInArea(c, uv(0), uv(1),
					windowSize, level1, level1);
				//vector<size_t> vIndices2 = F2.GetFeaturesInArea(c, uv(0), uv(1),
				//	windowSize);
				if (vIndices2.empty())
					continue;
				
				int descIdx = F1.cont_idx_to_local_cam_idx.find(i1)->second;
				const uint64_t* d1 = F1.mDescriptors[camIdxFeat].ptr<uint64_t>(descIdx);
				const uint64_t* d1_mask = 0;
				if (havingMasks)
					d1_mask = F1.mDescriptorMasks[camIdxFeat].ptr<uint64_t>(descIdx);

				// match
				int bestDist = INT_MAX;
				int bestDist2 = INT_MAX;
				int bestIdx2 = -1;
				int camIdx2 = 0;

				for (vector<size_t>::iterator vit = vIndices2.begin(), vend = vIndices2.end();
					vit != vend; ++vit)
				{
					size_t i2 = *vit;

					if (vpMapPointMatches2[i2])
						continue;

					int descIdx2 = F2.cont_idx_to_local_cam_idx.find(i2)->second;
					const uint64_t* d2 = F2.mDescriptors[c].ptr<uint64_t>(descIdx2);
					int dist = 0;
					if (havingMasks)
					{
						const uint64_t* d2_mask = F2.mDescriptorMasks[c].ptr<uint64_t>(descIdx2);
						dist = DescriptorDistance64Masked(d1, d2, d1_mask, d2_mask, mbFeatDim);
					}
					else
						dist = DescriptorDistance64(d1, d2, mbFeatDim);

					if (dist < bestDist)
					{
						bestDist2 = bestDist;
						bestDist = dist;
						bestIdx2 = i2;
					}
					else if (dist < bestDist2)
					{
						bestDist2 = dist;
					}
				}
				if (static_cast<double>(bestDist) <=
					static_cast<double>(bestDist2)*mfNNratio &&
					bestDist <= TH_HIGH_)
				{

					vpMapPointMatches2[bestIdx2] = pMP1;
					++nmatches;
				}
			}
		}
    }

    return nmatches;
}

int cORBmatcher::SearchForInitialization(cMultiFrame &F1, 
	cMultiFrame &F2,
	vector<cv::Vec2d> &vbPrevMatched, 
	vector<int> &vnMatches12, 
	int windowSize)
{
	HResClk::time_point begin = HResClk::now();

    int nmatches = 0;
    vnMatches12 = vector<int>(F1.mvKeys.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
	for (int i = 0; i < HISTO_LENGTH; ++i)
        rotHist[i].reserve(500);

    const double factor = 1.0 / HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeys.size(),INT_MAX);
    vector<int> vnMatches21(F2.mvKeys.size(),-1);

	for (size_t i1 = 0, iend1 = F1.mvKeys.size(); i1<iend1; ++i1)
    {
        cv::KeyPoint kp1 = F1.mvKeys[i1];
        int level1 = kp1.octave;
        //if (level1 > 0)
        //    continue;

		int camIdx1 = F1.keypoint_to_cam.find(i1)->second;
		
        vector<size_t> vIndices2 = 
			F2.GetFeaturesInArea(camIdx1, vbPrevMatched[i1](0), 
								 vbPrevMatched[i1](1),
								 windowSize,level1,level1);
		//cout << "vIndices2: " << vIndices2.size() << endl;
		//vector<size_t> vIndices2 =
		//	F2.GetFeaturesInArea(camIdx1, vbPrevMatched[i1](0), vbPrevMatched[i1](1),
		//	windowSize);
        if(vIndices2.empty())
            continue;

		int descIdx1 = F1.cont_idx_to_local_cam_idx.find(i1)->second;

		const uint64_t* d1 = F1.mDescriptors[camIdx1].ptr<uint64_t>(descIdx1);
		const uint64_t* d1_mask = 0;
		if (havingMasks)
			d1_mask = F1.mDescriptorMasks[camIdx1].ptr<uint64_t>(descIdx1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

		for (vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); ++vit)
        {
            size_t i2 = *vit;

			int camIdx2 = F2.keypoint_to_cam.find(i2)->second;
			int descIdx2 = F2.cont_idx_to_local_cam_idx.find(i2)->second;

			const uint64_t* d2 = F2.mDescriptors[camIdx2].ptr<uint64_t>(descIdx2);
			int dist = 0;
			if (havingMasks)
			{
				const uint64_t* d2_mask = F2.mDescriptorMasks[camIdx2].ptr<uint64_t>(descIdx2);
				dist = DescriptorDistance64Masked(d1, d2, d1_mask, d2_mask, mbFeatDim);
			}
			else
				dist = DescriptorDistance64(d1, d2, mbFeatDim);

            if (vMatchedDistance[i2] <= dist)
                continue;

            if (dist < bestDist)
            {
				bestDist2 = bestDist;
				bestDist = dist;
				bestIdx2 = i2;
            }
            else if (dist < bestDist2)
            {
				bestDist2 = dist;
            }
        }

		if (bestDist <= TH_LOW_)
        {
			if (bestDist < (double)bestDist2*mfNNratio)
            {
                if (vnMatches21[bestIdx2] >= 0)
                {
                    vnMatches12[vnMatches21[bestIdx2]] =- 1;
                    nmatches--;
                }
                vnMatches12[i1] = bestIdx2;
                vnMatches21[bestIdx2] = i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
					float rot = F1.mvKeys[i1].angle - F2.mvKeys[bestIdx2].angle;
                    if (rot < 0.0)
                        rot += 360.0f;
                    int bin = round(rot*factor);
                    if (bin == HISTO_LENGTH)
                        bin = 0;
                    //ROS_ASSERT(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
					--nmatches;
                }
            }
        }

    }

    //Update prev matched
	for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; ++i1)
        if (vnMatches12[i1] >= 0)
			vbPrevMatched[i1] = 
				cv::Vec2d(F2.mvKeys[vnMatches12[i1]].pt.x, F2.mvKeys[vnMatches12[i1]].pt.y);

	HResClk::time_point end = HResClk::now();
	cout << "---matching time (" << T_in_ms(begin, end) << ")--- nr:" << nmatches<<" "<< endl;
    return nmatches;
}

//int cORBmatcher::SearchByBoW(cMultiKeyFrame *pKF1, 
//	cMultiKeyFrame *pKF2, 
//	vector<cMapPoint *> &vpMatches12)
//{
//    vector<cv::KeyPoint> vKeys1 = pKF1->GetKeyPoints();
//    DBoW2::FeatureVector vFeatVec1 = pKF1->GetFeatureVector();
//    vector<cMapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
//	std::vector<cv::Mat> Descriptors1 = pKF1->GetAllDescriptors();
//	std::vector<cv::Mat> Descriptors1masks = pKF1->GetAllDescriptorMasks();
//
//    vector<cv::KeyPoint> vKeys2 = pKF2->GetKeyPoints();
//    DBoW2::FeatureVector vFeatVec2 = pKF2->GetFeatureVector();
//    vector<cMapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
//	std::vector<cv::Mat> Descriptors2 = pKF2->GetAllDescriptors();
//	std::vector<cv::Mat> Descriptors2masks = pKF2->GetAllDescriptorMasks();
//
//    vpMatches12 = vector<cMapPoint*>(vpMapPoints1.size(),static_cast<cMapPoint*>(NULL));
//    vector<bool> vbMatched2(vpMapPoints2.size(),false);
//
//    vector<int> rotHist[HISTO_LENGTH];
//	for (int i = 0; i < HISTO_LENGTH; ++i)
//        rotHist[i].reserve(500);
//
//    const float factor = 1.0f/HISTO_LENGTH;
//
//    int nmatches = 0;
//
//    DBoW2::FeatureVector::iterator f1it = vFeatVec1.begin();
//    DBoW2::FeatureVector::iterator f2it = vFeatVec2.begin();
//    DBoW2::FeatureVector::iterator f1end = vFeatVec1.end();
//    DBoW2::FeatureVector::iterator f2end = vFeatVec2.end();
//
//    while (f1it != f1end && f2it != f2end)
//    {
//        if (f1it->first == f2it->first)
//        {
//			for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; ++i1)
//            {
//                size_t idx1 = f1it->second[i1];
//
//                cMapPoint* pMP1 = vpMapPoints1[idx1];
//
//                if(!pMP1)
//                    continue;
//
//                if(pMP1->isBad())
//                    continue;
//
//				int camIdx1 = pKF1->keypoint_to_cam.find(i1)->second;
//				int descIdx1 = pKF1->cont_idx_to_local_cam_idx.find(i1)->second;
//				
//				const uint64_t* d1 = Descriptors1[camIdx1].ptr<uint64_t>(descIdx1);
//				const uint64_t* d1_mask = 0;
//				if (havingMasks)
//					d1_mask = Descriptors1masks[camIdx1].ptr<uint64_t>(descIdx1);
//
//                int bestDist1=INT_MAX;
//                int bestIdx2 =-1 ;
//                int bestDist2=INT_MAX;
//
//				for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; ++i2)
//                {
//                    size_t idx2 = f2it->second[i2];
//
//                    cMapPoint* pMP2 = vpMapPoints2[idx2];
//
//                    if (vbMatched2[idx2] || !pMP2)
//                        continue;
//
//                    if (pMP2->isBad())
//                        continue;
//
//					int camIdx2 = pKF2->keypoint_to_cam.find(i2)->second;
//					int descIdx2 = pKF2->cont_idx_to_local_cam_idx.find(i2)->second;
//
//					const uint64_t* d2 = Descriptors2[camIdx2].ptr<uint64_t>(descIdx2);
//					int dist = 0;
//					if (havingMasks)
//					{
//						const uint64_t* d2_mask = Descriptors2masks[camIdx2].ptr<uint64_t>(descIdx2);
//						dist = DescriptorDistance64Masked(d1, d2, d1_mask,d2_mask, mbFeatDim);
//					}
//					else
//						dist = DescriptorDistance64(d1, d2, mbFeatDim);
//
//                    if (dist < bestDist1)
//                    {
//                        bestDist2 = bestDist1;
//                        bestDist1 = dist;
//                        bestIdx2 = idx2;
//                    }
//                    else if (dist < bestDist2)
//                        bestDist2 = dist;
//                }
//
//                if (bestDist1 < TH_LOW_)
//                {
//					if (static_cast<double>(bestDist1) < 
//						mfNNratio*static_cast<double>(bestDist2))
//                    {
//                        vpMatches12[idx1] = vpMapPoints2[bestIdx2];
//                        vbMatched2[bestIdx2] = true;
//
//                        if(mbCheckOrientation)
//                        {
//                            float rot = vKeys1[idx1].angle-vKeys2[bestIdx2].angle;
//                            if (rot < 0.0)
//                                rot+=360.0f;
//                            int bin = round(rot*factor);
//                            if (bin==HISTO_LENGTH)
//                                bin=0;
//                            //ROS_ASSERT(bin>=0 && bin<HISTO_LENGTH);
//                            rotHist[bin].push_back(idx1);
//                        }
//						++nmatches;
//                    }
//                }
//            }
//
//			++f1it;
//			++f2it;
//        }
//        else if(f1it->first < f2it->first)
//        {
//            f1it = vFeatVec1.lower_bound(f2it->first);
//        }
//        else
//        {
//            f2it = vFeatVec2.lower_bound(f1it->first);
//        }
//    }
//
//    if (mbCheckOrientation)
//    {
//		int ind1 = -1;
//		int ind2 = -1;
//		int ind3 = -1;
//
//		ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);
//
//        for(int i=0; i<HISTO_LENGTH; i++)
//        {
//            if(i==ind1 || i==ind2 || i==ind3)
//                continue;
//			for (size_t j = 0, jend = rotHist[i].size(); j<jend; ++j)
//            {
//                vpMatches12[rotHist[i][j]]=NULL;
//                //vnMatches12[rotHist[i][j]]=-1;
//				--nmatches;
//            }
//        }
//
//    }
//
//    return nmatches;
//}

int cORBmatcher::SearchByBoW(cMultiKeyFrame *pKF1,
	cMultiKeyFrame *pKF2,
	vector<cMapPoint *> &vpMatches12)
{
	vector<cMapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
	std::vector<cv::Mat> Descriptors1 = pKF1->GetAllDescriptors();
	std::vector<cv::Mat> Descriptors1masks = pKF1->GetAllDescriptorMasks();

	vector<cMapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
	std::vector<cv::Mat> Descriptors2 = pKF2->GetAllDescriptors();
	std::vector<cv::Mat> Descriptors2masks = pKF2->GetAllDescriptorMasks();
	vpMatches12 = vector<cMapPoint*>(vpMapPoints1.size(), static_cast<cMapPoint*>(NULL));
	vector<bool> vbMatched2(vpMapPoints2.size(), false);

	int nmatches = 0;
	// loop through all map point of KF1
	for (size_t idx1 = 0; idx1 < vpMapPoints1.size(); ++idx1)
	{
		cMapPoint* pMP1 = vpMapPoints1[idx1];
		if (!pMP1)
			continue;
		if (pMP1->isBad())
			continue;

		int camIdx1 = pKF1->keypoint_to_cam.find(idx1)->second;
		int descIdx1 = pKF1->cont_idx_to_local_cam_idx.find(idx1)->second;
		const uint64_t* d1 = pKF1->GetDescriptorRowPtr(camIdx1, descIdx1);
		const uint64_t* d1_mask = 0;
		if (havingMasks)
			d1_mask = pKF1->GetDescriptorMaskRowPtr(camIdx1, descIdx1);

		// match so second MKF
        int bestDist1=INT_MAX;
        int bestIdx2 =-1 ;
        int bestDist2=INT_MAX;
		for (size_t idx2 = 0; idx2 < vpMapPoints2.size(); ++idx2)
		{
			cMapPoint* pMP2 = vpMapPoints2[idx2];
			
			if (vbMatched2[idx2] || !pMP2)
			    continue;	
			if (pMP2->isBad())
			    continue;

			int camIdx2 = pKF2->keypoint_to_cam.find(idx2)->second;
			int descIdx2 = pKF2->cont_idx_to_local_cam_idx.find(idx2)->second;

			const uint64_t* d2 = pKF2->GetDescriptorRowPtr(camIdx2, descIdx2);
			int dist = 0;
			if (havingMasks)
			{
				const uint64_t* d2_mask = pKF2->GetDescriptorMaskRowPtr(camIdx2, descIdx2);
				dist = DescriptorDistance64Masked(d1, d2, d1_mask,d2_mask, mbFeatDim);
			}
			else
				dist = DescriptorDistance64(d1, d2, mbFeatDim);

			if (dist < bestDist1)
			{
			    bestDist2 = bestDist1;
			    bestDist1 = dist;
			    bestIdx2 = idx2;
			}
			else if (dist < bestDist2)
			    bestDist2 = dist;
		}
		// ratio to second best and threshold test
		if (bestDist1 < TH_LOW_)
		{
			if (static_cast<double>(bestDist1) <
				mfNNratio*static_cast<double>(bestDist2))
			{
				vpMatches12[idx1] = vpMapPoints2[bestIdx2];
				vbMatched2[bestIdx2] = true;
				++nmatches;
			}
		}

	}

	return nmatches;
}

int cORBmatcher::SearchForTriangulationRaw(cMultiKeyFrame *pKF1,
	cMultiKeyFrame *pKF2,
	std::vector<cv::KeyPoint> &vMatchedKeys1,
	std::vector<cv::Vec3d> &vMatchedKeysRays1,
	std::vector<cv::KeyPoint> &vMatchedKeys2,
	std::vector<cv::Vec3d> &vMatchedKeysRays2,
	std::vector<std::pair<size_t, size_t> > &vMatchedPairs)
{
	vector<cMapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
	vector<cv::KeyPoint> vKeys1 = pKF1->GetKeyPoints();
	vector<cv::Vec3d> vKeysRays1 = pKF1->GetKeyPointsRays();


	vector<cMapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
	vector<cv::KeyPoint> vKeys2 = pKF2->GetKeyPoints();
	vector<cv::Vec3d> vKeysRays2 = pKF2->GetKeyPointsRays();

	// precompute essential matrices
	int nrCams = pKF1->camSystem.GetNrCams();
	vector<vector<cv::Matx33d>> Es(3);
	for (int i = 0; i < nrCams; ++i)
	{
		vector<cv::Matx33d> tmp(3);
		for (int j = 0; j < nrCams; ++j)
		{
			cv::Matx44d Mcs1 = pKF1->camSystem.Get_MtMc_inv(i);
			cv::Matx44d Mcs2 = pKF2->camSystem.Get_MtMc(j);
			cv::Matx33d E12 = ComputeE(Mcs1, Mcs2);
			tmp[j] = E12;
		}
		Es[i] = tmp;
	}

	// Find matches between not tracked keypoints
	// Matching speeded-up by ORB Vocabulary
	// Compare only ORB that share the same node
	int nmatches = 0;
	vector<bool> vbMatched2(vKeys2.size(), false);
	vector<int> vMatches12(vKeys1.size(), -1);

	vector<int> rotHist[HISTO_LENGTH];
	for (int i = 0; i < HISTO_LENGTH; ++i)
		rotHist[i].reserve(500);

	const double factor = 1.0 / HISTO_LENGTH;

	// loop through all map point of KF1
	for (size_t idx1 = 0; idx1 < vpMapPoints1.size(); ++idx1)
	{
		cMapPoint* pMP1 = vpMapPoints1[idx1];
		// If there is already a MapPoint associated skip
		if (pMP1)
			continue;

		// get observations
		const cv::KeyPoint &kp1 = vKeys1[idx1];
		const cv::Vec3d &ray1 = vKeysRays1[idx1];

		int camIdx1 = pKF1->keypoint_to_cam.find(idx1)->second;
		int descIdx1 = pKF1->cont_idx_to_local_cam_idx.find(idx1)->second;
		const uint64_t* d1 = pKF1->GetDescriptorRowPtr(camIdx1, descIdx1);
		const uint64_t* d1_mask = 0;
		if (havingMasks)
			d1_mask = pKF1->GetDescriptorMaskRowPtr(camIdx1, descIdx1);


		vector<pair<int, size_t> > vDistIndex;
		vector<int> vDistCamIndex; // cam idx of match
		// match so second multikeyframe
		for (size_t idx2 = 0; idx2 < vpMapPoints2.size(); ++idx2)
		{
			cMapPoint* pMP2 = vpMapPoints2[idx2];

			// If we have already matched or there is a MapPoint skip
			if (vbMatched2[idx2] || pMP2)
				continue;
			// get corresponding descriptor in second image
			int camIdx2 = pKF2->keypoint_to_cam.find(idx2)->second;
			//TODO for the moment take only matches between the same camera
			if (camIdx1 != camIdx2)
				continue;
			int descIdx2 = pKF2->cont_idx_to_local_cam_idx.find(idx2)->second;

			const uint64_t* d2 = pKF2->GetDescriptorRowPtr(camIdx2, descIdx2);

			int dist = 0;
			if (havingMasks)
			{ 
				const uint64_t* d2_mask = pKF2->GetDescriptorMaskRowPtr(camIdx2, descIdx2);
				dist = DescriptorDistance64Masked(d1, d2, d1_mask, d2_mask, mbFeatDim);
			}
			else
				dist = DescriptorDistance64(d1, d2, mbFeatDim);

			if (dist > TH_LOW_)
				continue;

			vDistIndex.push_back(make_pair(dist, idx2));
			vDistCamIndex.push_back(camIdx2);
		}

		if (vDistIndex.empty())
			continue;

		sort(vDistIndex.begin(), vDistIndex.end());
		int BestDist = vDistIndex.front().first;
		int DistTh = cvRound(2 * BestDist);
		for (size_t id = 0; id < vDistIndex.size(); ++id)
		{
			if (vDistIndex[id].first > DistTh)
				break;

			int currentIdx2 = vDistIndex[id].second;
			// get observations
			cv::KeyPoint &kp2 = vKeys2[currentIdx2];
			cv::Vec3d &ray2 = vKeysRays2[currentIdx2];
			int camIdx2 = vDistCamIndex[id];

			// get corresponding essential matrix between 2 cameras from 2 multikeyframes
			if (CheckDistEpipolarLine(ray1, ray2, Es[camIdx1][camIdx2], 1e-2))
			{
				vbMatched2[currentIdx2] = true;
				vMatches12[idx1] = currentIdx2;
				nmatches++;

				if (mbCheckOrientation)
				{
					float rot = kp1.angle - kp2.angle;
					if (rot < 0.0)
						rot += 360.0;
					int bin = cvRound(rot*factor);
					if (bin == HISTO_LENGTH)
						bin = 0;
					rotHist[bin].push_back(idx1);
				}
				break;
			}
		}
	}

	if (mbCheckOrientation)
	{
		int ind1 = -1;
		int ind2 = -1;
		int ind3 = -1;

		ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

		for (int i = 0; i<HISTO_LENGTH; i++)
		{
			if (i == ind1 || i == ind2 || i == ind3)
				continue;
			for (size_t j = 0, jend = rotHist[i].size(); j<jend; j++)
			{
				vMatches12[rotHist[i][j]] = -1;
				--nmatches;
			}
		}

	}

	vMatchedKeys1.clear();
	vMatchedKeys1.reserve(nmatches);
	vMatchedKeysRays1.clear();
	vMatchedKeysRays1.reserve(nmatches);
	vMatchedKeys2.clear();
	vMatchedKeys2.reserve(nmatches);
	vMatchedKeysRays2.clear();
	vMatchedKeysRays2.reserve(nmatches);
	vMatchedPairs.clear();
	vMatchedPairs.reserve(nmatches);

	for (size_t i = 0, iend = vMatches12.size(); i<iend; i++)
	{
		if (vMatches12[i] < 0)
			continue;

		vMatchedKeys1.push_back(vKeys1[i]);
		vMatchedKeys2.push_back(vKeys2[vMatches12[i]]);

		vMatchedKeysRays1.push_back(vKeysRays1[i]);
		vMatchedKeysRays2.push_back(vKeysRays2[vMatches12[i]]);

		vMatchedPairs.push_back(make_pair(i, vMatches12[i]));
	}

	return nmatches;
}


int cORBmatcher::SearchForTriangulationBetweenCameras(cMultiKeyFrame *pKF1,
	const int cam1, const int cam2,
	std::vector<cv::KeyPoint> &vMatchedKeys1,
	std::vector<cv::Vec3d> &vMatchedKeysRays1,
	std::vector<cv::KeyPoint> &vMatchedKeys2,
	std::vector<cv::Vec3d> &vMatchedKeysRays2,
	std::vector<std::pair<size_t, size_t> > &vMatchedPairs)
{
	// precompute essential matrices
	int nrCams = pKF1->camSystem.GetNrCams();

	cv::Matx44d Mcs1 = pKF1->camSystem.Get_M_c(cam1);
	cv::Matx44d Mcs2 = pKF1->camSystem.Get_M_c(cam2);
	cv::Matx44d RelOri = cConverter::invMat(Mcs1)*Mcs2;
	cv::Matx33d E12 = ComputeE(RelOri);
	cv::Matx33d Rrel = cConverter::Hom2R(RelOri).t();
	cv::Vec3d trel = -Rrel*cConverter::Hom2T(RelOri);

	cCamModelGeneral_ camModel2 = pKF1->camSystem.GetCamModelObj(cam2);

	vector<cMapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
	vector<cv::KeyPoint> vKeys1 = pKF1->GetKeyPoints();
	vector<cv::Vec3d> vKeysRays1 = pKF1->GetKeyPointsRays();
	int nmatches = 0;
	vector<int> vMatches12(vKeys1.size(), -1);
	for (size_t idx1 = 0; idx1 < vpMapPoints1.size(); ++idx1)
	{
		// if we have a map point already, continue
		if (vpMapPoints1[idx1])
			continue;

		// get observation from first camera
		const cv::KeyPoint &kp1 = vKeys1[idx1];
		const cv::Vec3d &Xl1 = vKeysRays1[idx1];

		int camIdx1 = pKF1->keypoint_to_cam.find(idx1)->second;
		// test if we have the correct camera
		if (camIdx1 != cam1)
			continue;

		// project and search in vicinity
		cv::Vec3d rotPoint = Rrel * Xl1 + trel;
		rotPoint /= norm(rotPoint);
		cv::Vec2d uv;
		camModel2.WorldToImg(rotPoint, uv);

		if (!camModel2.isPointInMirrorMask(uv(0), uv(1), 0))
			continue;

		vector<size_t> vIndices =
			pKF1->GetFeaturesInArea(cam2, uv(0), uv(1), 40);
		if (vIndices.empty())
			continue;
		// get descriptor from cam 1
		int descIdx = pKF1->cont_idx_to_local_cam_idx.find(idx1)->second;

		const uint64_t* descMP = pKF1->GetDescriptorRowPtr(cam1, descIdx);
		const uint64_t* descMP_mask = 0;
		if (havingMasks)
			descMP_mask = pKF1->GetDescriptorMaskRowPtr(cam1, descIdx);

		// match to descriptors in area in second camera
		int bestDist = INT_MAX;
		int bestIdx2 = -1;
		// match the descriptor of the current image point to all points in the area
		for (vector<size_t>::iterator vit = vIndices.begin(), vend = vIndices.end();
			vit != vend; ++vit)
		{
			size_t i2 = *vit;
			int idxDescCurr = pKF1->cont_idx_to_local_cam_idx.find(i2)->second;
			const uint64_t* d = pKF1->GetDescriptorRowPtr(cam2, idxDescCurr);

			int dist = 0;
			if (havingMasks)
			{
				const uint64_t* d_mask = pKF1->GetDescriptorMaskRowPtr(cam2, idxDescCurr);
				dist = DescriptorDistance64Masked(descMP, d, descMP_mask, d_mask, mbFeatDim);
			}
			else 
				dist = DescriptorDistance64(descMP, d,mbFeatDim);

			if (dist < bestDist)
			{
				bestDist = dist;
				bestIdx2 = i2;
			}
		}
		// epipolar check
		cv::Vec3d Xl2 = pKF1->GetKeyPointRay(bestIdx2);
		bool epiDist = CheckDistEpipolarLine(Xl1, Xl2, E12, 1e-2);
		if (bestDist <= 100 && epiDist)
		{
			vMatches12[idx1] = bestIdx2;
			vMatchedKeys1.push_back(vKeys1[idx1]);
			vMatchedKeys2.push_back(vKeys1[bestIdx2]);

			vMatchedKeysRays1.push_back(vKeysRays1[idx1]);
			vMatchedKeysRays2.push_back(vKeysRays1[bestIdx2]);

			vMatchedPairs.push_back(make_pair(idx1, bestIdx2));
			++nmatches;
		}
	}

	return nmatches;
}

int cORBmatcher::Fuse(cMultiKeyFrame *pKF,
	cMultiKeyFrame *curKF,
	vector<cMapPoint *> &vpMapPoints, 
	double th)
{
	cMultiCamSys_& camSys = pKF->camSystem;

    const int nMaxLevel = pKF->GetScaleLevels()-1;
    vector<double> vfScaleFactors = pKF->GetScaleFactors();

    cv::Vec3d Ow = pKF->GetCameraCenter();

    int nFused=0;

	for (size_t i = 0; i < vpMapPoints.size(); ++i)
	{
		cMapPoint* pMP = vpMapPoints[i];

		if (!pMP)
			continue;

		if (pMP->isBad() || pMP->IsInKeyFrame(pKF))
			continue;

		cv::Vec3d p3Dw = pMP->GetWorldPos();
		cv::Vec4d pt4 = cConverter::toVec4d(p3Dw);
		vector<int> bestIdxs;
		vector<int> cam2bestIdxs;
		for (int cam = 0; cam < camSys.GetNrCams(); ++cam)
		{
			cv::Vec2d uv(0.0, 0.0);
			
			camSys.WorldToCamHom_fast(cam, pt4, uv);

			// Point must be inside the image
			if (!camSys.GetCamModelObj(cam).isPointInMirrorMask(uv(0), uv(1), 0))
				continue;

			const double maxDistance = pMP->GetMaxDistanceInvariance();
			const double minDistance = pMP->GetMinDistanceInvariance();
			cv::Vec3d PO = p3Dw - Ow;
			const float dist3D = cv::norm(PO);

			// Depth must be inside the scale pyramid of the image
			if (dist3D < minDistance || dist3D > maxDistance)
				continue;

			// Viewing angle must be less than 60 deg
			cv::Vec3d Pn = pMP->GetNormal();

			//if (PO.dot(Pn) < 0.5*dist3D)
			//	continue;

			// Compute predicted scale level
			const double ratio = dist3D / minDistance;

			vector<double>::iterator it =
				std::lower_bound(vfScaleFactors.begin(), vfScaleFactors.end(), ratio);
			const int nPredictedLevel =
				std::min(static_cast<int>(it - vfScaleFactors.begin()), nMaxLevel);

			// Search in a radius
			const double radius = th * vfScaleFactors[nPredictedLevel];

			vector<size_t> vIndices = pKF->GetFeaturesInArea(cam, uv(0), uv(1), radius);

			if (vIndices.empty())
				continue;

			// Match to the most similar keypoint in the radius
			const uint64_t* dMP = pMP->GetDescriptorPtr();
			const uint64_t* dMP_mask = 0;
			if (havingMasks)
				dMP_mask = pMP->GetDescriptorMaskPtr();

			int bestDist = INT_MAX;
			int bestIdx = -1;
			for (vector<size_t>::iterator vit = vIndices.begin(), vend = vIndices.end();
				vit != vend; ++vit)
			{
				const size_t idx = *vit;
				const int kpLevel = pKF->GetKeyPointScaleLevel(idx);

				if (kpLevel<nPredictedLevel - 1 || kpLevel>nPredictedLevel)
					continue;

				int descIdx = pKF->cont_idx_to_local_cam_idx.find(idx)->second;
				const uint64_t* dKF = pKF->GetDescriptorRowPtr(cam, descIdx);

				int dist = 0;
				if (havingMasks)
				{
					const uint64_t* dKF_mask = pKF->GetDescriptorMaskRowPtr(cam, descIdx);
					dist = DescriptorDistance64Masked(dMP, dKF, dMP_mask, dKF_mask, mbFeatDim);
				}
				else 
					dist = DescriptorDistance64(dMP, dKF, mbFeatDim);

				if (dist < bestDist)
				{
					bestDist = dist;
					bestIdx = idx;
				}
			}
			// make it stronger, also the epipolar constrain has to hold
			if (bestIdx < 0)
				continue;
			if (bestDist <= TH_LOW_)
			{
				bestIdxs.push_back(bestIdx);
				cam2bestIdxs.push_back(cam);
			}

		} // over cams
		cMapPoint* pMPinKF = NULL;
		if (bestIdxs.size() > 0)
		{
			// add more than one observation
			for (int f = 0; f < bestIdxs.size(); ++f)
			{
				pMPinKF = pKF->GetMapPoint(bestIdxs[f]);
				// if there is a map point assigned to this postion 
				// fuse it 
				if (pMPinKF)
				{
					cv::Vec3d ray1 = curKF->GetKeyPointRay(i);
					cv::Vec3d ray2 = pKF->GetKeyPointRay(bestIdxs[f]);
					int camIdx1 = cam2bestIdxs[f]; // for current KF
					int camIdx2 = pKF->keypoint_to_cam.find(bestIdxs[f])->second;
					cv::Matx44d T1 = curKF->camSystem.Get_MtMc_inv(camIdx1);
					cv::Matx44d T2 = pKF->camSystem.Get_MtMc(camIdx1);
					cv::Matx33d E12 = ComputeE(T1*T2);

					if (!pMPinKF->isBad() && CheckDistEpipolarLine(ray1, ray2, E12, 1e-2))
					{
						//if (pMPinKF->TotalNrObservations() > pMP->TotalNrObservations())
							pMP->Replace(pMPinKF);
						//else
						//	pMPinKF->Replace(pMP);
						++nFused;
					}
				}
				else
				{
					pMP->AddObservation(pKF, bestIdxs[f]);
					pKF->AddMapPoint(pMP, bestIdxs[f]);
				}
			}

			
		}
	}
    return nFused;
}

int cORBmatcher::Fuse(cMultiKeyFrame* pKF,
	std::vector<cMapPoint*> &vpMapPoints,
	double th)
{
	cMultiCamSys_& camSys = pKF->camSystem;

	const int nMaxLevel = pKF->GetScaleLevels() - 1;
	vector<double> vfScaleFactors = pKF->GetScaleFactors();

	cv::Vec3d Ow = pKF->GetCameraCenter();

	int nFused = 0;

	for (size_t i = 0; i < vpMapPoints.size(); ++i)
	{
		cMapPoint* pMP = vpMapPoints[i];
		if (!pMP)
			continue;
		// if this point is already in the target keyframe
		if (pMP->isBad() || pMP->IsInKeyFrame(pKF))
			continue;
		// Match to the most similar keypoint in the radius
		const uint64_t* dMP = pMP->GetDescriptorPtr();
		const uint64_t* dMP_mask = 0;
		if (havingMasks)
			dMP_mask = pMP->GetDescriptorMaskPtr();

		cv::Vec3d p3Dw = pMP->GetWorldPos();
		cv::Vec4d pt4 = cConverter::toVec4d(p3Dw);
		vector<int> bestIdxs;
		// project the point into each camera
		for (int cam = 0; cam < camSys.GetNrCams(); ++cam)
		{
			cv::Vec2d uv(0.0, 0.0);
			camSys.WorldToCamHom_fast(cam, pt4, uv);

			// Point must be inside the image
			if (!camSys.GetCamModelObj(cam).isPointInMirrorMask(uv(0), uv(1), 0))
				continue;

			const double maxDistance = pMP->GetMaxDistanceInvariance();
			const double minDistance = pMP->GetMinDistanceInvariance();

			cv::Vec3d PO = p3Dw - Ow;
			const float dist3D = cv::norm(PO);

			// Depth must be inside the scale pyramid of the image
			if (dist3D < minDistance || dist3D > maxDistance)
				continue;

			// Viewing angle must be less than 60 deg
			cv::Vec3d Pn = pMP->GetNormal();

			//if (PO.dot(Pn) < 0.4*dist3D)
			//	continue;

			// Compute predicted scale level
			const double ratio = dist3D / minDistance;

			vector<double>::iterator it =
				std::lower_bound(vfScaleFactors.begin(), vfScaleFactors.end(), ratio);
			const int nPredictedLevel =
				std::min(static_cast<int>(it - vfScaleFactors.begin()), nMaxLevel);

			// Search in a radius
			const double radius = th * vfScaleFactors[nPredictedLevel];

			// get all indices from specific cam in a radius around the projection
			vector<size_t> vIndices = pKF->GetFeaturesInArea(cam, uv(0), uv(1), radius);

			if (vIndices.empty())
				continue;

			int bestDist = INT_MAX;
			int bestIdx = -1;
			for (vector<size_t>::iterator vit = vIndices.begin(), vend = vIndices.end();
				vit != vend; ++vit)
			{
				const size_t idx = *vit;
				const int kpLevel = pKF->GetKeyPointScaleLevel(idx);

				if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
					continue;

				int descIdx = pKF->cont_idx_to_local_cam_idx.find(idx)->second;

				const uint64_t* dKF = pKF->GetDescriptorRowPtr(cam, descIdx);			
				int dist = 0;
				if (havingMasks)
				{
					const uint64_t* dKF_mask = pKF->GetDescriptorMaskRowPtr(cam, descIdx);
					DescriptorDistance64Masked(dMP, dKF, dMP_mask, dKF_mask, mbFeatDim);
				}
				else
					DescriptorDistance64(dMP, dKF, mbFeatDim);

				if (dist < bestDist)
				{
					bestDist = dist;
					bestIdx = idx;
				}
			}
			
			if (bestIdx < 0)
				continue;
			// make it stronger, also the epipolar constrain has to hold
			//cv::Vec3d ray1 = curKF->GetKeyPointRay(i);
			//cv::Vec3d ray2 = pKF->GetKeyPointRay(bestIdx);
			//int camIdx1 = curKF->keypoint_to_cam.find(i)->second;
			//cv::Matx44d T1 = curKF->camSystem.Get_MtMc_inv(camIdx1);
			//cv::Matx44d T2 = pKF->camSystem.Get_MtMc(camIdx1);
			//cv::Matx33d E12 = ComputeE(T1*T2);

			if (bestDist <= TH_LOW_)
				bestIdxs.push_back(bestIdx);
		}
		cMapPoint* pMPinKF = NULL;
		if (bestIdxs.size() > 0)
		{
			// add more than one observation
			for (int f = 0; f < bestIdxs.size(); ++f)
			{
				
				pMPinKF = pKF->GetMapPoint(bestIdxs[f]);
				// if there is a map point assigned to this postion 
				// fuse it 
				if (pMPinKF)
				{
					if (!pMPinKF->isBad())
					{
						//if (pMPinKF->TotalNrObservations() > pMP->TotalNrObservations())
							pMP->Replace(pMPinKF);
						//else
						//	pMPinKF->Replace(pMP);
						++nFused;
					}
				}
				else
				{
					pMP->AddObservation(pKF, bestIdxs[f]);
					pKF->AddMapPoint(pMP, bestIdxs[f]);
				}
			}
		}
	}

	return nFused;
}


int cORBmatcher::Fuse(cMultiKeyFrame *pKF, cv::Matx44d Scw,
	const vector<cMapPoint *> &vpPoints, double th)
{
	cMultiCamSys_ camSys = pKF->camSystem;

	// Decompose Scw
	cv::Matx33d sRcw = cConverter::Hom2R(Scw);
	cv::Vec3d row1(sRcw(0, 0), sRcw(0, 1), sRcw(0, 2));
	// retrieve scale
	const double inv_scw = 1.0 / cv::sqrt(row1.dot(row1));
	// rotation without scale
	cv::Matx33d Rcw = inv_scw * sRcw;
	// translation without scale
	cv::Vec3d tcw = inv_scw * cConverter::Hom2T(Scw);
	camSys.Set_M_t(cConverter::invMat(cConverter::Rt2Hom(Rcw, tcw)));
	cv::Vec3d Ow = cConverter::Hom2T(camSys.Get_M_t());

    // Set of MapPoints already found in the KeyFrame
    set<cMapPoint*> spAlreadyFound = pKF->GetMapPoints();

    const int nMaxLevel = pKF->GetScaleLevels()-1;
    vector<double> vfScaleFactors = pKF->GetScaleFactors();

    int nFused=0;

    // For each candidate MapPoint project and match
	for (size_t iMP = 0, iendMP = vpPoints.size(); iMP<iendMP; ++iMP)
    {
        cMapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
		cv::Vec3d p3Dw = pMP->GetWorldPos();
		cv::Vec4d pt4 = cConverter::toVec4d(p3Dw);

		vector<int> bestIdxs;

		for (int cam = 0; cam < camSys.GetNrCams(); ++cam)
		{
			cv::Vec2d uv(0.0, 0.0);
			camSys.WorldToCamHom_fast(cam, pt4, uv);

			// Point must be inside the image
			if (!camSys.GetCamModelObj(cam).isPointInMirrorMask(uv(0), uv(1), 0))
				continue;

			// Depth must be inside the scale pyramid of the image
			const double maxDistance = pMP->GetMaxDistanceInvariance();
			const double minDistance = pMP->GetMinDistanceInvariance();
			cv::Vec3d PO = p3Dw - Ow;
			const double dist3D = cv::norm(PO);

			if (dist3D<minDistance || dist3D>maxDistance)
				continue;

			// Viewing angle must be less than 60 deg
			cv::Vec3d Pn = pMP->GetNormal();

			//if (PO.dot(Pn) < 0.5*dist3D)
			//	continue;

			// Compute predicted scale level
			const double ratio = dist3D / minDistance;

			vector<double>::iterator it =
				lower_bound(vfScaleFactors.begin(), vfScaleFactors.end(), ratio);
			const int nPredictedLevel =
				min(static_cast<int>(it - vfScaleFactors.begin()), nMaxLevel);

			// Search in a radius of 2.5*sigma(ScaleLevel)
			const double radius = th*pKF->GetScaleFactor(nPredictedLevel);

			vector<size_t> vIndices = pKF->GetFeaturesInArea(cam, uv(0), uv(1), radius);

			if (vIndices.empty())
				continue;

			// Match to the most similar keypoint in the radius
			const uint64_t* dMP = pMP->GetDescriptorPtr();
			const uint64_t* dMP_mask = 0;
			if (havingMasks) 
				dMP_mask = pMP->GetDescriptorMaskPtr();

			int bestDist = INT_MAX;
			int bestIdx = -1;
			for (vector<size_t>::iterator vit = vIndices.begin();
				vit != vIndices.end(); ++vit)
			{
				const size_t idx = *vit;
				const int kpLevel = pKF->GetKeyPointScaleLevel(idx);

				if (kpLevel<nPredictedLevel - 1 || kpLevel>nPredictedLevel)
					continue;

				int descIdx = pKF->cont_idx_to_local_cam_idx.find(idx)->second;
				const uint64_t* dKF = pKF->GetDescriptorRowPtr(cam, descIdx);

				int dist = 0;
				if (havingMasks)
				{
					const uint64_t* dKF_mask = pKF->GetDescriptorMaskRowPtr(cam, descIdx);
					dist = DescriptorDistance64Masked(dMP, dKF, dMP_mask, dKF_mask, mbFeatDim);
				}
				else 
					dist = DescriptorDistance64(dMP, dKF, mbFeatDim);

				if (dist < bestDist)
				{
					bestDist = dist;
					bestIdx = idx;
				}
			}
			if (bestIdx < 0)
				continue;

			if (bestDist <= TH_LOW_)
				bestIdxs.push_back(bestIdx);
		}
		// fuse over multiple cameras in one MKF
		cMapPoint* pMPinKF = NULL;
		if (bestIdxs.size() > 0)
		{
			// add more than one observation
			for (int f = 0; f < bestIdxs.size(); ++f)
			{

				pMPinKF = pKF->GetMapPoint(bestIdxs[f]);
				// if there is a map point assigned to this position -> fuse it 
				if (pMPinKF)
				{
					if (!pMPinKF->isBad())
					{
						pMP->Replace(pMPinKF);
						++nFused;
					}
				}
				else
				{
					pMP->AddObservation(pKF, bestIdxs[f]);
					pKF->AddMapPoint(pMP, bestIdxs[f]);
				}
			}
		}
    }

    return nFused;
}

int cORBmatcher::SearchBySim3(cMultiKeyFrame *pKF1,
	cMultiKeyFrame *pKF2,
	vector<cMapPoint*> &vpMatches12,
    const double &s12, 
	const cv::Matx33d &R12, 
	const cv::Vec3d &t12, 
	double th)
{
	cv::Matx44d T1 = pKF1->GetPoseInverse();
	cv::Matx44d T2 = pKF2->GetPoseInverse();

	// Camera 1 from world
	cv::Matx33d R1w = cConverter::Hom2R(T1);
	cv::Vec3d t1w = cConverter::Hom2T(T1);

	//Camera 2 from world
	cv::Matx33d R2w = cConverter::Hom2R(T2);
	cv::Vec3d t2w = cConverter::Hom2T(T2);

    //Transformation between cameras
    cv::Matx33d sR12 = s12 * R12;
    cv::Matx33d sR21 = (1.0/s12) * R12.t();
    cv::Vec3d t21    = -sR21 * t12;

    const int nMaxLevel1 = pKF1->GetScaleLevels()-1;
    vector<double> vfScaleFactors1 = pKF1->GetScaleFactors();

    vector<cMapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();

    const int nMaxLevel2 = pKF2->GetScaleLevels()-1;
	vector<double> vfScaleFactors2 = pKF2->GetScaleFactors();

    vector<cMapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();  
	// TODO:  need total nr of observations, cause multiple cameras can observe a map point
    const int N2 = vpMapPoints2.size(); 

    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

	for (int i = 0; i < N1; ++i)
    {
        cMapPoint* pMP = vpMatches12[i];
        if (pMP)
        {
            vbAlreadyMatched1[i]=true;
			int idx2 = pMP->GetIndexInKeyFrame(pKF2)[0]; // do it for more cams!
			if (idx2 >= 0 && idx2 < N2)
				vbAlreadyMatched2[idx2] = true;
			//for (auto & idxs : pMP->GetIndexInKeyFrame(pKF2))
			//	if (idxs >= 0 && idxs < N2)
			//		vbAlreadyMatched2[idxs] = true;
        }
    }

    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);
    // Transform from KF1 to KF2 and search
	for (int i1 = 0; i1 < N1; ++i1)
	{

		cMapPoint* pMP = vpMapPoints1[i1];

		if (!pMP || vbAlreadyMatched1[i1])
			continue;

		if (pMP->isBad())
			continue;

		int camIdx1 = pKF1->keypoint_to_cam.find(i1)->second;
		cCamModelGeneral_ camModel1 = pKF2->camSystem.GetCamModelObj(camIdx1);
		cv::Vec3d p3Dw = pMP->GetWorldPos();
		cv::Vec3d p3Dc1 = R1w*p3Dw + t1w; // point to MCS frame
		cv::Vec3d p3Dc2 = sR21*p3Dc1 + t21; // point from MCS frame 1 to MCS frame 2
		cv::Vec4d p4Dc2 = cConverter::invMat(pKF2->camSystem.Get_M_c(camIdx1)) * cConverter::toVec4d(p3Dc2);

		// Depth must be positive
		if (p3Dc2(2) < 0.0)
			continue;

		double u = 0.0;
		double v = 0.0;

		camModel1.WorldToImg(p4Dc2(0), p4Dc2(1), p4Dc2(2), u, v);

		// Point must be inside the image
		if (!camModel1.isPointInMirrorMask(u, v, 0))
			continue;

		double maxDistance = pMP->GetMaxDistanceInvariance();
		double minDistance = pMP->GetMinDistanceInvariance();
		double dist3D = cv::norm(cv::Vec3d(p4Dc2(0), p4Dc2(1), p4Dc2(2)));

		// Depth must be inside the scale invariance region
		if (dist3D < minDistance || dist3D > maxDistance)
			continue;

		// Compute predicted octave
		double ratio = dist3D / minDistance;

		vector<double>::iterator it = lower_bound(vfScaleFactors2.begin(), vfScaleFactors2.end(), ratio);
		const int nPredictedLevel = min(static_cast<int>(it - vfScaleFactors2.begin()), nMaxLevel2);
		// Search in a radius
		double radius = th*vfScaleFactors2[nPredictedLevel];

		vector<size_t> vIndices = pKF2->GetFeaturesInArea(camIdx1, u, v, radius);

		if (vIndices.empty())
			continue;

		// Match to the most similar keypoint in the radius
		const uint64_t* dMP = pMP->GetDescriptorPtr();
		const uint64_t* dMP_mask = 0;
		if (havingMasks)
			dMP_mask = pMP->GetDescriptorMaskPtr();

		int bestDist = INT_MAX;
		int bestIdx = -1;
		for (vector<size_t>::iterator vit =
			vIndices.begin(), vend = vIndices.end(); vit != vend; ++vit)
		{
			size_t idx = *vit;

			cv::KeyPoint kp = pKF2->GetKeyPoint(idx);

			if (kp.octave<nPredictedLevel - 1 || kp.octave>nPredictedLevel)
				continue;

			int descIdx = pKF2->cont_idx_to_local_cam_idx.find(idx)->second;

			const uint64_t* dKF = pKF2->GetDescriptorRowPtr(camIdx1, descIdx);
			int dist = 0;
			if (havingMasks)
			{
				const uint64_t* dKF_mask = pKF2->GetDescriptorMaskRowPtr(camIdx1, descIdx);
				dist = DescriptorDistance64Masked(dMP, dKF, dMP_mask, dKF_mask, mbFeatDim);

			}
			else
				dist = DescriptorDistance64(dMP, dKF, mbFeatDim);

			if (dist < bestDist)
			{
				bestDist = dist;
				bestIdx = idx;
			}

		}

		if (bestDist <= TH_HIGH_)
			vnMatch1[i1] = bestIdx;
	}
    // Transform from KF2 to KF1 and search
	for (int i2 = 0; i2 < N2; ++i2)
	{
		cMapPoint* pMP = vpMapPoints2[i2];

		if (!pMP || vbAlreadyMatched2[i2])
			continue;

		if (pMP->isBad())
			continue;

		int camIdx2 = pKF2->keypoint_to_cam.find(i2)->second;
		cCamModelGeneral_ camModel2 = pKF2->camSystem.GetCamModelObj(camIdx2);
		cv::Vec3d p3Dw = pMP->GetWorldPos();
		cv::Vec3d p3Dc2 = R2w*p3Dw + t2w;
		cv::Vec3d p3Dc1 = sR12*p3Dc2 + t12;
		cv::Vec4d p4Dc1 = cConverter::invMat(pKF1->camSystem.Get_M_c(camIdx2)) * cConverter::toVec4d(p3Dc1);

		// Depth must be positive
		if (p3Dc1(2) < 0.0)
			continue;

		double u = 0.0;
		double v = 0.0;
		camModel2.WorldToImg(p4Dc1(0), p4Dc1(1), p4Dc1(2), u, v);

		// Point must be inside the image
		if (!camModel2.isPointInMirrorMask(u, v, 0))
			continue;

		double maxDistance = pMP->GetMaxDistanceInvariance();
		double minDistance = pMP->GetMinDistanceInvariance();
		double dist3D = cv::norm(cv::Vec3d(p4Dc1(0), p4Dc1(1), p4Dc1(2)));

		// Depth must be inside the scale pyramid of the image
		if (dist3D<minDistance || dist3D>maxDistance)
			continue;

		// Compute predicted octave
		double ratio = dist3D / minDistance;

		vector<double>::iterator it = lower_bound(vfScaleFactors1.begin(), vfScaleFactors1.end(), ratio);
		const int nPredictedLevel = min(static_cast<int>(it - vfScaleFactors1.begin()), nMaxLevel1);

		// Search in a radius of 2.5*sigma(ScaleLevel)
		double radius = th*vfScaleFactors1[nPredictedLevel];

		vector<size_t> vIndices = pKF1->GetFeaturesInArea(camIdx2, u, v, radius);

		if (vIndices.empty())
			continue;

		// Match to the most similar keypoint in the radius
		const uint64_t* dMP = pMP->GetDescriptorPtr();
		const uint64_t* dMP_mask = 0;
		if (havingMasks)
			dMP_mask = pMP->GetDescriptorMaskPtr();

		int bestDist = INT_MAX;
		int bestIdx = -1;

		for (vector<size_t>::iterator vit = vIndices.begin(), vend = vIndices.end();
			vit != vend; ++vit)
		{
			size_t idx = *vit;

			cv::KeyPoint kp = pKF1->GetKeyPoint(idx);

			if (kp.octave<nPredictedLevel - 1 || kp.octave>nPredictedLevel)
				continue;

			int descIdx = pKF1->cont_idx_to_local_cam_idx.find(idx)->second;
			const uint64_t*  dKF = pKF1->GetDescriptorRowPtr(camIdx2, descIdx);

			int dist = 0;
			if (havingMasks)
			{
				const uint64_t*  dKF_mask = pKF1->GetDescriptorMaskRowPtr(camIdx2, descIdx);
				dist = DescriptorDistance64Masked(dMP, dKF, dMP_mask, dKF_mask, mbFeatDim);
			}
			else dist = DescriptorDistance64(dMP, dKF, mbFeatDim);


			if (dist < bestDist)
			{
				bestDist = dist;
				bestIdx = idx;
			}
		}

		if (bestDist <= TH_HIGH_)
		{
			vnMatch2[i2] = bestIdx;
		}
	}

    // Check agreement
    int nFound = 0;

	for (int i1 = 0; i1<N1; ++i1)
    {
        int idx2 = vnMatch1[i1];

        if (idx2 >= 0)
        {
            int idx1 = vnMatch2[idx2];
            if (idx1 == i1)
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}

int cORBmatcher::SearchByProjection(cMultiFrame &CurrentFrame,
	const cMultiFrame &LastFrame, double th)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

	cMultiCamSys_& camSys = CurrentFrame.camSystem;

	for (size_t i = 0, iend = LastFrame.mvpMapPoints.size(); i < iend; ++i)
	{
		cMapPoint* pMP = LastFrame.mvpMapPoints[i];

		if (!pMP)
			continue;
		if (pMP->isBad())
			continue;

		int cam = LastFrame.keypoint_to_cam.find(i)->second;
		// if this point was not classified to be an outlier
		if (!LastFrame.mvbOutlier[i])
		{
			// Project to current camera pose
			cv::Vec3d x3Dw = pMP->GetWorldPos();
			cv::Vec4d pt4 = cConverter::toVec4d(x3Dw);
			cv::Vec2d uv(0.0, 0.0);
			camSys.WorldToCamHom_fast(cam,pt4, uv);
			// if it is not in the mirror mask break
			if (!camSys.GetCamModelObj(cam).isPointInMirrorMask(uv(0), uv(1), 0))
				continue;

			int nPredictedOctave = LastFrame.mvKeys[i].octave;

			// Search in a window. Size depends on scale
			double radius = th*CurrentFrame.mvScaleFactors[nPredictedOctave];

			vector<size_t> vIndices2 =
				CurrentFrame.GetFeaturesInArea(cam, uv(0), uv(1), radius,
				nPredictedOctave - 1, nPredictedOctave + 1);
			//vector<size_t> vIndices2 =
			//	CurrentFrame.GetFeaturesInArea(cam, uv(0), uv(1), radius);
			// if there are no features in this image area
			if (vIndices2.empty())
				continue;
			// get descriptors (and learned masks)
			int idxDescLast = LastFrame.cont_idx_to_local_cam_idx.find(i)->second;
			const uint64_t* dMP = LastFrame.mDescriptors[cam].ptr<uint64_t>(idxDescLast);
			const uint64_t* dMP_mask = 0;
			if (havingMasks)
				dMP_mask = LastFrame.mDescriptorMasks[cam].ptr<uint64_t>(idxDescLast);

			int bestDist = INT_MAX;
			int bestIdx2 = -1;
			// match
			for (vector<size_t>::iterator vit = vIndices2.begin(), vend = vIndices2.end();
				vit != vend; ++vit)
			{
				size_t i2 = *vit;
				if (CurrentFrame.mvpMapPoints[i2])
					continue;
				int idxDescCurr = CurrentFrame.cont_idx_to_local_cam_idx.find(i2)->second;
				const uint64_t* d = CurrentFrame.mDescriptors[cam].ptr<uint64_t>(idxDescCurr);

				int dist = 0;
				if (havingMasks)
				{
					const uint64_t* d_mask = CurrentFrame.mDescriptorMasks[cam].ptr<uint64_t>(idxDescCurr);
					dist = DescriptorDistance64Masked(dMP, d, dMP_mask, d_mask, mbFeatDim);
				}
				else
					dist = DescriptorDistance64(dMP, d, mbFeatDim);

				if (dist < bestDist)
				{
					bestDist = dist;
					bestIdx2 = i2;
				}
			}

			if (bestDist <= TH_HIGH_)
			{
				CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
				++nmatches;

				if (mbCheckOrientation)
				{
					float rot = 
						LastFrame.mvKeys[i].angle - CurrentFrame.mvKeys[bestIdx2].angle;
					if (rot < 0.0)
						rot += 360.0f;
					int bin = cvRound(rot*factor);
					if (bin == HISTO_LENGTH)
						bin = 0;
					rotHist[bin].push_back(bestIdx2);
				}
			}

		}

	}

   // Apply rotation consistency
   if(mbCheckOrientation)
   {
       int ind1=-1;
       int ind2=-1;
       int ind3=-1;

       ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

       for(int i=0; i<HISTO_LENGTH; i++)
       {
           if(i!=ind1 && i!=ind2 && i!=ind3)
           {
               for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
               {
                   CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
				   --nmatches;
               }
           }
       }
   }

   return nmatches;
}

int cORBmatcher::SearchByProjection(cMultiFrame &CurrentFrame,
	cMultiKeyFrame *pKF,
	const set<cMapPoint*> &sAlreadyFound, 
	double th, int ORBdist)
{
	cMultiCamSys_& camSys = CurrentFrame.camSystem;

    int nmatches = 0;
	cv::Matx44d Tcurr = CurrentFrame.GetPose();
	const cv::Matx33d Rcw = Tcurr.get_minor<3, 3>(0, 0);
	const cv::Vec3d tcw(Tcurr(0, 3),
		Tcurr(1, 3), Tcurr(2, 3));
    const cv::Vec3d Ow = -Rcw.t()*tcw;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<cMapPoint*> vpMPs = pKF->GetMapPointMatches();

    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        cMapPoint* pMP = vpMPs[i];

        if(pMP)
        {
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                //Project
				cv::Vec3d x3Dw = pMP->GetWorldPos();
				cv::Vec4d pt4 = cConverter::toVec4d(x3Dw);

				for (int cam = 0; cam < camSys.GetNrCams(); ++cam)
				{
					cv::Vec2d uv(0.0, 0.0);
					camSys.WorldToCamHom_fast(cam, pt4, uv);

					if (!camSys.GetCamModelObj(cam).isPointInMirrorMask(uv(0), uv(1), 0))
						continue;

					// Compute predicted scale level
					double minDistance = pMP->GetMinDistanceInvariance();
					cv::Vec3d PO = x3Dw - Ow;
					double dist3D = cv::norm(PO);
					double ratio = dist3D / minDistance;

					vector<double>::iterator it =
						lower_bound(CurrentFrame.mvScaleFactors.begin(), CurrentFrame.mvScaleFactors.end(), ratio);
					const int nPredictedLevel =
						min(static_cast<int>(it - CurrentFrame.mvScaleFactors.begin()), CurrentFrame.mnScaleLevels - 1);

					// Search in a window
					double radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];

					vector<size_t> vIndices2 =
						CurrentFrame.GetFeaturesInArea(cam, uv(0), uv(1),
						radius, nPredictedLevel - 1, nPredictedLevel + 1);
					//vector<size_t> vIndices2 =
					//	CurrentFrame.GetFeaturesInArea(cam, uv(0), uv(1),40);
					if (vIndices2.empty())
						continue;

					const uint64_t* dMP = pMP->GetDescriptorPtr();
					const uint64_t* dMP_mask = 0;
					if (havingMasks)
						dMP_mask = pMP->GetDescriptorMaskPtr();

					int bestDist = INT_MAX;
					int bestIdx2 = -1;

					for (vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
					{
						size_t i2 = *vit;
						if (CurrentFrame.mvpMapPoints[i2])
							continue;
						int idxDescCurr = pKF->cont_idx_to_local_cam_idx.find(i2)->second;
						const uint64_t* d = pKF->GetDescriptorRowPtr(cam, idxDescCurr);

						int dist = 0;
						if (havingMasks)
						{
							const uint64_t* d_mask = pKF->GetDescriptorMaskRowPtr(cam, idxDescCurr);
							dist = DescriptorDistance64Masked(dMP, d, dMP_mask, d_mask, mbFeatDim);
						}
						else
							dist = DescriptorDistance64(dMP, d, mbFeatDim);

						if (dist < bestDist)
						{
							bestDist = dist;
							bestIdx2 = i2;
						}
					}

					if (bestDist <= ORBdist)
					{
						CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
						++nmatches;

						if (mbCheckOrientation)
						{
							float rot = pKF->GetKeyPoint(i).angle - CurrentFrame.mvKeys[bestIdx2].angle;
							if (rot < 0.0)
								rot += 360.0f;
							int bin = cvRound(rot*factor);
							if (bin == HISTO_LENGTH)
								bin = 0;
							//ROS_ASSERT(bin>=0 && bin<HISTO_LENGTH);
							rotHist[bin].push_back(bestIdx2);
						}
					}
				}
            }
        }
    }


   if(mbCheckOrientation)
   {
       int ind1=-1;
       int ind2=-1;
       int ind3=-1;

       ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

	   for (int i = 0; i<HISTO_LENGTH; ++i)
       {
           if (i!=ind1 && i!=ind2 && i!=ind3)
           {
			   for (size_t j = 0, jend = rotHist[i].size(); j<jend; ++j)
               {
                   CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
				   --nmatches;
               }
           }
       }
   }

    return nmatches;
}


// for loop detection
int cORBmatcher::SearchByProjection(cMultiKeyFrame* pKF,
	cv::Matx44d Scw,
	const vector<cMapPoint*> &vpPoints,
	vector<cMapPoint*> &vpMatched,
	int th)
{
	cout << "In search by projection" << endl;
	// Get Calibration Parameters for later
	cMultiCamSys_ camSys = pKF->camSystem;

	const int nMaxLevel = pKF->GetScaleLevels() - 1;
	vector<double> vfScaleFactors = pKF->GetScaleFactors();

	// Decompose Scw
	cv::Matx33d sRcw = cConverter::Hom2R(Scw);
	cv::Vec3d row1(sRcw(0, 0), sRcw(0, 1), sRcw(0, 2));
	// retrieve scale
	const double inv_scw = 1.0 / cv::sqrt(row1.dot(row1));
	// rotation without scale
	cv::Matx33d Rcw = inv_scw * sRcw;
	// translation without scale
	cv::Vec3d tcw = inv_scw * cConverter::Hom2T(Scw);
	camSys.Set_M_t(cConverter::invMat(cConverter::Rt2Hom(Rcw, tcw)));
	cv::Vec3d Ow = cConverter::Hom2T(camSys.Get_M_t());

	// Set of MapPoints already found in the KeyFrame
	set<cMapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
	spAlreadyFound.erase(NULL);

	int nmatches = 0;
	// For each Candidate MapPoint Project and Match
	for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; ++iMP)
	{
		cMapPoint* pMP = vpPoints[iMP];

		if (!pMP)
			continue;

		// Discard Bad MapPoints and already found
		if (pMP->isBad() || spAlreadyFound.count(pMP))
			continue;

		int camIdx = pKF->keypoint_to_cam.find(iMP)->second;

		// Get 3D Coords.
		cv::Vec3d p3Dw = pMP->GetWorldPos();
		cv::Vec2d uv;
		camSys.WorldToCamHom_fast(camIdx, p3Dw, uv);
		// Point must be inside the image
		if (!camSys.GetCamModelObj(camIdx).isPointInMirrorMask(uv(0), uv(1), 0))
			continue;

		// Depth must be inside the scale invariance region of the point
		const double maxDistance = pMP->GetMaxDistanceInvariance();
		const double minDistance = pMP->GetMinDistanceInvariance();
		cv::Vec3d PO = p3Dw - Ow;
		const double dist = cv::norm(PO);

		if (dist < minDistance || dist > maxDistance)
			continue;

		// Viewing angle must be less than 60 deg
		cv::Vec3d Pn = pMP->GetNormal();
		//if (PO.dot(Pn) < 0.5*dist)
		//	continue;

		// Compute predicted scale level
		const double ratio = dist / minDistance;

		vector<double>::iterator it =
			std::lower_bound(vfScaleFactors.begin(), vfScaleFactors.end(), ratio);
		const int nPredictedLevel = min(static_cast<int>(it - vfScaleFactors.begin()), nMaxLevel);

		// Search in a radius
		const double radius = th*pKF->GetScaleFactor(nPredictedLevel);

		vector<size_t> vIndices = pKF->GetFeaturesInArea(camIdx, uv(0), uv(1), radius);

		if (vIndices.empty())
			continue;

		// Match to the most similar keypoint in the radius
		const uint64_t* dMP = pMP->GetDescriptorPtr();
		const uint64_t* dMP_mask = 0;
		if (havingMasks)
			dMP_mask = pMP->GetDescriptorMaskPtr();

		int bestDist = INT_MAX;
		int bestIdx = -1;

		for (vector<size_t>::iterator vit = vIndices.begin(), 
			vend = vIndices.end();vit != vend; ++vit)
		{
			size_t idx = *vit;
			if (vpMatched[idx])
				continue;

			const int kpLevel = pKF->GetKeyPointScaleLevel(idx);

			if (kpLevel<nPredictedLevel - 1 || kpLevel>nPredictedLevel)
				continue;

			const uint64_t* dKF = pKF->GetDescriptorRowPtr(camIdx, idx);

			int dist = 0;
			if (havingMasks)
			{
				const uint64_t* dKF_mask = pKF->GetDescriptorMaskRowPtr(camIdx, idx);
				dist = DescriptorDistance64Masked(dMP, dKF, dMP_mask, dKF_mask, mbFeatDim);
			}
			else 
				dist = DescriptorDistance64(dMP, dKF, mbFeatDim);
			if (dist < bestDist)
			{
				bestDist = dist;
				bestIdx = idx;
			}

		}

		if (bestDist <= TH_LOW_ && bestIdx > 0)
		{
			vpMatched[bestIdx] = pMP;
			++nmatches;
		}
	}
	return nmatches;
}

void cORBmatcher::ComputeThreeMaxima(vector<int>* histo, 
	const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2 < 0.1f*(float)max1)
    {
        ind2 = -1;
        ind3 = -1;
    }
    else if(max3 < 0.1f*(float)max1)
    {
        ind3 = -1;
    }
}

int DescriptorDistance64(const uint64_t* descr_i,
	const uint64_t* descr_j,
	const int& dim)
{
	uint64_t dist = 0;
	for (int d = 0; d < dim / 8; ++d)
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		dist += __popcnt64(descr_i[d] ^ descr_j[d]);
#else
		dist += __builtin_popcountll(descr_i[d] ^ descr_j[d]);
#endif
	return static_cast<int>(dist);
}

int DescriptorDistance64Masked(const uint64_t* descr_i,
		const uint64_t* descr_j,
		const uint64_t* mask_i,
		const uint64_t* mask_j,
		const int& dim)
{
	uint64_t dist = 0;
	for (int i = 0; i < dim / 8; ++i)
	{
		uint64_t axorb = descr_i[i] ^ descr_j[i];
		uint64_t xormaskedL = axorb & mask_i[i];
		uint64_t xormaskedR = axorb & mask_j[i];
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		dist += __popcnt64(xormaskedL);
		dist += __popcnt64(xormaskedR);
#else
		dist += __builtin_popcountll(xormaskedL);
		dist += __builtin_popcountll(xormaskedR);
#endif
	}
	int res = static_cast<int>(dist / 2);
	return res;
}

//int DescriptorDistance64Masked(const uint64_t* descr_i,
//	const uint64_t* descr_j,
//	const uint64_t* mask_i,
//	const uint64_t* mask_j,
//	const int& dim)
//{
//	uint64_t distL = 0;
//	uint64_t distR = 0;
//	double nL = 0.0;
//	double nR = 0.0;
//	for (int i = 0; i < dim / 8; ++i)
//	{
//		uint64_t axorb = descr_i[i] ^ descr_j[i];
//		uint64_t xormaskedL = axorb & mask_i[i];
//		uint64_t xormaskedR = axorb & mask_j[i];
//		nL += mask_i[i];
//		nR += mask_j[i];
//		distL += __popcnt64(xormaskedL);
//		distR += __popcnt64(xormaskedR);
//	}
//	double n = nL + nR;
//	double wL = nL / n;
//	double wR = nR / n;
//	//int res = static_cast<int>(dist / 2);
//	int res = static_cast<int>(distL*wL + distR*wR);
//	return res;
//}
}
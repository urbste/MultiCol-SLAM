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


/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iterator>

#include "mdBRIEFextractorOct.h"

#include "misc.h"


namespace MultiColSLAM
{

using namespace cv;
using namespace std;


const float HARRIS_K = 0.04f;
const float DEG2RADf = static_cast<float>(CV_PI) / 180.f;
const int PATCH_SIZE = 32;
const int HALF_PATCH_SIZE = 16;
const int EDGE_THRESHOLD = 25;

static void HarrisResponses(const Mat& img,
	const std::vector<Rect>& layerinfo,
	std::vector<KeyPoint>& pts,
	int blockSize,
	float harris_k)
{
	CV_Assert(img.type() == CV_8UC1 && blockSize*blockSize <= 2048);

	size_t ptidx, ptsize = pts.size();

	const uchar* ptr00 = img.ptr<uchar>();
	int step = (int)(img.step / img.elemSize1());
	int r = blockSize / 2;

	float scale = 1.f / ((1 << 2) * blockSize * 255.f);
	float scale_sq_sq = scale * scale * scale * scale;

	AutoBuffer<int> ofsbuf(blockSize*blockSize);
	int* ofs = ofsbuf;
	for (int i = 0; i < blockSize; i++)
		for (int j = 0; j < blockSize; j++)
			ofs[i*blockSize + j] = (int)(i*step + j);

	for (ptidx = 0; ptidx < ptsize; ptidx++)
	{
		int x0 = cvRound(pts[ptidx].pt.x);
		int y0 = cvRound(pts[ptidx].pt.y);
		int z = pts[ptidx].octave;

		const uchar* ptr0 = ptr00 + (y0 - r + layerinfo[z].y)*step + x0 - r + layerinfo[z].x;
		int a = 0, b = 0, c = 0;

		for (int k = 0; k < blockSize*blockSize; k++)
		{
			const uchar* ptr = ptr0 + ofs[k];
			int Ix = (ptr[1] - ptr[-1]) * 2 + (ptr[-step + 1] - ptr[-step - 1]) + (ptr[step + 1] - ptr[step - 1]);
			int Iy = (ptr[step] - ptr[-step]) * 2 + (ptr[step - 1] - ptr[-step - 1]) + (ptr[step + 1] - ptr[-step + 1]);
			a += Ix*Ix;
			b += Iy*Iy;
			c += Ix*Iy;
		}
		pts[ptidx].response = ((float)a * b - (float)c * c -
			harris_k * ((float)a + b) * ((float)a + b))*scale_sq_sq;
	}
}

mdBRIEFextractorOct::mdBRIEFextractorOct(int _nfeatures,
	float _scaleFactor,
	int _nlevels,
	int _edgeThreshold,
	int _firstLevel,
	int _scoreType,
	int _patchSize,
	int _fastThreshold,
	bool _useAgast,
	int _fastAgastType,
	bool _do_dBrief,
	bool _learnMasks,
	int _descSize) :
	nfeatures(_nfeatures), scaleFactor(_scaleFactor), numlevels(_nlevels),
	edgeThreshold(_edgeThreshold), firstLevel(_firstLevel),
	scoreType(_scoreType), patchSize(_patchSize), fastThreshold(_fastThreshold),
	useAgast(_useAgast), fastAgastType(_fastAgastType), learnMasks(_learnMasks),
	descSize(_descSize), do_dBrief(_do_dBrief)
{
	mvScaleFactor.resize(numlevels);
	mvScaleFactor[0] = 1;
	for (int i = 1; i < numlevels; i++)
		mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;

	double invScaleFactor = 1.0 / scaleFactor;
	mvInvScaleFactor.resize(numlevels);
	mvInvScaleFactor[0] = 1;
	for (int i = 1; i < numlevels; i++)
		mvInvScaleFactor[i] = mvInvScaleFactor[i - 1] * invScaleFactor;

	mvImagePyramid.resize(numlevels);
	mvMaskPyramid.resize(numlevels);

	mnFeaturesPerLevel.resize(numlevels);
	double factor = (1.0 / scaleFactor);
	double nDesiredFeaturesPerScale = nfeatures*(1 - factor) /
		(1 - pow(factor, numlevels));

	int sumFeatures = 0;
	for (int level = 0; level < numlevels - 1; level++)
	{
		mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
		sumFeatures += mnFeaturesPerLevel[level];
		nDesiredFeaturesPerScale *= factor;
	}
	mnFeaturesPerLevel[numlevels - 1] = std::max(nfeatures - sumFeatures, 0);

	const int npoints = 2 * 8 * descSize;
	const Point* pattern0 = (const Point*)learned_pattern_64_ORB;
	std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

	//This is for orientation
	// pre-compute the end of a row in a circular patch
	umax.resize(HALF_PATCH_SIZE + 1);

	int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
	int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
	const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
	for (v = 0; v <= vmax; ++v)
		umax[v] = cvRound(sqrt(hp2 - v * v));

	// Make sure we are symmetric
	for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
	{
		while (umax[v0] == umax[v0 + 1])
			++v0;
		umax[v] = v0;
		++v0;
	}
}


int mdBRIEFextractorOct::descriptorSize() const
{
	return descSize;
}

int mdBRIEFextractorOct::descriptorType() const
{
	return CV_8U;
}

int mdBRIEFextractorOct::defaultNorm() const
{
	return NORM_HAMMING;
}

static float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)
{
    int m_01 = 0, m_10 = 0;

    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        m_10 += u * center[u];

    // Go line by line in the circuI853lar patch
    int step = (int)image.step1();
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return fastAtan2((float)m_01, (float)m_10);
}

static void rotateAndDistortPattern(const Point2d& undist_kps,
	const std::vector<Point>& patternIn,
	std::vector<Point>& patternOut,
	cCamModelGeneral_& camModel,
	const double& ax,
	const double& ay)
{
	const size_t npoints = patternIn.size();
	const double npointsd = static_cast<double>(npoints);
	std::vector<double> xcoords(npoints);
	std::vector<double> ycoords(npoints);
	double sumX = 0.0;
	double sumY = 0.0;

	for (size_t p = 0; p < npoints; ++p)
	{
		// rotate pattern point and move it to the keypoint
		double xr = patternIn[p].x*ax - patternIn[p].y*ay + undist_kps.x;
		double yr = patternIn[p].x*ay + patternIn[p].y*ax + undist_kps.y;

		camModel.distortPointsOcam(xr, yr, xcoords[p], ycoords[p]);

		sumX += xcoords[p];
		sumY += ycoords[p];
	}
	double meanX = sumX / npointsd;
	double meanY = sumY / npointsd;
	// substract mean, to get correct pattern size
	for (size_t p = 0; p < npoints; ++p)
	{
		patternOut[p].x = cvRound(xcoords[p] - meanX);
		patternOut[p].y = cvRound(ycoords[p] - meanY);
	}
}

static void rotatePattern(
	const std::vector<Point>& patternIn,
	std::vector<Point>& patternOut,
	const double& ax,
	const double& ay)
{
	const int npoints = (int)patternIn.size();
	for (int p = 0; p < npoints; ++p)
	{
		// rotate pattern point
		patternOut[p].x = cvRound(patternIn[p].x*ax - patternIn[p].y*ay);
		patternOut[p].y = cvRound(patternIn[p].x*ay + patternIn[p].y*ax);
		++p;
		patternOut[p].x = cvRound(patternIn[p].x*ax - patternIn[p].y*ay);
		patternOut[p].y = cvRound(patternIn[p].x*ay + patternIn[p].y*ax);
	}
}

static void compute_ORB(const Mat& image,
	const KeyPoint& keypoint,
	const std::vector<Point>& _pattern,
	cCamModelGeneral_& camModel,
	uchar* descriptor,
	const int& descsize)
{
	const int npoints = _pattern.size();

	std::vector<Point> rotatedPattern(npoints);
	double angle = static_cast<double>(keypoint.angle*DEG2RADf);

	rotatePattern(_pattern,
		rotatedPattern, cos(angle), sin(angle));

	const Point* pattern = &rotatedPattern[0];
	const uchar* center = 0;

	int ix = 0, iy = 0;

	int row = cvRound(keypoint.pt.y);
	int col = cvRound(keypoint.pt.x);
#define GET_VALUE(idx) \
               (ix = pattern[idx].x, \
                iy = pattern[idx].y, \
				center = image.ptr<uchar>(row+iy),\
                center[col+ix] )

	for (int i = 0; i < descsize; ++i, pattern += 16)
	{
		int t0, t1, val;
		t0 = GET_VALUE(0); t1 = GET_VALUE(1);
		val = t0 < t1;
		t0 = GET_VALUE(2); t1 = GET_VALUE(3);
		val |= (t0 < t1) << 1;
		t0 = GET_VALUE(4); t1 = GET_VALUE(5);
		val |= (t0 < t1) << 2;
		t0 = GET_VALUE(6); t1 = GET_VALUE(7);
		val |= (t0 < t1) << 3;
		t0 = GET_VALUE(8); t1 = GET_VALUE(9);
		val |= (t0 < t1) << 4;
		t0 = GET_VALUE(10); t1 = GET_VALUE(11);
		val |= (t0 < t1) << 5;
		t0 = GET_VALUE(12); t1 = GET_VALUE(13);
		val |= (t0 < t1) << 6;
		t0 = GET_VALUE(14); t1 = GET_VALUE(15);
		val |= (t0 < t1) << 7;

		descriptor[i] = (uchar)val;
	}
#undef GET_VALUE
}

static void compute_dBRIEF(const Mat& image,
	const KeyPoint& keypoint,
	const Vec2d& undistortedKeypoint,
	const std::vector<Point>& _pattern,
	cCamModelGeneral_& camModel,
	uchar* descriptor,
	const int& descsize)
{
	const int npoints = _pattern.size();

	std::vector<Point> distortedRotatedPattern(npoints);
	double angle = static_cast<double>(keypoint.angle*DEG2RADf);

	rotateAndDistortPattern(undistortedKeypoint, _pattern,
		distortedRotatedPattern, camModel, cos(angle), sin(angle));

	const Point* pattern = &distortedRotatedPattern[0];
	const uchar* center = 0;

	int ix = 0, iy = 0;

	int row = cvRound(keypoint.pt.y);
	int col = cvRound(keypoint.pt.x);
#define GET_VALUE(idx) \
               (ix = pattern[idx].x, \
                iy = pattern[idx].y, \
				center = image.ptr<uchar>(row+iy),\
                center[col+ix] )

	for (int i = 0; i < descsize; ++i, pattern += 16)
	{
		int t0, t1, val;
		t0 = GET_VALUE(0); t1 = GET_VALUE(1);
		val = t0 < t1;
		t0 = GET_VALUE(2); t1 = GET_VALUE(3);
		val |= (t0 < t1) << 1;
		t0 = GET_VALUE(4); t1 = GET_VALUE(5);
		val |= (t0 < t1) << 2;
		t0 = GET_VALUE(6); t1 = GET_VALUE(7);
		val |= (t0 < t1) << 3;
		t0 = GET_VALUE(8); t1 = GET_VALUE(9);
		val |= (t0 < t1) << 4;
		t0 = GET_VALUE(10); t1 = GET_VALUE(11);
		val |= (t0 < t1) << 5;
		t0 = GET_VALUE(12); t1 = GET_VALUE(13);
		val |= (t0 < t1) << 6;
		t0 = GET_VALUE(14); t1 = GET_VALUE(15);
		val |= (t0 < t1) << 7;

		descriptor[i] = (uchar)val;
	}
#undef GET_VALUE
}

static void compute_mdBRIEF(const Mat& image,
	const KeyPoint& keypoint,
	const Vec2d& undistortedKeypoint,
	const std::vector<Point>& _pattern,
	cCamModelGeneral_& camModel,
	uchar* descriptor,
	uchar* descMask,
	const int& descsize)
{
	const int npoints = 2 * 8 * descsize; //2*512

	std::vector<Point> distortedRotatedPattern(npoints);
	std::vector<std::vector<Point>> maskPattern(2, std::vector<Point>(npoints));
	// the two rotations to learn the mask
	double rot = 20.0 / RHOd;
	double angle = static_cast<double>(keypoint.angle / RHOf);
	double angle1 = angle + rot;
	double angle2 = angle - rot;

	rotateAndDistortPattern(undistortedKeypoint, _pattern,
		distortedRotatedPattern, camModel, cos(angle), sin(angle));

	rotateAndDistortPattern(undistortedKeypoint, _pattern,
		maskPattern[0], camModel, cos(angle1), sin(angle1));

	rotateAndDistortPattern(undistortedKeypoint, _pattern,
		maskPattern[1], camModel, cos(angle2), sin(angle2));

	const Point* pattern = &distortedRotatedPattern[0];
	const Point* maskPattern1 = &maskPattern[0][0];
	const Point* maskPattern2 = &maskPattern[1][0];
	const uchar* center = 0;

	int row = cvRound(keypoint.pt.y);
	int col = cvRound(keypoint.pt.x);
	int ix = 0, iy = 0;
#define GET_VALUE(idx) \
               (ix = pattern[idx].x, \
                iy = pattern[idx].y, \
				center = image.ptr<uchar>(row+iy),\
                center[col+ix] )
#define GET_VALUE_MASK1(idx) \
				(ix = maskPattern1[idx].x, \
                 iy = maskPattern1[idx].y, \
				center = image.ptr<uchar>(row+iy),\
                center[col+ix] )
#define GET_VALUE_MASK2(idx) \
				(ix = maskPattern2[idx].x, \
                 iy = maskPattern2[idx].y, \
				center = image.ptr<uchar>(row+iy),\
                center[col+ix] )
	for (int i = 0; i < descsize; ++i, pattern += 16,
		maskPattern1 += 16, maskPattern2 += 16)
	{
		int temp_val;
		int t0, t1, val, maskVal;
		int mask1_1, mask1_2, mask2_1, mask2_2, stable_val = 0;
		// first bit
		t0 = GET_VALUE(0); t1 = GET_VALUE(1);
		temp_val = t0 < t1;
		val = temp_val;
		mask1_1 = GET_VALUE_MASK1(0); mask1_2 = GET_VALUE_MASK1(1); // mask1
		mask2_1 = GET_VALUE_MASK2(0); mask2_2 = GET_VALUE_MASK2(1); // mask2
		stable_val += (mask1_1 < mask1_2) ^ temp_val;
		stable_val += (mask2_1 < mask2_2) ^ temp_val;
		maskVal = (stable_val == 0);
		stable_val = 0;
		// second bit
		t0 = GET_VALUE(2); t1 = GET_VALUE(3);
		temp_val = t0 < t1;
		val |= temp_val << 1;
		mask1_1 = GET_VALUE_MASK1(2); mask1_2 = GET_VALUE_MASK1(3); // mask1
		mask2_1 = GET_VALUE_MASK2(2); mask2_2 = GET_VALUE_MASK2(3); // mask2
		stable_val += (mask1_1 < mask1_2) ^ temp_val;
		stable_val += (mask2_1 < mask2_2) ^ temp_val;
		maskVal |= (stable_val == 0) << 1;
		stable_val = 0;
		// third bit
		t0 = GET_VALUE(4); t1 = GET_VALUE(5);
		temp_val = t0 < t1;
		val |= temp_val << 2;
		mask1_1 = GET_VALUE_MASK1(4); mask1_2 = GET_VALUE_MASK1(5); // mask1
		mask2_1 = GET_VALUE_MASK2(4); mask2_2 = GET_VALUE_MASK2(5); // mask2
		stable_val += (mask1_1 < mask1_2) ^ temp_val;
		stable_val += (mask2_1 < mask2_2) ^ temp_val;
		maskVal |= (stable_val == 0) << 2;
		stable_val = 0;
		// fourth bit
		t0 = GET_VALUE(6); t1 = GET_VALUE(7);
		temp_val = t0 < t1;
		val |= temp_val << 3;
		mask1_1 = GET_VALUE_MASK1(6); mask1_2 = GET_VALUE_MASK1(7); // mask1
		mask2_1 = GET_VALUE_MASK2(6); mask2_2 = GET_VALUE_MASK2(7); // mask2
		stable_val += (mask1_1 < mask1_2) ^ temp_val;
		stable_val += (mask2_1 < mask2_2) ^ temp_val;
		maskVal |= (stable_val == 0) << 3;
		stable_val = 0;
		// fifth bit
		t0 = GET_VALUE(8); t1 = GET_VALUE(9);
		temp_val = t0 < t1;
		val |= temp_val << 4;
		mask1_1 = GET_VALUE_MASK1(8); mask1_2 = GET_VALUE_MASK1(9); // mask1
		mask2_1 = GET_VALUE_MASK2(8); mask2_2 = GET_VALUE_MASK2(9); // mask2
		stable_val += (mask1_1 < mask1_2) ^ temp_val;
		stable_val += (mask2_1 < mask2_2) ^ temp_val;
		maskVal |= (stable_val == 0) << 4;
		stable_val = 0;
		// sixth bit
		t0 = GET_VALUE(10); t1 = GET_VALUE(11);
		temp_val = t0 < t1;
		val |= temp_val << 5;
		mask1_1 = GET_VALUE_MASK1(10); mask1_2 = GET_VALUE_MASK1(11); // mask1
		mask2_1 = GET_VALUE_MASK2(10); mask2_2 = GET_VALUE_MASK2(11); // mask2
		stable_val += (mask1_1 < mask1_2) ^ temp_val;
		stable_val += (mask2_1 < mask2_2) ^ temp_val;
		maskVal |= (stable_val == 0) << 5;
		stable_val = 0;
		// seventh bit
		t0 = GET_VALUE(12); t1 = GET_VALUE(13);
		temp_val = t0 < t1;
		val |= temp_val << 6;
		mask1_1 = GET_VALUE_MASK1(12); mask1_2 = GET_VALUE_MASK1(13); // mask1
		mask2_1 = GET_VALUE_MASK2(12); mask2_2 = GET_VALUE_MASK2(13); // mask2
		stable_val += (mask1_1 < mask1_2) ^ temp_val;
		stable_val += (mask2_1 < mask2_2) ^ temp_val;
		maskVal |= (stable_val == 0) << 6;
		stable_val = 0;
		// eigth bit
		t0 = GET_VALUE(14); t1 = GET_VALUE(15);
		temp_val = t0 < t1;
		val |= temp_val << 7;
		mask1_1 = GET_VALUE_MASK1(14); mask1_2 = GET_VALUE_MASK1(15); // mask1
		mask2_1 = GET_VALUE_MASK2(14); mask2_2 = GET_VALUE_MASK2(15); // mask2
		stable_val += (mask1_1 < mask1_2) ^ temp_val;
		stable_val += (mask2_1 < mask2_2) ^ temp_val;
		maskVal |= (stable_val == 0) << 7;

		descriptor[i] = (uchar)val;
		descMask[i] = (uchar)maskVal;
	}
#undef GET_VALUE
#undef GET_VALUE_MASK1
#undef GET_VALUE_MASK2

}


static void computeOrientation(const Mat& image,
	vector<KeyPoint>& keypoints,
	const vector<int>& umax)
{
	for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
		keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
	{
		keypoint->angle = IC_Angle(image, keypoint->pt, umax);
	}
}


void ExtractorNode_mdbrief::DivideNode(
	ExtractorNode_mdbrief &n1, 
	ExtractorNode_mdbrief &n2,
	ExtractorNode_mdbrief &n3,
	ExtractorNode_mdbrief &n4)
{
	const int halfX = ceil(static_cast<double>(UR.x - UL.x) / 2.0);
	const int halfY = ceil(static_cast<double>(BR.y - UL.y) / 2.0);

	//Define boundaries of childs
	n1.UL = UL;
	n1.UR = cv::Point2i(UL.x + halfX, UL.y);
	n1.BL = cv::Point2i(UL.x, UL.y + halfY);
	n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
	n1.vKeys.reserve(vKeys.size());

	n2.UL = n1.UR;
	n2.UR = UR;
	n2.BL = n1.BR;
	n2.BR = cv::Point2i(UR.x, UL.y + halfY);
	n2.vKeys.reserve(vKeys.size());

	n3.UL = n1.BL;
	n3.UR = n1.BR;
	n3.BL = BL;
	n3.BR = cv::Point2i(n1.BR.x, BL.y);
	n3.vKeys.reserve(vKeys.size());

	n4.UL = n3.UR;
	n4.UR = n2.BR;
	n4.BL = n3.BR;
	n4.BR = BR;
	n4.vKeys.reserve(vKeys.size());

	//Associate points to childs
	for (size_t i = 0; i<vKeys.size(); i++)
	{
		const cv::KeyPoint &kp = vKeys[i];
		if (kp.pt.x<n1.UR.x)
		{
			if (kp.pt.y<n1.BR.y)
				n1.vKeys.push_back(kp);
			else
				n3.vKeys.push_back(kp);
		}
		else if (kp.pt.y<n1.BR.y)
			n2.vKeys.push_back(kp);
		else
			n4.vKeys.push_back(kp);
	}

	if (n1.vKeys.size() == 1)
		n1.bNoMore = true;
	if (n2.vKeys.size() == 1)
		n2.bNoMore = true;
	if (n3.vKeys.size() == 1)
		n3.bNoMore = true;
	if (n4.vKeys.size() == 1)
		n4.bNoMore = true;

}

vector<cv::KeyPoint> mdBRIEFextractorOct::DistributeOctTree(
	const vector<cv::KeyPoint>& vToDistributeKeys,
	const int &minX,
	const int &maxX,
	const int &minY,
	const int &maxY,
	const int &N,
	const int &level)
{
	// Compute how many initial nodes   
	const int nIni = cvRound(static_cast<double>(maxX - minX) / (maxY - minY));

	const double hX = static_cast<double>(maxX - minX) / nIni;

	list<ExtractorNode_mdbrief> lNodes;

	vector<ExtractorNode_mdbrief*> vpIniNodes;
	vpIniNodes.resize(nIni);

	for (int i = 0; i<nIni; i++)
	{
		ExtractorNode_mdbrief ni;
		ni.UL = cv::Point2i(hX*static_cast<double>(i), 0);
		ni.UR = cv::Point2i(hX*static_cast<double>(i + 1), 0);
		ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
		ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
		ni.vKeys.reserve(vToDistributeKeys.size());

		lNodes.push_back(ni);
		vpIniNodes[i] = &lNodes.back();
	}

	//Associate points to childs
	for (size_t i = 0; i<vToDistributeKeys.size(); i++)
	{
		const cv::KeyPoint &kp = vToDistributeKeys[i];
		vpIniNodes[kp.pt.x / hX]->vKeys.push_back(kp);
	}

	list<ExtractorNode_mdbrief>::iterator lit = lNodes.begin();

	while (lit != lNodes.end())
	{
		if (lit->vKeys.size() == 1)
		{
			lit->bNoMore = true;
			lit++;
		}
		else if (lit->vKeys.empty())
			lit = lNodes.erase(lit);
		else
			lit++;
	}

	bool bFinish = false;

	int iteration = 0;

	vector<pair<int, ExtractorNode_mdbrief*> > vSizeAndPointerToNode;
	vSizeAndPointerToNode.reserve(lNodes.size() * 4);

	while (!bFinish)
	{
		iteration++;

		int prevSize = lNodes.size();

		lit = lNodes.begin();

		int nToExpand = 0;

		vSizeAndPointerToNode.clear();

		while (lit != lNodes.end())
		{
			if (lit->bNoMore)
			{
				// If node only contains one point do not subdivide and continue
				lit++;
				continue;
			}
			else
			{
				// If more than one point, subdivide
				ExtractorNode_mdbrief n1, n2, n3, n4;
				lit->DivideNode(n1, n2, n3, n4);

				// Add childs if they contain points
				if (n1.vKeys.size()>0)
				{
					lNodes.push_front(n1);
					if (n1.vKeys.size()>1)
					{
						nToExpand++;
						vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
						lNodes.front().lit = lNodes.begin();
					}
				}
				if (n2.vKeys.size()>0)
				{
					lNodes.push_front(n2);
					if (n2.vKeys.size()>1)
					{
						nToExpand++;
						vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
						lNodes.front().lit = lNodes.begin();
					}
				}
				if (n3.vKeys.size()>0)
				{
					lNodes.push_front(n3);
					if (n3.vKeys.size()>1)
					{
						nToExpand++;
						vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
						lNodes.front().lit = lNodes.begin();
					}
				}
				if (n4.vKeys.size()>0)
				{
					lNodes.push_front(n4);
					if (n4.vKeys.size()>1)
					{
						nToExpand++;
						vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
						lNodes.front().lit = lNodes.begin();
					}
				}

				lit = lNodes.erase(lit);
				continue;
			}
		}

		// Finish if there are more nodes than required features
		// or all nodes contain just one point
		if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize)
		{
			bFinish = true;
		}
		else if (((int)lNodes.size() + nToExpand * 3)>N)
		{

			while (!bFinish)
			{

				prevSize = lNodes.size();

				vector<pair<int, ExtractorNode_mdbrief*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
				vSizeAndPointerToNode.clear();

				sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end());
				for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--)
				{
					ExtractorNode_mdbrief n1, n2, n3, n4;
					vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

					// Add childs if they contain points
					if (n1.vKeys.size()>0)
					{
						lNodes.push_front(n1);
						if (n1.vKeys.size()>1)
						{
							vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
							lNodes.front().lit = lNodes.begin();
						}
					}
					if (n2.vKeys.size()>0)
					{
						lNodes.push_front(n2);
						if (n2.vKeys.size()>1)
						{
							vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
							lNodes.front().lit = lNodes.begin();
						}
					}
					if (n3.vKeys.size()>0)
					{
						lNodes.push_front(n3);
						if (n3.vKeys.size()>1)
						{
							vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
							lNodes.front().lit = lNodes.begin();
						}
					}
					if (n4.vKeys.size()>0)
					{
						lNodes.push_front(n4);
						if (n4.vKeys.size()>1)
						{
							vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
							lNodes.front().lit = lNodes.begin();
						}
					}

					lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

					if ((int)lNodes.size() >= N)
						break;
				}

				if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize)
					bFinish = true;

			}
		}
	}

	// Retain the best point in each node
	vector<cv::KeyPoint> vResultKeys;
	vResultKeys.reserve(nfeatures);
	for (list<ExtractorNode_mdbrief>::iterator lit = lNodes.begin(); lit != lNodes.end(); lit++)
	{
		vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
		cv::KeyPoint* pKP = &vNodeKeys[0];
		float maxResponse = pKP->response;

		for (size_t k = 1; k<vNodeKeys.size(); k++)
		{
			if (vNodeKeys[k].response>maxResponse)
			{
				pKP = &vNodeKeys[k];
				maxResponse = vNodeKeys[k].response;
			}
		}

		vResultKeys.push_back(*pKP);
	}

	return vResultKeys;
}

void mdBRIEFextractorOct::ComputeKeyPointsOctTree(
	vector<vector<KeyPoint> >& allKeypoints)
{
	allKeypoints.resize(numlevels);

	const double W = 30.0;
	Ptr<AgastFeatureDetector> ag = 
		AgastFeatureDetector::create(fastThreshold, true, fastAgastType);
	Ptr<FastFeatureDetector> fd = 
		FastFeatureDetector::create(fastThreshold, true, fastAgastType);

	for (int level = 0; level < numlevels; ++level)
	{
		const int minBorderX = EDGE_THRESHOLD - 3;
		const int minBorderY = minBorderX;
		const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD + 3;
		const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD + 3;

		vector<cv::KeyPoint> vToDistributeKeys;
		vToDistributeKeys.reserve(nfeatures * 10);

		const double width = (maxBorderX - minBorderX);
		const double height = (maxBorderY - minBorderY);

		const int nCols = width / W;
		const int nRows = height / W;
		const int wCell = ceil(width / nCols);
		const int hCell = ceil(height / nRows);

		for (int i = 0; i<nRows; i++)
		{
			const double iniY = minBorderY + i*hCell;
			double maxY = iniY + hCell + 6;

			if (iniY >= maxBorderY - 3)
				continue;
			if (maxY>maxBorderY)
				maxY = maxBorderY;

			for (int j = 0; j<nCols; j++)
			{
				const double iniX = minBorderX + j*wCell;
				double maxX = iniX + wCell + 6;
				if (iniX >= maxBorderX - 6)
					continue;
				if (maxX>maxBorderX)
					maxX = maxBorderX;

				vector<cv::KeyPoint> vKeysCell;
				if (useAgast)
					ag->detect(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX), 
					vKeysCell, mvMaskPyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX));
				else
					fd->detect(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
					vKeysCell, mvMaskPyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX));

				//FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
				//	vKeysCell, fastThreshold, true);

				//if (vKeysCell.empty())
				//{
				//	if (useAgast)
				//	{
				//		ag->setThreshold(ceil((double)fastThreshold / 2.0));
				//		ag->detect(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
				//			vKeysCell, mvMaskPyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX));
				//	}
				//	else
				//	{
				//		fd->setThreshold(ceil((double)fastThreshold / 2.0));
				//		fd->detect(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
				//			vKeysCell, mvMaskPyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX));
				//	}
				//}

				if (!vKeysCell.empty())
				{
					for (vector<cv::KeyPoint>::iterator vit = vKeysCell.begin(); vit != vKeysCell.end(); vit++)
					{
						(*vit).pt.x += j*wCell;
						(*vit).pt.y += i*hCell;
						vToDistributeKeys.push_back(*vit);
					}
				}

			}
		}

		vector<KeyPoint> & keypoints = allKeypoints[level];
		keypoints.reserve(nfeatures);

		keypoints = DistributeOctTree(vToDistributeKeys, 
			minBorderX, maxBorderX,
			minBorderY, maxBorderY, 
			mnFeaturesPerLevel[level], level);

		const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

		// Add border to coordinates and scale information
		const int nkps = keypoints.size();

		for (int i = 0; i < nkps; ++i)
		{
			keypoints[i].pt.x += minBorderX;
			keypoints[i].pt.y += minBorderY;
			keypoints[i].octave = level;
			keypoints[i].size = scaledPatchSize;
		}
	}

	// compute orientations
	for (int level = 0; level < numlevels; ++level)
		computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);		
}

void mdBRIEFextractorOct::ComputeKeyPointsOld(
	std::vector<std::vector<KeyPoint> > &allKeypoints)
{
	allKeypoints.resize(numlevels);

	double imageRatio = (double)mvImagePyramid[0].cols / mvImagePyramid[0].rows;

	for (int level = 0; level < numlevels; ++level)
	{
		const int nDesiredFeatures = mnFeaturesPerLevel[level];

		const int levelCols = sqrt((double)nDesiredFeatures / (5 * imageRatio));
		const int levelRows = imageRatio*levelCols;

		const int minBorderX = EDGE_THRESHOLD;
		const int minBorderY = minBorderX;
		const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD;
		const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD;

		const int W = maxBorderX - minBorderX;
		const int H = maxBorderY - minBorderY;
		const int cellW = ceil((double)W / levelCols);
		const int cellH = ceil((double)H / levelRows);

		const int nCells = levelRows*levelCols;
		const int nfeaturesCell = ceil((double)nDesiredFeatures / nCells);

		vector<vector<vector<KeyPoint> > > cellKeyPoints(levelRows, vector<vector<KeyPoint> >(levelCols));

		vector<vector<int> > nToRetain(levelRows, vector<int>(levelCols, 0));
		vector<vector<int> > nTotal(levelRows, vector<int>(levelCols, 0));
		vector<vector<bool> > bNoMore(levelRows, vector<bool>(levelCols, false));
		vector<int> iniXCol(levelCols);
		vector<int> iniYRow(levelRows);
		int nNoMore = 0;
		int nToDistribute = 0;


		double hY = cellH + 6;

		for (int i = 0; i<levelRows; i++)
		{
			const double iniY = minBorderY + i*cellH - 3;
			iniYRow[i] = iniY;

			if (i == levelRows - 1)
			{
				hY = maxBorderY + 3 - iniY;
				if (hY <= 0)
					continue;
			}

			double hX = cellW + 6;

			for (int j = 0; j<levelCols; j++)
			{
				double iniX;

				if (i == 0)
				{
					iniX = minBorderX + j*cellW - 3;
					iniXCol[j] = iniX;
				}
				else
				{
					iniX = iniXCol[j];
				}


				if (j == levelCols - 1)
				{
					hX = maxBorderX + 3 - iniX;
					if (hX <= 0)
						continue;
				}


				Mat cellImage = mvImagePyramid[level].rowRange(iniY, iniY + hY).colRange(iniX, iniX + hX);

				cellKeyPoints[i][j].reserve(nfeaturesCell * 5);

				FAST(cellImage, cellKeyPoints[i][j], fastThreshold, true);

				if (cellKeyPoints[i][j].size() <= 3)
				{
					cellKeyPoints[i][j].clear();

					FAST(cellImage, cellKeyPoints[i][j], 5, true);
				}


				const int nKeys = cellKeyPoints[i][j].size();
				nTotal[i][j] = nKeys;

				if (nKeys>nfeaturesCell)
				{
					nToRetain[i][j] = nfeaturesCell;
					bNoMore[i][j] = false;
				}
				else
				{
					nToRetain[i][j] = nKeys;
					nToDistribute += nfeaturesCell - nKeys;
					bNoMore[i][j] = true;
					nNoMore++;
				}

			}
		}


		// Retain by score

		while (nToDistribute>0 && nNoMore<nCells)
		{
			int nNewFeaturesCell = nfeaturesCell + ceil((double)nToDistribute / (nCells - nNoMore));
			nToDistribute = 0;

			for (int i = 0; i<levelRows; i++)
			{
				for (int j = 0; j<levelCols; j++)
				{
					if (!bNoMore[i][j])
					{
						if (nTotal[i][j]>nNewFeaturesCell)
						{
							nToRetain[i][j] = nNewFeaturesCell;
							bNoMore[i][j] = false;
						}
						else
						{
							nToRetain[i][j] = nTotal[i][j];
							nToDistribute += nNewFeaturesCell - nTotal[i][j];
							bNoMore[i][j] = true;
							nNoMore++;
						}
					}
				}
			}
		}

		vector<KeyPoint> & keypoints = allKeypoints[level];
		keypoints.reserve(nDesiredFeatures * 2);

		const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

		// Retain by score and transform coordinates
		for (int i = 0; i<levelRows; i++)
		{
			for (int j = 0; j<levelCols; j++)
			{
				vector<KeyPoint> &keysCell = cellKeyPoints[i][j];
				KeyPointsFilter::retainBest(keysCell, nToRetain[i][j]);
				if ((int)keysCell.size()>nToRetain[i][j])
					keysCell.resize(nToRetain[i][j]);


				for (size_t k = 0, kend = keysCell.size(); k <kend; ++k)
				{
					keysCell[k].pt.x += iniXCol[j];
					keysCell[k].pt.y += iniYRow[i];
					keysCell[k].octave = level;
					keysCell[k].size = scaledPatchSize;
					keypoints.push_back(keysCell[k]);
				}
			}
		}

		if ((int)keypoints.size() > nDesiredFeatures)
		{
			KeyPointsFilter::retainBest(keypoints, nDesiredFeatures);
			keypoints.resize(nDesiredFeatures);
		}
	}

	// and compute orientations
	for (int level = 0; level < numlevels; ++level)
		computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}

void mdBRIEFextractorOct::ComputePyramid(
	cv::Mat image, 
	cv::Mat Mask)
{
	for (int level = 0; level < numlevels; ++level)
	{
		double scale = mvInvScaleFactor[level];
		Size sz(cvRound((double)image.cols*scale), cvRound((double)image.rows*scale));
		Size wholeSize(sz.width + EDGE_THRESHOLD * 2, sz.height + EDGE_THRESHOLD * 2);
		Mat temp(wholeSize, image.type()), masktemp;
		mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

		if (!Mask.empty())
		{
			masktemp = Mat(wholeSize, Mask.type());
			mvMaskPyramid[level] = masktemp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));
		}

		// Compute the resized image
		if (level != 0)
		{
			resize(mvImagePyramid[level - 1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);
			if (!Mask.empty())
			{
				resize(mvMaskPyramid[level - 1], mvMaskPyramid[level], sz, 0, 0, INTER_NEAREST);
			}

			copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
				BORDER_REFLECT_101 + BORDER_ISOLATED);
			if (!Mask.empty())
				copyMakeBorder(mvMaskPyramid[level], masktemp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
				BORDER_CONSTANT + BORDER_ISOLATED);
		}
		else
		{
			copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
				BORDER_REFLECT_101);
			if (!Mask.empty())
				copyMakeBorder(Mask, masktemp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
				BORDER_CONSTANT + BORDER_ISOLATED);
		}
	}

}

static void computeDescriptors(
	const Mat& image,
	vector<KeyPoint>& keypoints,
	vector<Vec2d>& undistortedKeypoints,
	Mat& descriptors,
	Mat& descriptorMasks,
	cCamModelGeneral_& camModel,
	const vector<Point>& pattern,
	const bool learnMasks,
	const bool do_dBrief,
	const int desc_size)
{
	descriptors = Mat::zeros((int)keypoints.size(), desc_size, CV_8UC1);
	descriptorMasks = Mat::zeros((int)keypoints.size(), desc_size, CV_8UC1);

	if (learnMasks)
	{
#pragma omp parallel for num_threads(2)
		for (int i = 0; i < (int)keypoints.size(); ++i)
			compute_mdBRIEF(image,
			keypoints[i], undistortedKeypoints[i],
			pattern, camModel, descriptors.ptr<uchar>(i),
			descriptorMasks.ptr<uchar>(i), desc_size);
	}
	else if (do_dBrief)
	{
#pragma omp parallel for num_threads(2)
		for (int i = 0; i < (int)keypoints.size(); ++i)
			compute_dBRIEF(image,
			keypoints[i], undistortedKeypoints[i],
			pattern, camModel, descriptors.ptr<uchar>(i), desc_size);
	}
	else
	{
		for (int i = 0; i < (int)keypoints.size(); ++i)
			compute_ORB(image,
			keypoints[i], pattern,
			camModel, descriptors.ptr<uchar>(i), desc_size);
	}
}

void mdBRIEFextractorOct::operator()(
	InputArray _image,
	InputArray _mask,
	vector<KeyPoint>& _keypoints,
	cCamModelGeneral_& camModel,
	OutputArray _descriptors,
	OutputArray _descriptorMasks)
{
	if (_image.empty())
		return;

	Mat image = _image.getMat(), mask = _mask.getMat();
	assert(image.type() == CV_8UC1);

	// Pre-compute the scale pyramids
	ComputePyramid(image, mask);

	vector < vector<KeyPoint> > allKeypoints;
	ComputeKeyPointsOctTree(allKeypoints);
	//ComputeKeyPointsOld(allKeypoints);

	Mat descriptors, descriptorMasks;

	int nkeypoints = 0;
	for (int level = 0; level < numlevels; ++level)
		nkeypoints += (int)allKeypoints[level].size();
	if (nkeypoints == 0)
	{
		_descriptors.release();
		_descriptorMasks.release();
	}
	else
	{
		_descriptors.create(nkeypoints, descSize, CV_8U);
		_descriptorMasks.create(nkeypoints, descSize, CV_8U);
		descriptors = _descriptors.getMat();
		descriptorMasks = _descriptorMasks.getMat();
	}

	_keypoints.clear();
	_keypoints.reserve(nkeypoints);

	int offset = 0;

	const double scaleF = camModel.Get_P().at<double>(0);

	for (int level = 0; level < numlevels; ++level)
	{
		vector<KeyPoint>& keypoints = allKeypoints[level];
		int nkeypointsLevel = (int)keypoints.size();

		if (nkeypointsLevel == 0)
			continue;

		// preprocess the resized image
		Mat& workingMat = mvImagePyramid[level];
		//GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);
		boxFilter(workingMat, workingMat, workingMat.depth(), Size(5, 5), Point(-1, -1), true, BORDER_REFLECT_101);
		
		// undistort keypoint coordinates
		std::vector<Vec2d> undistortedKeypoints = std::vector<Vec2d>(nkeypointsLevel);	
		float scale = mvScaleFactor[level];
		if (do_dBrief)
		{
			for (int i = 0; i < nkeypointsLevel; ++i)
			{
				camModel.undistortPointsOcam(
					static_cast<double>(keypoints[i].pt.x*scale), 
					static_cast<double>(keypoints[i].pt.y*scale),
					scaleF, 
					undistortedKeypoints[i](0), 
					undistortedKeypoints[i](1));
			}
		}
		// Compute the descriptors
		Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
		Mat descMasks = descriptorMasks.rowRange(offset, offset + nkeypointsLevel);
		computeDescriptors(workingMat, keypoints, undistortedKeypoints,
			desc, descMasks, camModel, pattern, this->learnMasks, this->do_dBrief,this->descSize);

		offset += nkeypointsLevel;

		// Scale keypoint coordinates
		if (level != 0)
		{
			for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
				keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
				keypoint->pt *= scale;
		}
			
		// And add the keypoints to the output
		_keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
	}
}
}
// The original version was released under the following license
/**
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

// All modifications are released under the following license
/**
* This file is part of MultiCol-SLAM
*
* Copyright (C) 2015-2016 Steffen Urban <rurbste at googlemail.com>
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

#include <iostream>
#include <fstream> 
#include <iomanip>
#include <thread>
#include <mutex>

#include <opencv2/core/core.hpp>

#include "cTracking.h"
#include "cConverter.h"
#include "cam_model_omni.h"
#include "cSystem.h"

using namespace std;

void LoadImagesAndTimestamps(
	const int startFrame,
	const int endFrame,
	const string path2imgs,
	vector<vector<string>> &vstrImageFilenames,
	vector<double> &vTimestamps);

int main(int argc, char **argv)
{
	if (argc != 5)
	{
		cerr << endl << "Usage: ./MultiCol_Slam_Lafida vocabulary_file slam_settings_file path_to_settings path_to_img_sequence" << endl;
		return 1;
	}

	string path2voc = string(argv[1]);
	string path2settings = string(argv[2]);
	string path2calibrations = string(argv[3]);
	string path2imgs = string(argv[4]);

	cout << endl << "MultiCol-SLAM Copyright (C) 2016 Steffen Urban" << endl << endl;
	// --------------
	// 1. Tracking settings
	// --------------
	cv::FileStorage frameSettings(path2settings, cv::FileStorage::READ);

	int traj = (int)frameSettings["traj2Eval"];
	string trajs = to_string(traj);
	const int endFrame = (int)frameSettings["traj.EndFrame"];
	const int startFrame = (int)frameSettings["traj.StartFrame"];

	// --------------
	// 4. Load image paths and timestamps
	// --------------
	vector<vector<string>> imgFilenames;
	vector<double> timestamps;
	LoadImagesAndTimestamps(startFrame, endFrame, path2imgs, imgFilenames, timestamps);

	int nImages = imgFilenames[0].size();

	MultiColSLAM::cSystem MultiSLAM(path2voc, path2settings, path2calibrations, true);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;

	// Main loop
	const int nrCams = static_cast<int>(imgFilenames.size());
	std::vector<cv::Mat> imgs(nrCams);
	for (int ni = 0; ni < nImages; ni++)
	{
		// Read image from file
		std::vector<bool> loaded(nrCams);
		for (int c = 0; c < nrCams; ++c)
		{
			imgs[c] = cv::imread(imgFilenames[c][ni], CV_LOAD_IMAGE_GRAYSCALE);
			if (imgs[c].empty())
			{
				cerr << endl << "Failed to load image at: " << imgFilenames[c][ni] << endl;
				return 1;
			}
		}
		double tframe = timestamps[ni];
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		// Pass the image to the SLAM system
		MultiSLAM.TrackMultiColSLAM(imgs, tframe);

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[ni] = ttrack;

		// Wait to load the next frame
		double T = 0;
		if (ni < nImages - 1)
			T = timestamps[ni + 1] - tframe;
		else if (ni > 0)
			T = tframe - timestamps[ni - 1];
		//std::this_thread::sleep_for(std::chrono::milliseconds(30));

		if (ttrack < T)
			std::this_thread::sleep_for(std::chrono::milliseconds(
			static_cast<long>((T - ttrack))));
	}

	// Stop all threads
	MultiSLAM.Shutdown();

	// Tracking time statistics
	sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for (int ni = 0; ni<nImages; ni++)
	{
		totaltime += vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	MultiSLAM.SaveMKFTrajectoryLAFIDA("MKFTrajectory.txt");

	return 0;
}


void LoadImagesAndTimestamps(const int startFrame,
	const int endFrame,
	const string path2imgs,
	vector<vector<string>> &vstrImageFilenames,
	vector<double> &vTimestamps)
{
	vstrImageFilenames.resize(3);
	ifstream fTimes;
	string strPathTimeFile = path2imgs + "/images_and_timestamps.txt";

	fTimes.open(strPathTimeFile.c_str());
	string line;


	int cnt = 1;
	while (std::getline(fTimes, line))
	{
		if (cnt >= startFrame && cnt < endFrame) // skip until startframe
		{
			std::istringstream iss(line);
			double timestamp;
			string pathimg1, pathimg2, pathimg3;
			if (!(iss >> timestamp >> pathimg1 >> pathimg2 >> pathimg3))
				break;
			vTimestamps.push_back(timestamp);
			vstrImageFilenames[0].push_back(path2imgs + '/' + pathimg1);
			vstrImageFilenames[1].push_back(path2imgs + '/' + pathimg2);
			vstrImageFilenames[2].push_back(path2imgs + '/' + pathimg3);
		}
		++cnt;

	}
}
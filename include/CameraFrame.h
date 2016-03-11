/**
* This file is part of ORB-SLAM.
*
* Copyright (C) Lukas Bischofberger extended
* 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CAMERAFRAME_H
#define CAMERAFRAME_H

namespace ORB_SLAM
{

class Frame
{
public:
    Frame();
    Frame(const Frame &frame);
    Frame(cv::Mat &im, cv::Mat &K, cv::Mat &distCoef);

    // Frame image
    cv::Mat im;

    // Calibration Matrix and k1,k2,p1,p2 Distortion Parameters
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    cv::Mat mDistCoef;

    // Undistorted Image Bounds (computed once)
    static int mnMinX;
    static int mnMaxX;
    static int mnMinY;
    static int mnMaxY;

    static bool mbInitialComputations;


private:

    void UndistortKeyPoints();
    void ComputeImageBounds();

    // Call UpdatePoseMatrices(), before using
    cv::Mat mOw;
    cv::Mat mRcw;
    cv::Mat mtcw;
};

}// namespace ORB_SLAM

#endif // FRAME_H

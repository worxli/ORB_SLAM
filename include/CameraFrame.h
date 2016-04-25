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

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include <Eigen/Eigen>

namespace ORB_SLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;

class CameraFrame
{
public:
    CameraFrame();
    CameraFrame(const CameraFrame &frame);
    CameraFrame(cv::Mat &im, cv::Mat &K, cv::Mat &distCoef, cv::Mat &R, cv::Mat &t, float &xi, cv::Mat &mapX, cv::Mat &mapY, ORBextractor* extractor, ORBVocabulary* voc);

    ORBVocabulary* mpORBvocabulary;
    ORBextractor* mpORBextractor;

    // Frame image
    cv::Mat im;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system)
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector< std::vector<Eigen::Vector3d> > pluckerLines;

    // Number of KeyPoints
    int N;

    // Calibration Matrix and k1,k2,p1,p2 Distortion Parameters
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    cv::Mat mDistCoef;

    cv::Mat mR;
    cv::Mat mt;

    // and mirror parameter
    float mXi;

    // Maps for lense undistortion
    cv::Mat mmapX;
    cv::Mat mmapY;

    // ORB descriptor, each row associated to a keypoint
    cv::Mat mDescriptors;

    // MapPoints associated to keypoints, NULL pointer if not association
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Check if a MapPoint is in the frustum of the camera and also fills variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Undistorted Image Bounds (computed once)
    static int mnMinX;
    static int mnMaxX;
    static int mnMinY;
    static int mnMaxY;

    static bool mbInitialComputations;

    cv::Mat mOw;
    cv::Mat mRcw;
    cv::Mat mtcw;

    // Scale Pyramid Info
    int mnScaleLevels;
    vector<float> mvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    void SetScaleParams(int &_mnScaleLevels, vector<float> _mvScaleFactors, vector<float> _mvLevelSigma2, vector<float> _mvInvLevelSigma2);
    void SetPoseMatrices(cv::Mat _mRcw, cv::Mat _mtcw, cv::Mat _mOw);
    void SetORB(ORBextractor* mpORBextractor, ORBVocabulary* mpORBvocabulary);

private:

    void UndistortKeyPoints();
    void PluckerLine();
    void ComputeImageBounds();
    void undistort(const Eigen::Vector2d& p, Eigen::Vector2d& p_u);
    void UndistortPoint(const Eigen::Vector2d& p, Eigen::Vector2d& p_u, Eigen::Vector3d& P);

};

}// namespace ORB_SLAM

#endif // CAMERAFRAME_H

/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
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

#ifndef FRAME_H
#define FRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "CameraFrame.h"

#include <opencv2/opencv.hpp>
#include<Eigen/Eigen>

namespace ORB_SLAM {
    class tracking;
    class MapPoint;
    class KeyFrame;
    class KeyFrameDatabase;
    class CameraFrame;

    class Frame {
    public:
        Frame();
        Frame(const Frame &frame);
        Frame(vector <CameraFrame> cameraFrames, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc);

        // Camera
        vector <CameraFrame> cameraFrames;

        ORBVocabulary *mpORBvocabulary;
        ORBextractor *mpORBextractor;

        // Frame timestamp
        double mTimeStamp;

        // Bag of Words Vector structures
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // MapPoints associated to keypoints, NULL pointer if not association
        std::vector<MapPoint *> mvpMapPoints;

        // Flag to identify outlier associations
        std::vector<bool> mvbOutlier;

        // Camera Pose
        cv::Mat mTcw;

        // Current and Next Frame id
        static long unsigned int nNextId;
        long unsigned int mnId;

        KeyFrame *mpReferenceKF;

        void ComputeBoW();

        void UpdatePoseMatrices();

        // Check if a MapPoint is in the frustum of the camera and also fills variables of the MapPoint to be used by the tracking
        bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

        vector <size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, int camera, const int minLevel = -1,
                                          const int maxLevel = -1) const;

        // Scale Pyramid Info
        int mnScaleLevels;
        float mfScaleFactor;
        vector<float> mvScaleFactors;
        vector<float> mvLevelSigma2;
        vector<float> mvInvLevelSigma2;

    private:

        // Call UpdatePoseMatrices(), before using
        cv::Mat mOw;
        cv::Mat mRcw;
        cv::Mat mtcw;
    };

}// namespace ORB_SLAM

#endif // FRAME_H

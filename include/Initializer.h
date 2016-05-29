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

#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"
#include <opengv/types.hpp>


namespace ORB_SLAM
{

class Initializer
{
    typedef pair<int,int> Match;

public:

    // Fix the reference frame
    Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool Initialize(const Frame &CurrentFrame, const vector<vector<int> > &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<vector<cv::Point3f> > &vP3D, vector<vector<bool> > &vbTriangulated);
    void InitializeGenCam();

private:

    void TriangulateOpenGV(const opengv::transformation_t best_transformation, opengv::bearingVectors_t bearingVectors1, opengv::bearingVectors_t bearingVectors2,
                           std::vector<int> mvCorr1, std::vector<int> mvCorr2, opengv::rotation_t mR, opengv::translation_t mt, int index, opengv::point_t point);

    bool CheckRelativePose(const cv::Mat &R, const cv::Mat &t, vector<vector<cv::Point3f> > &vP3D, vector<vector<bool> > &vbTriangulated,  vector<vector<bool> > vbMatchesInliers);

    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);


    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax, cv::Mat mR, cv::Mat mt, vector<int> bearingMatch, int camera);




    // Keypoints from Reference Frame (Frame 1)
    vector<vector<cv::KeyPoint> > mvKeys1;

    // Keypoints from Current Frame (Frame 2)
    vector<vector<cv::KeyPoint> > mvKeys2;

    // Current Matches from Reference to Current
    vector<vector<Match> > mvMatches12;
    vector<vector<bool> > mvbMatched1;

    // Calibration
    vector<cv::Mat> mK;
    vector<cv::Mat> mR;
    vector<cv::Mat> mt;

    // Standard Deviation and Variance
    float mSigma, mSigma2;

    // Ransac max iterations
    int mMaxIterations;

    // Ransac sets
    vector<vector<size_t> > mvSets;

    // # of cameras
    int cameras;

    //
    vector<vector<Eigen::Vector3d> > mvBearings1;

    // sample data
    cv::Mat gR;
    cv::Mat gt;

    vector<Eigen::Vector3d> v1c1;
    vector<Eigen::Vector3d> v2c1;
    vector<Eigen::Vector3d> v1c2;
    vector<Eigen::Vector3d> v2c2;

    Eigen::Matrix3d c1R;
    Eigen::Vector3d c1t;
    Eigen::Matrix3d c2R;
    Eigen::Vector3d c2t;

    void generateSampleData();

    // generalized camera
    opengv::bearingVectors_t mvBearings1Adapter;
    opengv::bearingVectors_t mvBearings2Adapter;
    std::vector<int> mvCorr1;
    std::vector<int> mvCorr2;
    opengv::rotations_t mvR;
    opengv::translations_t mvT;

    vector<vector<int> > matchesBearing;
    opengv::transformation_t best_transformation;

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H

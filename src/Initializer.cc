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

#include "Initializer.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include "Optimizer.h"
#include "ORBmatcher.h"
#include "../include/Initializer.h"

#include<boost/thread.hpp>
#include<opengv/relative_pose/RelativeAdapterBase.hpp>
#include<opengv/relative_pose/NoncentralRelativeAdapter.hpp>
#include<opengv/relative_pose/CentralRelativeAdapter.hpp>
#include<opengv/relative_pose/methods.hpp>
#include <opengv/triangulation/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>
#include "../Thirdparty/opengv/test/random_generators.hpp"
#include "../Thirdparty/opengv/test/experiment_helpers.hpp"

using namespace opengv;

namespace ORB_SLAM
{

Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
    cameras = ReferenceFrame.cameraFrames.size();

    for(uint i = 0; i<cameras; i++) {
        mK.push_back(ReferenceFrame.cameraFrames[i].mK.clone());
        mR.push_back(ReferenceFrame.cameraFrames[i].mR.clone());
        mt.push_back(ReferenceFrame.cameraFrames[i].mt.clone());
        mvKeys1.push_back(ReferenceFrame.cameraFrames[i].mvKeysUn);
        mvBearings1.push_back(ReferenceFrame.cameraFrames[i].vBearings);
    }

    mSigma = sigma;
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}

bool Initializer::Initialize(const Frame &CurrentFrame, const vector<vector<int> > &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<vector<cv::Point3f> > &vP3D, vector<vector<bool> > &vbTriangulated)
{
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2

    mvBearings1Adapter.clear();
    mvBearings2Adapter.clear();
    matchesBearing.clear();
    mvCorr1.clear();
    mvCorr2.clear();

    matchesBearing.resize(cameras);
    for(int j =0; j<cameras; j++) {
        matchesBearing[j].resize(vMatches12[j].size());
        mvKeys2.push_back(CurrentFrame.cameraFrames[j].mvKeysUn);

        // check for matches and get the bearing vectors
        for (size_t i = 0, iend = vMatches12[j].size(); i < iend; i++) {
            if (vMatches12[j][i] >= 0) {
                const cv::KeyPoint &kp1 = mvKeys1[j][i];
                const cv::KeyPoint &kp2 = mvKeys2[j][vMatches12[j][i]];

//                cout << "---------------" << endl;
//                cout << "kp1 " << kp1.pt.x << "," << kp1.pt.y << endl;
//                cout << "kp2 " << kp2.pt.x << "," << kp2.pt.y << endl;
//                cout << "bearing" << mvBearings1[j][i] << endl;
//                cout << "bearing" << mvBearings1[j][vMatches12[j][i]] << endl;

                mvBearings1Adapter.push_back(mvBearings1[j][i]);
                mvBearings2Adapter.push_back(CurrentFrame.cameraFrames[j].vBearings[vMatches12[j][i]]);
                mvCorr1.push_back(j);
                mvCorr2.push_back(j); // because we're only correlating camera 1 to camera 1
                matchesBearing[j][i] = mvBearings1Adapter.size()-1;
            }
        }

        cout << "correspondences " << mvBearings1Adapter.size() << endl;

        Eigen::Matrix3d mR;
        Eigen::Vector3d mt;
        cv::cv2eigen(CurrentFrame.cameraFrames[j].mR, mR);
        cv::cv2eigen(CurrentFrame.cameraFrames[j].mt, mt);
        mvR.push_back(mR);
        mvT.push_back(mt);
    }

    //generateSampleData();

    // create the non-central relative adapter
    relative_pose::NoncentralRelativeAdapter adapter(mvBearings1Adapter, mvBearings2Adapter, mvCorr1, mvCorr2, mvT, mvR);


    // create a RANSAC object
    opengv::sac::Ransac<opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem>
            ransac;

    // create a NoncentralRelativePoseSacProblem
    std::shared_ptr<opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem>
            relposeproblem_ptr(
            new opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem(
                    adapter, opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem::SEVENTEENPT)
    );
    // run ransac
    ransac.sac_model_ = relposeproblem_ptr;
//    ransac.threshold_ = 0.8 - cos(atan(sqrt(2.0)*10/800.0));
    ransac.threshold_ = 2.0*(1.0 - cos(atan(sqrt(2.0)*0.5/800.0)));
    ransac.max_iterations_ = 1000;
    cout << "compute model" << endl;


    ransac.computeModel();
/*
    ///////////////////////////////////////////////////////////////////////////////////

    vector<size_t> vAvailableIndices;
    vector<size_t> vAllIndices;

    for(int i=0; i<mvBearings1Adapter.size(); i++)
    {
        vAllIndices.push_back(i);
    }

    opengv::rotations_t sixpt_rotations;
    opengv::transformation_t seventeenpt_transformation_all;

    for( size_t i = 0; i < 400; i++ ) {
        std::vector<int> indices6(6,0);
        vAvailableIndices = vAllIndices;
        for(size_t j=0; j<6; j++)
        {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            indices6[j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
        sixpt_rotations = opengv::relative_pose::sixpt(adapter,indices6);
        seventeenpt_transformation_all = relative_pose::seventeenpt(adapter);
    }


    std::cout << "results from 6pt algorithm:" << std::endl;
    for( int i = 0; i < sixpt_rotations.size(); i++ )
        std::cout << sixpt_rotations[i] << std::endl << std::endl;

    std::cout << "results from 17pt algorithm with all points:" << std::endl;
    std::cout << seventeenpt_transformation_all << std::endl << std::endl;
*/


    //////////////////////////////////////////////////////////////////////////////////

//
//    opengv::transformation_t seventeenpt_transformation_all;
//
//    for( size_t i = 0; i < 10; i++ ) {
//        seventeenpt_transformation_all = opengv::relative_pose::seventeenpt(adapter);
//    }
//
//    translation_t translation = seventeenpt_transformation_all.col(3);
//    rotation_t rotation = seventeenpt_transformation_all.block<3,3>(0,0);
//
//    adapter.sett12(translation);
//    adapter.setR12(rotation);
//    transformation_t best_transformation =
//            relative_pose::optimize_nonlinear(adapter);
//
//    std::cout << best_transformation << std::endl << std::endl;

    //////////////////////////////////////////////////////////////////////////////////


    //print the results
    std::cout << "the ransac threshold is: " << ransac.threshold_ << std::endl;
    std::cout << "the ransac results is: " << std::endl;
    std::cout << ransac.model_coefficients_ << std::endl << std::endl;
    std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
    std::cout << "the number of inliers is: " << ransac.inliers_.size();
    std::cout << std::endl << std::endl;
    std::cout << "the found inliers are: " << std::endl;
    for(size_t i = 0; i < ransac.inliers_.size(); i++)
        std::cout << ransac.inliers_[i] << " ";
    std::cout << std::endl << std::endl;

    // get the result
    best_transformation = ransac.model_coefficients_;

    cv::Mat Tcw;
    cv::eigen2cv(best_transformation, Tcw);

    cv::Mat Rcw = cv::Mat::eye(3,3,CV_32F);
    cv::Mat tcw = cv::Mat::zeros(3,1, CV_32F);
    Tcw.rowRange(0,3).colRange(0,3).copyTo(Rcw);
    Tcw.rowRange(0,3).col(3).copyTo(tcw);

    Rcw.convertTo(Rcw, CV_32F);
    tcw.convertTo(tcw, CV_32F);
    R21 = Rcw;
    t21 = tcw;

    vector<vector<bool> > vbMatchesInliers(cameras, vector<bool>(vMatches12[0].size(), false));
    for(int i =0; i<ransac.inliers_.size(); i++) {
        for (int j = 0; j < cameras; ++j) {
            for (int k = 0; k < vMatches12[0].size(); ++k) {
                if (matchesBearing[j][k] == ransac.inliers_[i]) {
                    vbMatchesInliers[j][k] = true;
                }
            }
        }
    }

    // fill old matches structure
    mvMatches12.clear();
    for(int j =0; j<cameras; j++) {
        vector<Match> matches;
        matches.reserve(mvKeys2[j].size());
        mvMatches12.push_back(matches);
        for (size_t i = 0, iend = vMatches12[j].size(); i < iend; i++) {
            if (vMatches12[j][i] >= 0)
                mvMatches12[j].push_back(make_pair(i, vMatches12[j][i]));
        }
    }

    return CheckRelativePose(R21, t21, vP3D, vbTriangulated, vbMatchesInliers);
}

bool Initializer::CheckRelativePose(const cv::Mat &R, const cv::Mat &t, vector<vector<cv::Point3f> > &vP3D, vector<vector<bool> > &vbTriangulated, vector<vector<bool> > vbMatchesInliers)
{
    int nGood = 0;

    // reproject points and check score
    vbTriangulated.clear();
    vbTriangulated.resize(cameras);
    vP3D.clear();
    vP3D.resize(cameras);

    cout << "R " << R << endl;
    cout << "t " << t << endl;

    for(uint i = 0; i<cameras; i++) {
        float parallaxi;
        vector<cv::Point3f> mvP3Di;
        vector<bool> mvbTriangulated;

//        std::cout << "R: " << std::endl << R << std::endl
//                  << "t: " << t << std::endl
////                  << "mvKeys1: " << mvKeys1[i] << std::endl
////                  << "mvKeys2: " << mvKeys2[i] << std::endl
////                  << "vbMatchesInliers: " << vbMatchesInliers
//                  << "mK: " << std::endl << mK[i] << std::endl
//                  << "mvP3Di: " << std::endl << mvP3Di << std::endl
//                  << "4*mSigma2: " << mSigma2 << std::endl
////                  << "mvbTri: " << mvbTriangulated << std::endl
//                  << "parallaxi: " << parallaxi << std::endl
//                  << "mR: " << mR[i] << std::endl
//                  << "mt: " << mt[i] << std::endl;

        nGood += CheckRT(R, t, mvKeys1[i], mvKeys2[i], mvMatches12[i], vbMatchesInliers[i], mK[i], mvP3Di,
                            12.0 * mSigma2, mvbTriangulated, parallaxi, mR[i], mt[i], matchesBearing[i], i);
        // should be 4.0 * mSigma2
        vP3D[i] = mvP3Di;
        vbTriangulated[i] = mvbTriangulated;
        cout << "CheckRelativePose: ngood " << nGood << endl;
    }
    return nGood > 5;
}


void Initializer::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    cv::Mat x3d = vt.row(3).t();
    x3d = x3d.rowRange(0,3)/x3d.at<float>(3);
    x3D.at<float>(0) = x3d.at<float>(0);
    x3D.at<float>(1) = x3d.at<float>(1);
    x3D.at<float>(2) = x3d.at<float>(2);
    x3D.at<float>(3) = 1.0f;
}

void Initializer::TriangulateOpenGV(opengv::transformation_t best_transformation, opengv::bearingVectors_t bearingVectors1, opengv::bearingVectors_t bearingVectors2,
                                    std::vector<int> mvCorr1, std::vector<int> mvCorr2, opengv::rotation_t mR, opengv::translation_t mt, int index, opengv::point_t point)
{
    opengv::bearingVectors_t mBearingVectors1;
    opengv::bearingVectors_t mBearingVectors2;

    cout << "push bearing: " << bearingVectors1[index] << endl;
    mBearingVectors1.push_back((mR*bearingVectors1[index] + mt)/(mR*bearingVectors1[index] + mt).norm());
    mBearingVectors2.push_back((mR*bearingVectors2[index] + mt)/(mR*bearingVectors2[index] + mt).norm());

    opengv::translation_t position = best_transformation.col(3);
    opengv::rotation_t rotation = best_transformation.block<3,3>(0,0);

    //create a central relative adapter and pass the relative pose
    opengv::relative_pose::CentralRelativeAdapter adapter(
            bearingVectors1,
            bearingVectors2,
            position,
            rotation);

    point = opengv::triangulation::triangulate(adapter, 0);
}

int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax, cv::Mat mR, cv::Mat mt, vector<int> bearingMatch, int camera)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    cv::Mat Kproj = cv::Mat::zeros(3,4, CV_32F);
    K.copyTo(Kproj.rowRange(0,3).colRange(0,3));

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 with baseframe at position 0 --> Rot. = eye and trans = 0
    // Comined rotation and translation world->base->camera //Tcb

    cv::Mat Tbw1 = cv::Mat::zeros(4,4,CV_32F);
    cv::Mat Rbw1 = cv::Mat::eye(3,3,CV_32F);
    cv::Mat tbw1 = cv::Mat::zeros(3,1,CV_32F);
    Rbw1.copyTo(Tbw1.rowRange(0,3).colRange(0,3));
    tbw1.copyTo(Tbw1.rowRange(0,3).col(3));
    Tbw1.at<float>(3,3) = 1.0f;
    cv::Mat Twb1 = Tbw1.inv();


    cv::Mat Rcb = mR;
    cv::Mat tcb = mt;
    cv::Mat Tcb = cv::Mat::zeros(4,4,CV_32F);
    Rcb.copyTo(Tcb.rowRange(0,3).colRange(0,3));
    tcb.copyTo(Tcb.rowRange(0,3).col(3));
    Tcb.at<float>(3,3) = 1.0f;

    cv::Mat Tbc = Tcb.inv();

    cv::Mat Rbc1, tbc1;
    Tbc.rowRange(0,3).colRange(0,3).copyTo(Rbc1);
    Tbc.rowRange(0,3).col(3).copyTo(tbc1);

    cv::Mat Tcw1(4,4,CV_32F);
    Tcw1 = Tbw1*Tcb;

    // Camera 1 Projection Matrix K[Rc|tc][I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
//    Tbc.rowRange(0,3).colRange(0,3).copyTo(P1.rowRange(0,3).colRange(0,3));
//    Tbc.rowRange(0,3).col(3).copyTo(P1.rowRange(0,3).col(3));
    P1 = Kproj*Tbc*Twb1;

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);
//    cv::Mat O1 = tcb;
//    cv::Mat O1 = -Tcw1.rowRange(0,3).colRange(0,3).t()*Tcw1.rowRange(0,3).col(3);

//    cout << "-----------------------------------------" << endl;
//    cout << "R base - world 1 " << Rbw1 << " t " << tbw1 << endl;
//    cout << "R camera - base " << Rcb << " t " << tcb << endl;
//    cout << "camera center 1 " << O1 << endl;

    cv::Mat Rbw2 = R;
    cv::Mat tbw2 = t;
    cv::Mat Tbw2 = cv::Mat::zeros(4,4,CV_32F);
    Rbw2.copyTo(Tbw2.rowRange(0,3).colRange(0,3));
    tbw2.copyTo(Tbw2.rowRange(0,3).col(3));
    Tbw2.at<float>(3,3) = 1.0f;
    cv::Mat Twb2 = Tbw2.inv();

    cv::Mat Tcw2(4,4,CV_32F);
    Tcw2 = Tbw2*Tcb;

//    cout << "Tcw2 " << Tcw2 << endl;

    cv::Mat Twc2 = Tcw2.inv();
    cv::Mat Rcw2 = Tcw2.rowRange(0,3).colRange(0,3);
    cv::Mat tcw2 = Tcw2.rowRange(0,3).col(3);
    cv::Mat Rwc2 = Twc2.rowRange(0,3).colRange(0,3);
    cv::Mat twc2 = Twc2.rowRange(0,3).col(3);

//    cout << "-------" << endl;
//    cout << "R base - world 2 " << Rbw2 << " t " << tbw2 << endl;
//    cout << "R camera world 2" << Rcw2 << " t " << tcw2 << endl;

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,CV_32F);
//    Tbc.rowRange(0,3).colRange(0,3).copyTo(P2.rowRange(0,3).colRange(0,3));
//    Tbc.rowRange(0,3).col(3).copyTo(P2.rowRange(0,3).col(3));

    P2 = Kproj*Tbc*Twb2;

    cv::Mat O2 = cv::Mat::zeros(3,1,CV_32F);
//    cv::Mat O2 = Rbw2.t()*tcb-Rbw2.t()*tbw2;

//    cout << "camera center 2 " << O2 << endl;

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        if(!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

        //-- --------------- ------------ ------------ ------------ ------------ ------------ ------------ -------

//        int index = bearingMatch[i];
//        opengv::point_t point;
//        TriangulateOpenGV(best_transformation, mvBearings1Adapter, mvBearings2Adapter, mvCorr1, mvCorr2, mvR[camera], mvT[camera], index, point);

        // -------------------------------------------------------------------------------------------------------
        cv::Mat p3dC1(4,1,CV_32F);

//        cout << "opengv triangulated " << point << endl;

//        cv::Mat p3dC1 = (cv::Mat_<double>(1,3) << point(0) , point(1) , point(2));
//        p3dC1(0) = point(0);
//        p3dC1(1) = point(1);
//        p3dC1(2) = point(2);

//        cout << "---------------" << endl;
//        cout << "kp1 " << kp1.pt.x << "," << kp1.pt.y << endl;
//        cout << "kp2 " << kp2.pt.x << "," << kp2.pt.y << endl;

        Triangulate(kp1,kp2,P1,P2,p3dC1);

//        cout << "p3dC1 " << p3dC1 << endl;
//        cv::Mat p3dC1 = p3dC*Tbw1.t();

        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

//        cout << "---------------" << endl;
//        cout << "kp1 " << kp1.pt.x << "," << kp1.pt.y << endl;
//        cout << "kp2 " << kp2.pt.x << "," << kp2.pt.y << endl;
//        cout << "bearing" << mvBearings1[camera][vMatches12[i].first] << endl;
//        cout << "bearing" << mvBearings1[camera][vMatches12[i].second] << endl;

        // Check parallax
        cv::Mat _p3dC1 = Tbc*p3dC1;
        cv::Mat normal1 = _p3dC1.rowRange(0,3) - O1;
        float dist1 = cv::norm(normal1);
//        cout << _p3dC1 << " " << normal1 << " " << dist1 << endl;
        cv::Mat _p3dC2 = Twc2*p3dC1;
        cv::Mat normal2 = _p3dC2.rowRange(0,3) - O2;
        float dist2 = cv::norm(normal2);
//        cout << _p3dC2 << " " << normal2 << " " << dist2 << endl;

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);
//        cout << "parallax " << cosParallax << endl;

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(_p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2world = Twb2*p3dC1;
        if(_p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // directly transform to camera coordinate system
        cv::Mat p3dC2 = Twc2*p3dC1;

        // transform to camera: Tbc
        p3dC1 = Tbc*p3dC1;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        if(squareError1>th2)
            continue;

//        cout << im1x << "," << im1y << " - " << kp1.pt.x << "," << kp1.pt.y << endl;
//        cout << "P1" << P1 << endl;
//        cout << "p2" << P2 << endl;
//        cout << "3d points " << p3dC1 << " " << p3dC2 << " world: " << p3dC2world << endl;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

//        cout << im2x << "," << im2y << " - " << kp2.pt.x << "," << kp2.pt.y << endl;

        if(squareError2>th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

//        cout << "kp1 " << kp1.pt.x << "," << kp1.pt.y << endl;
//        cout << "kp2 " << kp2.pt.x << "," << kp2.pt.y << endl;
//        cout << "error 1: " << squareError1 << endl;
//        cout << "error 2: " << squareError2 << endl;
//        cout << "p3dC1 " << p3dC1 << endl;

        if(cosParallax<0.99998) {
            vbGood[vMatches12[i].first]=true;
//            cout << "set vbgood at " << vMatches12[i].first << " at camera " << camera << endl;
//            cout << vP3D[vMatches12[i].first] << " to " << vbGood[vMatches12[i].first] << endl;
        }
    }

    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());

        size_t idx = min(50,int(vCosParallax.size()-1));
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}

void Initializer::generateSampleData()
{
    // initialize random seed
    struct timeval tic;
    gettimeofday( &tic, 0 );
    srand ( tic.tv_usec );

    //set experiment parameters
    double noise = 0.3;
    double outlierFraction = 0.3;
    size_t numberPoints = 1000;
    int numberCameras = 4;

    //generate a random pose for viewpoint 1
    translation_t position1 = Eigen::Vector3d::Zero();
    rotation_t rotation1 = Eigen::Matrix3d::Identity();

    //generate a random pose for viewpoint 2
    translation_t position2 = generateRandomTranslation(2.0);
    rotation_t rotation2 = generateRandomRotation(0.5);

    //create a fake central camera
    generateRandomCameraSystem( numberCameras, mvT, mvR );

    //derive correspondences based on random point-cloud
    Eigen::MatrixXd gt(3,numberPoints);
    generateRandom2D2DCorrespondences(
            position1, rotation1, position2, rotation2,
            mvT, mvR, numberPoints, noise, outlierFraction,
            mvBearings1Adapter, mvBearings2Adapter,
            mvCorr1, mvCorr2, gt );

    //Extract the relative pose
    translation_t position; rotation_t rotation;
    extractRelativePose(
            position1, position2, rotation1, rotation2, position, rotation, false );

    //print experiment characteristics
    printExperimentCharacteristics( position, rotation, noise, outlierFraction );
}

} //namespace ORB_SLAM

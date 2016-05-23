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

    //generateSampleData();
    //test();
}

void Initializer::generateSampleData()
{
//    gR =(cv::Mat_<float>(3,3) << 0.9975167526, -0.0094179208, 0.0697970700, -0.0572561871, -0.6855342392, 0.7257854613, 0.0410128913, -0.7279794706, -0.6843711224);
//    gt =(cv::Mat_<float>(3,1) << 1.8693504635, 0.7787120638, 0.8834578976);
    gR =(cv::Mat_<float>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    gt =(cv::Mat_<float>(3,1) << 0, 0, 0);
    cv::Mat mc1R;
    cv::Mat mc1t;
    cv::Mat mc2R;
    cv::Mat mc2t;

    mc1R =(cv::Mat_<float>(3,3) << -0.0062716301, 0.0303626693, 0.9995192719, -0.9999429069, -0.0088381698, -0.0060058088, 0.0086515687, -0.9994998725, 0.0304163655);
    mc1t =(cv::Mat_<float>(3,1) << 3.3273137587, -0.1992388656, 0.5566928679);
    mc2R =(cv::Mat_<float>(3,3) << 0.01, 1, 0.2, 0.001, -1.1, 0.06, -0.008, 0.004, 0.7);
    mc2t =(cv::Mat_<float>(3,1) << -1, 0.5, 2);

    vector<cv::Mat> mv1c1;
    vector<cv::Mat> mv2c1;
    vector<cv::Mat> mv1c2;
    vector<cv::Mat> mv2c2;

    Eigen::Matrix<double, 3, 1> mv1point;
    Eigen::Matrix<double, 3, 1> mv2point;

    srand (time(NULL));
    cv::Mat p1;
    cv::Mat p2;
    for(uint i = 0; i<100; i++) {
        p1 = (cv::Mat_<float>(3,1) << rand() % 1+0.1, rand() % 1+0.05, rand() % 1-0.1 );
//        cv::Mat p1 = (cv::Mat_<float>(3,1) << rand() % 10, rand() % 10, rand() % 10 );
//        cv::Mat p2 = (cv::Mat_<float>(3,1) << rand() % 10, rand() % 10, rand() % 10 );
        p2 = (cv::Mat_<float>(3,1) << rand() % 1+0.12, rand() % 1+0.07, rand() % 1-0.2 );
        mv1c1.push_back(mc1R*p1+mc1t);
        mv2c1.push_back(mc1R*(gR*p1+gt)+mc1t);
        mv1c2.push_back(mc2R*p2+mc2t);
        mv2c2.push_back(mc2R*(gR*p2+gt)+mc2t);

        //cout << "gR: " << gR << endl;
        //cout << "gR.inv()*gR: " << gR.inv()*gR << endl;
        //cout << "p1: " << p1;
        //cout << " p1 back calc from mv2c1: " << gR.inv()*(mc1R.inv()*(mv2c1[i]-mc1t)-gt) << endl;
        cout << "mv1c1[" << i << "] w/o norm:" << mv1c1[i] << endl;
        //cout << "mv2c1[" << i << "] w/o norm:" << mv2c1[i] << endl;
        //cout << "mv1c1[" << i << "] back calculated: " << mc1R*(gR.inv()*(mc1R.inv()*(mv2c1[i]-mc1t)-gt))+mc1t << endl;

        mv1c1[i] = mv1c1[i]/cv::norm(mv1c1[i]);
        mv2c1[i] = mv2c1[i]/cv::norm(mv2c1[i]);
        mv1c2[i] = mv1c2[i]/cv::norm(mv1c2[i]);
        mv2c2[i] = mv2c2[i]/cv::norm(mv2c2[i]);

        //cout << "mv1c1[" << i << "] with norm:" << mv1c1[i] << endl;
        //cout << "mv2c1[" << i << "] with norm:" << mv2c1[i] << endl;

        cv::cv2eigen(mv1c1[i], mv1point);
        cv::cv2eigen(mv1c2[i], mv2point);
        v1c1.push_back(mv1point);
        v1c2.push_back(mv2point);

        cv::cv2eigen(mv2c1[i], mv1point);
        cv::cv2eigen(mv2c2[i], mv2point);
        v2c1.push_back(mv1point);
        v2c2.push_back(mv2point);
    }

    cv::cv2eigen(mc1R, c1R);
    cv::cv2eigen(mc1t, c1t);
    cv::cv2eigen(mc2R, c2R);
    cv::cv2eigen(mc2t, c2t);

    cout << "done generating sample data" << endl;
}

//void test()
//    {
//
//    }


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

                cout << "---------------" << endl;
                cout << "kp1 " << kp1.pt.x << "," << kp1.pt.y << endl;
                cout << "kp2 " << kp2.pt.x << "," << kp2.pt.y << endl;

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


    // create the non-central relative adapter
    opengv::relative_pose::NoncentralRelativeAdapter adapter(mvBearings1Adapter, mvBearings2Adapter, mvCorr1, mvCorr2, mvT, mvR );

    // create a RANSAC object
    opengv::sac::Ransac<opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem>
            ransac;

    // create a NoncentralRelativePoseSacProblem
    std::shared_ptr<opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem>
            relposeproblem_ptr(
            new opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem(
                    adapter, opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem::SIXPT)
    );
    // run ransac
    ransac.sac_model_ = relposeproblem_ptr;
    //ransac.threshold_ = 0.8 - cos(atan(sqrt(2.0)*10/800.0));
    ransac.threshold_ = 1.0*(1.0 - cos(atan(sqrt(2.0)*0.5/800.0)));
    ransac.max_iterations_ = 1000;
    cout << "compute model" << endl;

    ransac.computeModel();

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
    opengv::transformation_t best_transformation = ransac.model_coefficients_;

    cv::Mat Tcw;
    cv::eigen2cv(best_transformation, Tcw);
    cv::Mat Rcw;
    cv::Mat tcw;
    Tcw.rowRange(0,3).colRange(0,3).copyTo(Rcw);
    Tcw.rowRange(0,3).col(3).copyTo(tcw);

    Rcw.convertTo(Rcw, CV_32F);
    tcw.convertTo(tcw, CV_32F);
    R21 = Rcw;
    t21 = tcw;//-Rcw*tcw;

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

//    opengv::points_t points;
//    TriangulateOpenGV(best_transformation, mvBearings1Adapter, mvBearings2Adapter, mvCorr1, mvCorr2, mvR, mvT, points);

    // don't know how to get the feature back from a bearing vector
//    vector<vector<cv::Point3f> > triangulated(cameras, vector<cv::Point3f>(vMatches12[j].size()));
//    for (int j = 0; j < cameras; ++j) {
//        for (int i = 0; i < mvBearings1Adapter.size(); ++i) {
//            if (mvCorr1[i] == j) {
//                triangulated[j][i] ==
//            }
//        }
//    }
//
//    vP3D.resize(cameras);
//    for (int j = 0; j < cameras; ++j) {
//        for (int i = 0; i < points.size(); ++i) {
//            if (mvCorr1[i] == j) {
//                cv::Point3f point(points[i](0), points[i](1), points[i](2));
//                vP3D[j].push_back(point);
//            }
//        }
//    }
//
//    cout << vP3D[0].size() << " triangulated" << endl;


    return CheckRelativePose(R21, t21, vP3D, vbTriangulated, vbMatchesInliers);
}

bool Initializer::CheckRelativePose(const cv::Mat &R, const cv::Mat &t, vector<vector<cv::Point3f> > &vP3D, vector<vector<bool> > &vbTriangulated, vector<vector<bool> > vbMatchesInliers)
{
    int nGood = 0;

    // reproject points and check score
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
                            4.0 * mSigma2, mvbTriangulated, parallaxi, mR[i], mt[i]);
        vP3D.push_back(mvP3Di);
        vbTriangulated.push_back(mvbTriangulated);
        cout << "CheckRelativePose: ngood " << nGood << endl;
    }
    return nGood > 5; //TODO
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
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}

void Initializer::TriangulateOpenGV(const opengv::transformation_t best_transformation, opengv::bearingVectors_t bearingVectors1, opengv::bearingVectors_t bearingVectors2,
                                    std::vector<int> mvCorr1, std::vector<int> mvCorr2, opengv::rotations_t mvR, opengv::translations_t mvT, opengv::points_t points)
{
    opengv::bearingVectors_t mBearingVectors1;
    opengv::bearingVectors_t mBearingVectors2;

    cout << "bearings size " << bearingVectors1.size() << endl;
    for (int j = 0; j < mvR.size(); ++j) {
        for (int i = 0; i < bearingVectors1.size(); ++i) {
            // check for camera correspondance -> camera j
            if (mvCorr1[i] == j) {
                cout << "push bearing: " << bearingVectors1[i] << endl;
                mBearingVectors1.push_back((mvR[j]*bearingVectors1[i] + mvT[j])/(mvR[j]*bearingVectors1[i] + mvT[j]).norm());
                mBearingVectors2.push_back((mvR[j]*bearingVectors2[i] + mvT[j])/(mvR[j]*bearingVectors2[i] + mvT[j]).norm());
            }
        }
    }

    opengv::translation_t position = best_transformation.col(3);
    opengv::rotation_t rotation = best_transformation.block<3,3>(0,0);

    //create a central relative adapter and pass the relative pose
    opengv::relative_pose::CentralRelativeAdapter adapter(
            bearingVectors1,
            bearingVectors2,
            position,
            rotation);

    for(size_t j = 0; j < bearingVectors1.size(); j++) {
        points.push_back(opengv::triangulation::triangulate(adapter, j));
    }
}

int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax, cv::Mat mR, cv::Mat mt)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

//    // Camera 1 Projection Matrix K[I|0]
//    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
//    K.copyTo(P1.rowRange(0,3).colRange(0,3));

//    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 1 with baseframe at position 0 --> Rot. = eye and trans = 0
    // Comined rotation and translation world->base->camera //Tcb
    cv::Mat Rbw1 = cv::Mat::eye(3,3,CV_32F);
    cv::Mat Rcb1 = mR;
//    cv::Mat Rcw1 = Rcb1 * Rbw1; //Rbw1 * Rcb1;
//    cv::Mat tbw1 = cv::Mat::zeros(3,1,CV_32F);
//    cv::Mat tcb1 = mt;
//    cv::Mat tcw1 = Rbw1*tcb1 + tbw1; //tcb1 + tbw1;
    //cv::Mat Rcw1 = Rcb1 * Rbw1;
    cv::Mat tbw1 = cv::Mat::zeros(3,1,CV_32F);
    cv::Mat tcb1 = mt;
    //cv::Mat tcw1 = tcb1 + tbw1;

    cv::Mat Tcw1(4,4,CV_32F);
    cv::Mat Tbw1(4,4,CV_32F);
    cv::Mat Tcb1(4,4,CV_32F);
    Rbw1.copyTo(Tbw1.rowRange(0,3).colRange(0,3));
    tbw1.copyTo(Tbw1.rowRange(0,3).col(3));
    Rcb1.copyTo(Tcb1.rowRange(0,3).colRange(0,3));
    tcb1.copyTo(Tcb1.rowRange(0,3).col(3));
    Tcw1 = Tbw1*Tcb1;

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    //Rcw1.copyTo(P1.rowRange(0,3).colRange(0,3));
    //tcw1.copyTo(P1.rowRange(0,3).col(3));
    Tcw1.rowRange(0,3).colRange(0,3).copyTo(P1.rowRange(0,3).colRange(0,3));
    Tcw1.rowRange(0,3).col(3).copyTo(P1.rowRange(0,3).col(3));
    P1 = K*P1;

//    cv::Mat O1 = -Rcw1.t()*tcw1;
    cv::Mat O1 = -Tcw1.rowRange(0,3).colRange(0,3).t()*Tcw1.rowRange(0,3).col(3);


    cout << "-----------------------------------------" << endl;
    cout << "R base - world 1 " << Rbw1 << " t " << tbw1 << endl;
    cout << "R camera - base 1 " << Rcb1 << " t " << tcb1 << endl;
    cout << "R camera world 1" << Rcw1 << " t " << tcw1 << endl;
    cout << "camera center 1 " << O1 << endl;
    cout << "P1 " << P1 << endl;

//    // Camera 2 Projection Matrix K[R|t]
//    cv::Mat P2(3,4,CV_32F);
//    R.copyTo(P2.rowRange(0,3).colRange(0,3));
//    t.copyTo(P2.rowRange(0,3).col(3));
//    P2 = K*P2;

//    cv::Mat O2 = -R.t()*t;

    // Comined rotation and translation world->base->camera //Tcb
    cv::Mat Rbw2 = R;
    cv::Mat Rcb2 = mR;
//    cv::Mat Rcw2 = Rcb2 * Rbw2; //Rbw2 * Rcb2;
//    cv::Mat tbw2 = t; //cv::Mat::zeros(3,1,CV_32F); //t
//    cv::Mat tcb2 = mt;
//    cv::Mat tcw2 = Rbw2*tcb2 + tbw2; //tcb2 + tbw2;

    cout << "-------" << endl;
    cout << "R base - world 2 " << Rbw2 << " t " << tbw2 << endl;
    cout << "R camera - base 2 " << Rcb2 << " t " << tcb2 << endl;
    //cout << "R camera world 2" << Rcw2 << " t " << tcw2 << endl;
//    cv::Mat Rcw2 = Rcb2 * Rbw2;
    cv::Mat tbw2 = t;
    cv::Mat tcb2 = mt;
//    cv::Mat tcw2 = tcb2 + tbw2;

    cv::Mat Tcw2(4,4,CV_32F);
    cv::Mat Tbw2(4,4,CV_32F);
    cv::Mat Tcb2(4,4,CV_32F);
    Rbw2.copyTo(Tbw2.rowRange(0,3).colRange(0,3));
    tbw2.copyTo(Tbw2.rowRange(0,3).col(3));
    Rcb2.copyTo(Tcb2.rowRange(0,3).colRange(0,3));
    tcb2.copyTo(Tcb2.rowRange(0,3).col(3));
    Tcw2 = Tbw2*Tcb2;

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,CV_32F);
//    Rcw2.copyTo(P2.rowRange(0,3).colRange(0,3));
//    tcw2.copyTo(P2.rowRange(0,3).col(3));
    Tcw2.rowRange(0,3).colRange(0,3).copyTo(P2.rowRange(0,3).colRange(0,3));
    Tcw2.rowRange(0,3).col(3).copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

//    cv::Mat O2 = -Rcw2.t()*tcw2;
    cv::Mat O2 = -Tcw2.rowRange(0,3).colRange(0,3).t()*Tcw2.rowRange(0,3).col(3);


    cout << "camera center 2 " << O2 << endl;
    cout << "P2 " << P2 << endl;

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        if(!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        cout << "---------------" << endl;
        cout << "kp1 " << kp1.pt.x << "," << kp1.pt.y << endl;
        cout << "kp2 " << kp2.pt.x << "," << kp2.pt.y << endl;

        // TODO: P1 and P2 not baseframe but cameraframe instead, also for reprojection error needed
        Triangulate(kp1,kp2,P1,P2,p3dC1);

        cout << "p3dC1 " << p3dC1 << endl;
        
        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);
        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);
        cout << "parallax " << cosParallax << endl;

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;
        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = Rcw2*p3dC1+tcw2;

        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        cout << "x" << im1x << "," << im1y << " - " << kp1.pt.x << "," << kp1.pt.y << endl;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        cout << "error 1: " << squareError1 << endl;
        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        cout << "error 2: " << squareError2 << endl;
        if(squareError2>th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
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

    /*
void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}
*/

} //namespace ORB_SLAM

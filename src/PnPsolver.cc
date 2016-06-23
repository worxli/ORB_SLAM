/**
* This file is part of ORB-SLAM.
* This is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php> including a RANSAC scheme
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

#include <iostream>

#include "PnPsolver.h"

#include <vector>
#include <cmath>
#include <opencv/cv.h>
#include "Thirdparty/DBoW2/DUtils/Random.h"
#include <ros/ros.h>
#include <algorithm>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;

namespace ORB_SLAM
{


PnPsolver::PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches):
    pws(0), us(0), alphas(0), pcs(0), maximum_number_of_correspondences(0), number_of_correspondences(0), mnInliersi(0),
    mnIterations(0), mnBestInliers(0), N(0)
{
    mvpMapPointMatches = vpMapPointMatches;
    mvP2D.reserve(F.mvpMapPoints.size());
    mvSigma2.reserve(F.mvpMapPoints.size());
    mvP3Dw.reserve(F.mvpMapPoints.size());
    mvKeyPointIndices.reserve(F.mvpMapPoints.size());
    mvAllIndices.reserve(F.mvpMapPoints.size());

    for(uint i = 0; i<F.cameraFrames.size(); i++) {
        vBearings.insert(vBearings.end(), F.cameraFrames[i].vBearings.begin(), F.cameraFrames[i].vBearings.end());
    }

    int idx=0;
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];

        if(pMP)
        {
            if(!pMP->isBad())
            {
                const cv::KeyPoint &kp = F.cameraFrames[0].mvKeysUn[i];

                mvP2D.push_back(kp.pt);
                mvSigma2.push_back(F.mvLevelSigma2[kp.octave]);

                cv::Mat Pos = pMP->GetWorldPos();
                mvP3Dw.push_back(cv::Point3f(Pos.at<float>(0),Pos.at<float>(1), Pos.at<float>(2)));

                mvKeyPointIndices.push_back(i);
                mvAllIndices.push_back(idx);               

                idx++;
            }
        }
    }

    // Set camera calibration parameters
    // TODO
//    fu = F.cameraFrames[0].fx;
//    fv = F.cameraFrames[0].fy;
//    uc = F.cameraFrames[0].cx;
//    vc = F.cameraFrames[0].cy;

    //SetRansacParameters();
}

PnPsolver::~PnPsolver()
{
  delete [] pws;
  delete [] us;
  delete [] alphas;
  delete [] pcs;
}

cv::Mat PnPsolver::gpnp()
{
    for(uint i=0; i<mvpMapPointMatches.size(); i++) {
        Eigen::Vector3d worldPos;
        cv::cv2eigen(mvpMapPointMatches[i]->GetWorldPos(), worldPos); //TODO check for good mapPoints first
        points.push_back(worldPos);
    }

    //create a non-central absolute adapter
    opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(
            bearingVectors,
            camCorrespondences,
            points,
            camOffsets,
            camRotations);

    // returns rotation and translation
    opengv::transformation_t gpnp_transformation = opengv::absolute_pose::gpnp(adapter);

    size_t iterations = 50;
    for(size_t i = 0; i < iterations; i++)
        gpnp_transformation = opengv::absolute_pose::gpnp(adapter);

    // convert back to return value
    cv::Mat Tcw;
    cv::eigen2cv(gpnp_transformation, Tcw);
    return Tcw;
}


    void PnPSolver::TestGPNP()
    {
        // Define config parameters
        bool bFixData = false;
        unsigned int nKFs = 3;
        unsigned int nCams = 2;
        unsigned int nMPs = 6;
        unsigned int nReferenceKF_ID = 1000;
        unsigned int nFirstMapPointID = nReferenceKF_ID + nKFs;

        // Noise
        // Map Point translational noise
        float fMPSigma = 0.1;//0.2;//1; // noise level, (+-)0.1 m
        // Key Frame translational noise
        float f_t1Noise = 0.1;
        float f_t2Noise = 0.1;
        float f_t3Noise = 0.1;
        // Key Frame rotational noise
        float fRollError = 0.5; //degree
        float fPitchError = 0.5;
        float fYawError = 0.5;

        int nMAX_X2 = 2*10*10;
        int nMAX_Y2 = 2*10*10;
        int nMAX_Z2 = 20*20;

        // Generate Intrinsics
        float fx1, fy1, cx1, cy1; // mono-front
        fx1 = 1408.7635631701; fy1 = 1408.2906649996; cx1 = 653.2604772699; cy1 = 389.0180463604;
        cv::Mat K1 = cv::Mat::eye(3,3,CV_32F);
        K1.at<float>(0,0) = fx1;
        K1.at<float>(1,1) = fy1;
        K1.at<float>(0,2) = cx1;
        K1.at<float>(1,2) = cy1;

        float fx2, fy2, cx2, cy2; // mono-left
        fx2 = 1301.9283442901; fy2 = 1298.8333979612; cx2 = 632.0986331432; cy2 = 391.8724523420;
        cv::Mat K2 = cv::Mat::eye(3,3,CV_32F);
        K2.at<float>(0,0) = fx2;
        K2.at<float>(1,1) = fy2;
        K2.at<float>(0,2) = cx2;
        K2.at<float>(1,2) = cy2;

        std::vector<cv::Mat> K;
        K.push_back(K1);
        K.push_back(K2);

        // Generate Extrinsics
        // mono-front
        cv::Mat Rcb1 =(cv::Mat_<float>(3,3) << -0.0062716301, 0.0303626693, 0.9995192719, -0.9999429069, -0.0088381698, -0.0060058088, 0.0086515687, -0.9994998725, 0.0304163655);
        cv::Mat tcb1 =(cv::Mat_<float>(3,1) << 3.3273137587, -0.1992388656, 0.5566928679);
        // mono-left
        cv::Mat Rcb2 =(cv::Mat_<float>(3,3) << 0.9975167526, -0.0094179208, 0.0697970700, -0.0572561871, -0.6855342392, 0.7257854613, 0.0410128913, -0.7279794706, -0.6843711224);
        cv::Mat tcb2 =(cv::Mat_<float>(3,1) << 1.8693504635, 0.7787120638, 0.8834578976);

        std::vector<cv::Mat> Rcb;
        Rcb.push_back(Rcb1);
        Rcb.push_back(Rcb2);
        std::vector<cv::Mat> tcb;
        tcb.push_back(tcb1);
        tcb.push_back(tcb2);

        srand (time(NULL));

        // Store all Keyframes and Mappoints
        std::vector<KeyFrame*> vAllKeyFrames;
        std::vector<MapPoint*> vAllMapPoints;

        // Generate test data for KF0;
        KeyFrame* pKF0;
        // Generate extra KFs
        for(unsigned int i = 0; i < nKFs; i++){
            KeyFrame* pKFi = new KeyFrame();

            // Check if keyframe is bad and set to not bad
            pKFi->SetmbBad(false);

            // generate pose
            float roll, pitch, yaw;
            if(!bFixData){
                roll = rand()%20 - 10;
                pitch = rand()%20 - 10;
                yaw = rand()%20 - 10;
            }
            else{
                roll = 3*i;
                pitch = 3*i;
                yaw = 3*i;
            }
            roll = roll*3.14/180.0; pitch = pitch*3.14/180.0; yaw = yaw*3.14/180.0;

            tf::Matrix3x3 rotationM;
            rotationM.setEulerZYX(yaw, pitch, roll);
            cv::Mat Rbw2(3,3,CV_32F);
            Rbw2.at<float>(0,0) = rotationM[0][0];
            Rbw2.at<float>(0,1) = rotationM[0][1];
            Rbw2.at<float>(0,2) = rotationM[0][2];

            Rbw2.at<float>(1,0) = rotationM[1][0];
            Rbw2.at<float>(1,1) = rotationM[1][1];
            Rbw2.at<float>(1,2) = rotationM[1][2];

            Rbw2.at<float>(2,0) = rotationM[2][0];
            Rbw2.at<float>(2,1) = rotationM[2][1];
            Rbw2.at<float>(2,2) = rotationM[2][2];

            cv::Mat camCenter(3,1,CV_32F);
            if(!bFixData){
                camCenter.at<float>(0) = rand()%5 - 5;
                camCenter.at<float>(1) = rand()%5 - 5;
                camCenter.at<float>(2) = (rand()%5)*(-1.0);
            }
            else{
                camCenter.at<float>(0) = i*1;
                camCenter.at<float>(1) = i*1;
                camCenter.at<float>(2) = i*(-1.0);
            }

            cout << "\nDEBUG camCenter " << endl << camCenter << endl;
            cv::Mat tbw2 = -Rbw2*camCenter;

            cv::Mat Tbw2 = cv::Mat::eye(4,4,CV_32F);
            Rbw2.copyTo(Tbw2.rowRange(0,3).colRange(0,3));
            tbw2.copyTo(Tbw2.col(3).rowRange(0,3));

            // set pose and ID to KFs
            pKFi->SetPose(Tbw2);
            cout << "DEBUG generated pose " << endl << pKFi->GetPose() << endl;

            // Include cameraframes to keyframe
            CameraFrame* pCF1 = new CameraFrame();
            pCF1->TestSetExtrinsics(Rcb1, tcb1);
            pCF1->TestSetIntrinsics(K[0]);

            CameraFrame* pCF2 = new CameraFrame();
            pCF2->TestSetExtrinsics(Rcb2, tcb2);
            pCF2->TestSetIntrinsics(K[1]);

            pKFi->cameraFrames.push_back(*pCF1);
            pKFi->cameraFrames.push_back(*pCF2);

            std::cout << "\npCF1 fx: " << pCF1->fx << " | " << pCF1->fy << " | " << pCF1->cx << " | " << pCF1->cy << endl;
            std::cout << "\npCF2 fx: " << pCF2->fx << " | " << pCF2->fy << " | " << pCF2->cx << " | " << pCF2->cy << endl << endl;

            pKFi->mnId = i + nReferenceKF_ID;
            vAllKeyFrames.push_back(pKFi);
            if(i == 0){
                pKF0 = pKFi;
            }
        }

        for (int k = 0; k < nCams; ++k) {

            cout << "#######################################################\n\n" << std::endl;
            cout << "DEBUG cam: " << k << endl;

            // generate Map points in KF0 camera frames;
            for(unsigned int i = 0 ; i < nMPs; i++){
                double x, y, z;
                if(!bFixData){
                    x = (rand()%nMAX_X2 - nMAX_X2/2.0)/sqrt(nMAX_X2/2);
                    y = (rand()%nMAX_Y2 - nMAX_Y2/2.0)/sqrt(nMAX_Y2/2);
                    z = (rand()%nMAX_Z2)/sqrt(nMAX_Z2/2) + 2;
                }
                else{
                    x = 0.5*i + 1;
                    y = 0.5*i + 2;
                    z = 0.5*i + 3;
                }

                cout << "DEBUG MPs: " << x << " " << y << " " << z << endl;

                cv::Mat x3D(4,1,CV_32F);
                x3D.at<float>(0) = x; x3D.at<float>(1) = y; x3D.at<float>(2) = z; x3D.at<float>(3) = 1.0;
                // assign ID and world coordinate to generated MP point.
                MapPoint* pMPi = new MapPoint();
                pMPi->mnId = i + nFirstMapPointID + (k*nMPs);

                std::cout << "nFirstMapPointID: " << nFirstMapPointID << std::endl;
                std::cout << "Testfunk: pMPi->mnId: " << pMPi->mnId << std::endl;

                pMPi->SetWorldPos(x3D.rowRange(0,3));
                pMPi->camera = k;

                //project pMPi to keyframes to generate KeyPoint location
                for(size_t j = 0; j < vAllKeyFrames.size(); j++){
                    KeyFrame* pKF = vAllKeyFrames[j];

                    cv::Mat Tbw = pKF->GetPose();

                    cv::Mat Tcb = cv::Mat::eye(4,4,CV_32F);
                    cv::Mat Rcb = pKF->cameraFrames[k].mR;
                    cv::Mat tcb = pKF->cameraFrames[k].mt;
                    Rcb.copyTo(Tcb.rowRange(0,3).colRange(0,3));
                    tcb.copyTo(Tcb.col(3).rowRange(0,3));

                    cv::Mat Tcw(4,4,CV_32F);
                    Tcw = Tbw * Tcb;
                    std::cout << "Tcb: " << Tcb << std::endl;
                    std::cout << "Tbw: " << Tbw << std::endl;
                    std::cout << "Tcw: " << Tcw << std::endl;

                    cv::Mat x3DinCurKP = Tcw.inv()*x3D;
                    x3DinCurKP = x3DinCurKP/x3DinCurKP.at<float>(2);
                    x3DinCurKP = K[k]*x3DinCurKP.rowRange(0,3);
                    cv::KeyPoint kpt;
                    kpt.pt.x = x3DinCurKP.at<float>(0)/x3DinCurKP.at<float>(2);
                    kpt.pt.y = x3DinCurKP.at<float>(1)/x3DinCurKP.at<float>(2);
                    kpt.octave = 2;
                    std::cout << "kpt: " << kpt.pt.x << " | " << kpt.pt.y << std::endl;
                    std::cout << "K[" << k << "]: " << K[k] << std::endl << std::endl;

                    pKF->TestSetFeatureSize(nCams);
                    pKF->TestAddKeyPointUn(kpt, k);
                    pKF->TestAddMapPoint(pMPi);
                    pMPi->AddObservation(pKF,i);
                    pKF->UpdateConnections();
                }
                vAllMapPoints.push_back(pMPi);
            }
        }

        // store orignal pose and MP locations
        std::vector<cv::Mat> vOriginal_Poses;
        std::vector<cv::Mat> vOriginalMP_Position;
        for(size_t i = 0; i < vAllKeyFrames.size(); i++){
            vOriginal_Poses.push_back(vAllKeyFrames.at(i)->GetPose());
        }
        for(size_t i = 0; i < vAllMapPoints.size(); i++){
            vOriginalMP_Position.push_back(vAllMapPoints.at(i)->GetWorldPos());
        }

        // add noise for testing
        vector<MapPoint*> allMPs = pKF0->GetMapPointMatches();
        for(size_t i = 0; i < allMPs.size(); i++){
            MapPoint* pMP = allMPs[i];
            cv::Mat x3D = pMP->GetWorldPos();

            float x_noise = fMPSigma;
            float y_noise = fMPSigma;
            float z_noise = fMPSigma;

            x3D.at<float>(0) += x_noise;
            x3D.at<float>(1) += y_noise;
            x3D.at<float>(2) += z_noise;
            pMP->SetWorldPos(x3D);
        }

        // Pose translation noise
        for(size_t i = 0; i < vAllKeyFrames.size(); i++){
            KeyFrame* pKFi = vAllKeyFrames[i];
            std::cout << "ID KF: " << pKFi->mnId << std::endl;
            if (pKFi->mnId == nReferenceKF_ID || pKFi->mnId == nReferenceKF_ID + 1) continue; // TODO: generalize for more KF's
            cout << "DEBUG Previous Pose: " << endl << pKFi->GetPose() << endl;
            cv::Mat camCenter = pKFi->GetRotation().t()*pKFi->GetTranslation()*(-1.0);

            camCenter.at<float>(0) += f_t1Noise;
            camCenter.at<float>(1) += f_t2Noise;
            camCenter.at<float>(2) += f_t3Noise;

            camCenter = (-1.0)*pKFi->GetRotation()*camCenter;

            cv::Mat pose = pKFi->GetPose();
            camCenter.copyTo(pose.rowRange(0,3).col(3));
            pKFi->SetPose(pose);
            cout << "DEBUG noise corrupted cam center: " << endl << pKFi->GetCameraCenter() << endl;
        }

        // Pose rotation noise
        for(size_t i = 0; i <  vAllKeyFrames.size(); i++){
            KeyFrame* pKFi = vAllKeyFrames[i];
            if(pKFi->mnId == nReferenceKF_ID || pKFi->mnId == nReferenceKF_ID + 1) continue;
            cv::Mat R = pKFi->GetRotation();

            tf::Matrix3x3 r_noise;
            r_noise.setEulerZYX(fYawError*3.14/180.0, fPitchError*3.14/180.0, fRollError*3.14/180.0);
            cv::Mat R_noise(3,3,CV_32F);

            R_noise.at<float>(0,0) = r_noise[0][0];
            R_noise.at<float>(0,1) = r_noise[0][1];
            R_noise.at<float>(0,2) = r_noise[0][2];

            R_noise.at<float>(1,0) = r_noise[1][0];
            R_noise.at<float>(1,1) = r_noise[1][1];
            R_noise.at<float>(1,2) = r_noise[1][2];

            R_noise.at<float>(2,0) = r_noise[2][0];
            R_noise.at<float>(2,1) = r_noise[2][1];
            R_noise.at<float>(2,2) = r_noise[2][2];

            R = R_noise*R;
            cv::Mat T = pKFi->GetPose();
            R.copyTo(T.colRange(0,3).rowRange(0,3));
            pKFi->SetPose(T);
        }

        // store  pose and MP locations after noise is added
        std::vector<cv::Mat> vNoise_Poses;
        std::vector<cv::Mat> vNoiseMP_Position;
        for(size_t i = 0; i < vAllKeyFrames.size(); i++){
            vNoise_Poses.push_back(vAllKeyFrames.at(i)->GetPose());
        }
        for(size_t i = 0; i < vAllMapPoints.size(); i++){
            vNoiseMP_Position.push_back(vAllMapPoints.at(i)->GetWorldPos());
        }

        // call optimization
        vector<bool> vbMatched; vbMatched.resize(nMPs, true);
        bool pbStopFlag = false;
        LocalBundleAdjustment(pKF0, &pbStopFlag);
        PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vAllMapPoints);

        cout << endl << "==================== DEBUG Optimizer::LocalBA ==================" << endl;

        cout << "Number of KeyFrames: " << nKFs << endl;
        cout << "Number of Cameras: " << nCams << endl;
        cout << "Number of Map Points per camera: " << nMPs << " | In total: " << nMPs*nCams << endl << endl;

        cout << "Noise Map Point translation (xyz): " <<  fMPSigma << endl;
        cout << "Noise KeyFrame Pose translation (xyz): " << f_t1Noise << " | " << f_t2Noise << " | " << f_t3Noise << endl;
        cout << "Noise KeyFrame Pose rotation (RPY): " << fRollError << " | " << fPitchError << " | " << fYawError << endl << endl;

        // compute optimization error
        for(size_t i = 0; i < vAllKeyFrames.size(); i++){
            KeyFrame* pKFi = vAllKeyFrames[i];
            cout << "+++++++ Error for keyframe pose   +++++++ " << i << endl;
            cout << vOriginal_Poses[i]*pKFi->GetPose().inv() << endl;

            cout << "+++++++ Error for keyframe center +++++++ " << i << endl;
            cv::Mat T = vOriginal_Poses[i];
            cv::Mat R = T.colRange(0,3).rowRange(0,3);
            cv::Mat t = T.col(3).rowRange(0,3);
            cv::Mat T_noise = vNoise_Poses[i];
            cv::Mat R_noise = T_noise.colRange(0,3).rowRange(0,3);
            cv::Mat t_noise = T_noise.col(3).rowRange(0,3);

            cout << "DEBUG estimated keyframe center of frame: " << pKFi->mnId << endl;
            cout << "ORIGINAL: " << -R.t()*t << endl; // original
            cout << "with NOISE: " << -R_noise.t()*t_noise << endl; // after noise
            cout << "OPTIMIZED: " << pKFi->GetCameraCenter() << endl; // optimized
            cout << "DIFFERENCE: " << -R.t()*t - pKFi->GetCameraCenter() << endl << endl; // difference
        }

        cout << "+++++++ Error for map points locations +++++++ " << endl;
        for(size_t i = 0; i < vAllMapPoints.size(); i++){
            MapPoint* pMP = vAllMapPoints[i];
            cout << "ORIGINAL: " << vOriginalMP_Position[i].t() << endl; // original
            cout << "with NOISE: " << vNoiseMP_Position[i].t() << endl; // after noise
            cout << "OPTIMIZED: " << pMP->GetWorldPos().t() << endl; // optimized
            cout << "DIFFERENCE: " << vOriginalMP_Position[i].t() - pMP->GetWorldPos().t()<< endl; // difference
        }
    }

} //namespace ORB_SLAM

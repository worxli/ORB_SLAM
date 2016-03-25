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

#include "Tracking.h"
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>

#include"ORBmatcher.h"
#include"FramePublisher.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>
#include<fstream>


using namespace std;

namespace ORB_SLAM
{


Tracking::Tracking(ORBVocabulary* pVoc, FramePublisher *pFramePublisher, MapPublisher *pMapPublisher, Map *pMap, vector<string> strSettingPath):
    mState(NO_IMAGES_YET), mpORBVocabulary(pVoc), mpFramePublisher(pFramePublisher), mpMapPublisher(pMapPublisher), mpMap(pMap),
    mnLastRelocFrameId(0), mbPublisherStopped(false), mbReseting(false), mbForceRelocalisation(false), mbMotionModel(false)
{
	// Load parameters for each camera from settings files
	vector<float> xi, k1, k2, p1, p2, gamma1, gamma2, u0, v0, fx, fy, cx, cy;
    vector<int> image_width, image_height;
    vector<cv::Mat> K, DistCoef;
    vector<string> camera_name;
    vector<cv::Mat> R, T;

    for(unsigned int i=0; i < (strSettingPath.size()-1); i++)
    {
	    cv::FileStorage fSettings(strSettingPath[i], cv::FileStorage::READ);
        camera_name.push_back(fSettings["camera_name"]);
        image_width.push_back(fSettings["image_width"]);
        image_height.push_back(fSettings["image_height"]);
        fx.push_back(fSettings["fx"]);
        fy.push_back(fSettings["fy"]);
        cx.push_back(fSettings["cx"]);
        cy.push_back(fSettings["cy"]);
        xi.push_back(fSettings["xi"]);
	    k1.push_back(fSettings["k1"]);
	    k2.push_back(fSettings["k2"]);
	    p1.push_back(fSettings["p1"]);
	    p2.push_back(fSettings["p2"]);
	    gamma1.push_back(fSettings["gamma1"]);
	    gamma2.push_back(fSettings["gamma2"]);
	    u0.push_back(fSettings["u0"]);
	    v0.push_back(fSettings["v0"]);

        K.push_back(cv::Mat::eye(3,3,CV_32F));
        K[i].at<float>(0,0) = fx[i];
        K[i].at<float>(1,1) = fy[i];
        K[i].at<float>(0,2) = cx[i];
        K[i].at<float>(1,2) = cy[i];
        //K[i].copyTo(mK);

        DistCoef.push_back(cv::Mat(4,1,CV_32F));
        DistCoef[i].at<float>(0) = k1[i];
        DistCoef[i].at<float>(1) = k2[i];
        DistCoef[i].at<float>(2) = p1[i];
        DistCoef[i].at<float>(3) = p2[i];
        //DistCoef[i].copyTo(mDistCoef);

        // Extrinisic parameters
        R.push_back(cv::Mat(3,3,CV_32F));
        R[i].at<float>(0,0) = fSettings["r11"];
        R[i].at<float>(0,1) = fSettings["r12"];
        R[i].at<float>(0,2) = fSettings["r13"];
        R[i].at<float>(1,0) = fSettings["r21"];
        R[i].at<float>(1,1) = fSettings["r22"];
        R[i].at<float>(1,2) = fSettings["r23"];
        R[i].at<float>(2,0) = fSettings["r31"];
        R[i].at<float>(2,1) = fSettings["r32"];
        R[i].at<float>(2,2) = fSettings["r33"];

        T.push_back(cv::Mat(4,1,CV_32F));
        T[i].at<float>(0) = fSettings["t1"];
        T[i].at<float>(1) = fSettings["t2"];
        T[i].at<float>(2) = fSettings["t3"];


        mK.push_back(K[i]);
        mDistCoef.push_back(DistCoef[i]);
        mT.push_back(T[i]);
        mR.push_back(R[i]);
        mXi.push_back(xi[i]);
        im_width.push_back(image_width[i]);
        im_height.push_back(image_height[i]);

        // Print parameters
        cout << "Parameters Camera[" << i << "]: " << camera_name[i] << endl;
        cout << "- Image_width[" << i << "]: " << im_width[i] << endl;
        cout << "- Image_height[" << i << "]: " << im_height[i] << endl;
        cout << "- fx[" << i << "]: " << fx[i] << endl;
        cout << "- fy[" << i << "]: " << fy[i] << endl;
        cout << "- cx[" << i << "]: " << cx[i] << endl;
        cout << "- cy[" << i << "]: " << cy[i] << endl;
        cout << "- k1[" << i << "]: " << DistCoef[i].at<float>(0) << endl;
        cout << "- k2[" << i << "]: " << DistCoef[i].at<float>(1) << endl;
        cout << "- p1[" << i << "]: " << DistCoef[i].at<float>(2) << endl;
        cout << "- p2[" << i << "]: " << DistCoef[i].at<float>(3) << endl;
        cout << "- gamma1[" << i << "]: " << gamma1[i] << endl;
        cout << "- gamma2[" << i << "]: " << gamma2[i] << endl;
        cout << "- u0[" << i << "]: " << u0[i] << endl;
        cout << "- v0[" << i << "]: " << v0[i] << endl;
        cout << "- mXi[" << i << "]: " << mXi[i] << endl;


        cout << "- mR[" << i << "]: " << mR[i].at<float>(0,0) << " " << mR[i].at<float>(0,1) << " " << mR[i].at<float>(0,2) << endl;
        cout << "        " << mR[i].at<float>(1,0) << " " << mR[i].at<float>(1,1) << " " << mR[i].at<float>(1,2) << endl;
        cout << "        " << mR[i].at<float>(2,0) << " " << mR[i].at<float>(2,1) << " " << mR[i].at<float>(2,2) << endl;
        cout << "- mT[" << i << "]: " << mT[i].at<float>(0) << " " << mT[i].at<float>(1) << " " << mT[i].at<float>(2) << endl;

        cout << "- mK[" << i << "]: " << mK[i].at<float>(0,0) << " " << mK[i].at<float>(0,1) << " " << mK[i].at<float>(0,2) << endl;
        cout << "         " << mK[i].at<float>(1,0) << " " << mK[i].at<float>(1,1) << " " << mK[i].at<float>(1,2) << endl;
        cout << "         " << mK[i].at<float>(2,0) << " " << mK[i].at<float>(2,1) << " " << mK[i].at<float>(2,2) << endl;
        cout << "- mDistCoef[" << i << "]: " << mDistCoef[i].at<float>(0) << " " << mDistCoef[i].at<float>(1) << " "
                                             << mDistCoef[i].at<float>(2) << " " << mDistCoef[i].at<float>(3) << endl;
    }

    // Load camera parameters from settings file
    //K[0].copyTo(mK);
    //DistCoef[0].copyTo(mDistCoef);

    // ORB settings read from last element of the vector strSettingPath
    cv::FileStorage fSettings_ORB(strSettingPath[strSettingPath.size()-1], cv::FileStorage::READ);
    float fps = fSettings_ORB["Camera.fps"];
    if(fps==0)
        fps=30;

    cout << "\n- fps: " << fps << endl;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = 18*fps/30;


    int nRGB = fSettings_ORB["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;


    // Load ORB parameters

    int nFeatures = fSettings_ORB["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings_ORB["ORBextractor.scaleFactor"];
    int nLevels = fSettings_ORB["ORBextractor.nLevels"];
    int fastTh = fSettings_ORB["ORBextractor.fastTh"];
    int Score = fSettings_ORB["ORBextractor.nScoreType"];

    assert(Score==1 || Score==0);

    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,Score,fastTh);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Fast Threshold: " << fastTh << endl;
    if(Score==0)
        cout << "- Score: HARRIS" << endl;
    else
        cout << "- Score: FAST" << endl;


    // ORB extractor for initialization
    // Initialization uses only points from the finest scale level
    mpIniORBextractor = new ORBextractor(nFeatures*2,1.2,8,Score,fastTh);  

    int nMotion = fSettings_ORB["UseMotionModel"];
    mbMotionModel = nMotion;

    if(mbMotionModel)
    {
        mVelocity = cv::Mat::eye(4,4,CV_32F);
        cout << endl << "Motion Model: Enabled" << endl << endl;
    }
    else
        cout << endl << "Motion Model: Disabled (not recommended, change settings UseMotionModel: 1)" << endl << endl;


    tf::Transform tfT;
    tfT.setIdentity();
    mTfBr.sendTransform(tf::StampedTransform(tfT,ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetKeyFrameDatabase(KeyFrameDatabase *pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

void Tracking::Run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/sensor_msgs/imageraw", 1, &Tracking::GrabImage, this);

    ros::spin();
}

void Tracking::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u)
{
    double k1 = mDistCoef[0].at<float>(0);
    double k2 = mDistCoef[0].at<float>(1);
    double p1 = mDistCoef[0].at<float>(2);
    double p2 = mDistCoef[0].at<float>(3);

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
            p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void Tracking::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p)
{
    Eigen::Vector2d p_u, p_d;
    bool m_noDistortion;

    // Project points to the normalised plane
    double z = P(2) + mXi[0] * P.norm();
    p_u << P(0) / z, P(1) / z;

    if ((mDistCoef[0].at<float>(0) == 0.0) &&
        (mDistCoef[0].at<float>(1) == 0.0) &&
        (mDistCoef[0].at<float>(2) == 0.0) &&
        (mDistCoef[0].at<float>(3) == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    if (m_noDistortion)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        Tracking::distortion(p_u, d_u);
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    p << mK[0].at<float>(0,0) * p_d(0) + mK[0].at<float>(0,2),
            mK[0].at<float>(1,1) * p_d(1) + mK[0].at<float>(1,2);
}

void Tracking::initUndistortMap(cv::Mat& map1, cv::Mat& map2)
{
    cv::Size imageSize(im_width[0], im_height[0]);

    cv::Mat mapX = cv::Mat::zeros(imageSize, CV_32F);
    cv::Mat mapY = cv::Mat::zeros(imageSize, CV_32F);

    // cout << "im_width: " << im_width[0] << " im_height: " << im_height[0] << endl;

    cv::Mat mK_inv = mK[0].inv();

    for (int v = 0; v < imageSize.height; ++v)
    {
        for (int u = 0; u < imageSize.width; ++u)
        {
            double mx_u = mK_inv.at<float>(0,0) * u + mK_inv.at<float>(0,2);
            double my_u = mK_inv.at<float>(1,1) * v + mK_inv.at<float>(1,2);

//            cout << "mK_inv:  "<< mK_inv << endl;
//            cout << "mK_inv.at<double>(0,0): " << mK_inv.at<double>(0,0) << endl;
//            cout << "mK_inv.at<floate>(0,0): " << mK_inv.at<float>(0,0) << endl;
//            cout << "mx, my _u: " << mx_u << " " << my_u << endl;

            double xi = mXi[0];
            double d2 = mx_u * mx_u + my_u * my_u;

            Eigen::Vector3d P;
            double P_z = 1.0 + (1.0 - xi * xi) * d2;
            P << mx_u, my_u, 1.0 - xi * (d2 + 1.0) / (xi + sqrt(1.0 + (1.0 - xi * xi) * d2));

          //  if (P_z > 0)
          //      P << mx_u, my_u, 1.0 - xi * (d2 + 1.0) / (xi + sqrt(1.0 + (1.0 - xi * xi) * d2));
          //  else
          //      P << mx_u, my_u, 1.0 - xi * (d2 + 1.0) / (xi);

//            cout << "P: " << P << endl;
//            cout << "xi: " << xi << endl;
//            cout << "sqrt(1.0 + (1.0 - xi * xi) * d2): " << sqrt(1.0 + (1.0 - xi * xi) * d2) << endl;
//            cout << "d2: " << d2 << endl;

            Eigen::Vector2d p;
            spaceToPlane(P, p);

//            cout << "p: " << p << endl;

            mapX.at<float>(v,u) = p(0);
            mapY.at<float>(v,u) = p(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);
}

void Tracking::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{

    cv::Mat im;

    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_ASSERT(cv_ptr->image.channels()==3 || cv_ptr->image.channels()==1);

    if(cv_ptr->image.channels()==3)
    {
        if(mbRGB)
            cvtColor(cv_ptr->image, im, CV_RGB2GRAY);
        else
            cvtColor(cv_ptr->image, im, CV_BGR2GRAY);
    }
    else if(cv_ptr->image.channels()==1)
    {
        cv_ptr->image.copyTo(im);
    }

    // Extract images from stitched image -> assuming 4 images
    int width = im.cols/2;
    int height = im.rows/2;

    cv::Mat img1 = cv::Mat(im, cv::Rect(0,0, width, height));
    cv::Mat img2 = cv::Mat(im, cv::Rect(0, height, width, height));
    cv::Mat img3 = cv::Mat(im, cv::Rect(width, 0, width, height));
    cv::Mat img4 = cv::Mat(im, cv::Rect(width, height, width, height));

    vector<cv::Mat> imgs;
    imgs.push_back(img1);
    imgs.push_back(img2);
    imgs.push_back(img3);
    imgs.push_back(img4);

    cv::Mat mapX;
    cv::Mat mapY;

    cout << "BOOL: INITIALCOMPUTATION: " << Frame::mbInitialComputations << endl;

    if (Frame::mbInitialComputations) // true only for first incoming frame
    {
        Tracking::initUndistortMap(mapX, mapY);

        // Declare what you need
        cv::FileStorage fileX("/home/marius/catkin_3dvision_ws/src/ORB_SLAM/Data/MapX.yaml", cv::FileStorage::WRITE);
        cv::FileStorage fileY("/home/marius/catkin_3dvision_ws/src/ORB_SLAM/Data/MapY.yaml", cv::FileStorage::WRITE);


        // Write to file!
        //fileX << "mapX" << mapX;
        //fileY << "mapY" << mapY;

        cout << "img1.type(): " << img1.type() << endl;
        cout << "img1.channels(): " << img1.channels() << endl;

        cout << "Pos1 " << endl;
        cv::Mat img_new = cv::Mat::zeros(img1.rows, img1.cols, CV_8U);

        cout << "Pos1 --b " << endl;

        cout << "img1.cols: " <<  img1.cols << endl;
        cout << "img1.rows: " <<  img1.rows << endl;
        cout << "img1.size: " << img1.size() << endl;
        cout << "mapX.size: " << mapX.size() << endl;
        cout << "mapY.size: " << mapY.size() << endl;

        // Convert and Write new undistorted image
        for (int u=0; u<img1.cols; u++) {
            //cout << "u: " << u << endl;
            for (int v=0; v<img1.rows; v++) {
                //cout << "v: " << v << endl;
                //cout << "mapX: " << mapX.at<float>(v,u) << " | mapY: " << mapY.at<float>(v,u) << endl;
                int new_v = static_cast<int>(mapX.at<float>(v,u));
                int new_u = static_cast<int>(mapY.at<float>(v,u));
                if (new_u > 0 && new_u < img1.rows && new_v > 0 && new_v < img1.cols)
                {
                    img_new.at<uint8_t>(new_u,new_v) = img1.at<uint8_t>(v, u);
                }
                else
                {
                    //img_new.at<uint8_t>(new_u,new_v) = img1.at<uint8_t>(v, u);
                    // cout << "u: " << u << " | new_u: " << new_u << " | v: " << v << " | new_v: " << new_v << endl;
                }
            }
        }
        cv::imwrite( "/home/marius/catkin_3dvision_ws/src/ORB_SLAM/Data/undist_img.bmp", img_new);
    }

    if(mState==WORKING || mState==LOST)
        mCurrentFrame = Frame(imgs[0],cv_ptr->header.stamp.toSec(),mpORBextractor,mpORBVocabulary,mK[0],mDistCoef[0]);
    else
        mCurrentFrame = Frame(imgs[0],cv_ptr->header.stamp.toSec(),mpIniORBextractor,mpORBVocabulary,mK[0],mDistCoef[0]);

    // Depending on the state of the Tracker we perform different tasks

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    if(mState==NOT_INITIALIZED)
    {
        FirstInitialization();
    }
    else if(mState==INITIALIZING)
    {
        Initialize();
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        if(mState==WORKING && !RelocalisationRequested())
        {
            if(!mbMotionModel || mpMap->KeyFramesInMap()<4 || mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                bOK = TrackPreviousFrame();
            else
            {
                bOK = TrackWithMotionModel();
                if(!bOK)
                    bOK = TrackPreviousFrame();
            }
        }
        else
        {
            bOK = Relocalisation();
        }

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(bOK)
            bOK = TrackLocalMap();

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.mTcw);

            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(size_t i=0; i<mCurrentFrame.mvbOutlier.size();i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }

        if(bOK)
            mState = WORKING;
        else
            mState=LOST;

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                Reset();
                return;
            }
        }

        // Update motion model
        if(mbMotionModel)
        {
            if(bOK && !mLastFrame.mTcw.empty())
            {
                cv::Mat LastRwc = mLastFrame.mTcw.rowRange(0,3).colRange(0,3).t();
                cv::Mat Lasttwc = -LastRwc*mLastFrame.mTcw.rowRange(0,3).col(3);
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                LastRwc.copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                Lasttwc.copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();
        }

        mLastFrame = Frame(mCurrentFrame);
     }       

    // Update drawer
    mpFramePublisher->Update(this);

    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Rwc = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*mCurrentFrame.mTcw.rowRange(0,3).col(3);
        tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                        Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                        Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
        tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

        tf::Transform tfTcw(M,V);

        mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORB_SLAM/World", "ORB_SLAM/Camera"));
    }

}

void Tracking::FirstInitialization()
{
    //We ensure a minimum ORB features to continue, otherwise discard frame
    if(mCurrentFrame.mvKeys.size()>100)
    {
        mInitialFrame = Frame(mCurrentFrame);
        mLastFrame = Frame(mCurrentFrame);
        mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
        for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
            mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

        if(mpInitializer)
            delete mpInitializer;

        mpInitializer =  new Initializer(mCurrentFrame,1.0,200);


        mState = INITIALIZING;
    }
}

void Tracking::Initialize()
{
    // Check if current frame has enough keypoints, otherwise reset initialization process
    if(mCurrentFrame.mvKeys.size()<=100)
    {
        fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
        mState = NOT_INITIALIZED;
        return;
    }    

    // Find correspondences
    ORBmatcher matcher(0.9,true);
    int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

    // Check if there are enough correspondences
    if(nmatches<100)
    {
        mState = NOT_INITIALIZED;
        return;
    }  

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
    {
        for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
        {
            if(mvIniMatches[i]>=0 && !vbTriangulated[i])
            {
                mvIniMatches[i]=-1;
                nmatches--;
            }           
        }

        CreateInitialMap(Rcw,tcw);
    }

}

void Tracking::CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw)
{
    // Set Frame Poses
    mInitialFrame.mTcw = cv::Mat::eye(4,4,CV_32F);
    mCurrentFrame.mTcw = cv::Mat::eye(4,4,CV_32F);
    Rcw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
    tcw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).col(3));

    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;

        //Add to Map
        mpMap->AddMapPoint(pMP);

    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    ROS_INFO("New Map created with %d points",mpMap->MapPointsInMap());

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints()<100)
    {
        ROS_INFO("Wrong initialization, reseting...");
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.mTcw = pKFcur->GetPose().clone();
    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapPublisher->SetCurrentCameraPose(pKFcur->GetPose());

    mState=WORKING;
}


bool Tracking::TrackPreviousFrame()
{
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;

    // Search first points at coarse scale levels to get a rough initial estimate
    int minOctave = 0;
    int maxOctave = mCurrentFrame.mvScaleFactors.size()-1;
    if(mpMap->KeyFramesInMap()>5)
        minOctave = maxOctave/2+1;

    int nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,200,vpMapPointMatches,minOctave);

    // If not enough matches, search again without scale constraint
    if(nmatches<10)
    {
        nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,100,vpMapPointMatches,0);
        if(nmatches<10)
        {
            vpMapPointMatches=vector<MapPoint*>(mCurrentFrame.mvpMapPoints.size(),static_cast<MapPoint*>(NULL));
            nmatches=0;
        }
    }

    mLastFrame.mTcw.copyTo(mCurrentFrame.mTcw);
    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

    // If enough correspondeces, optimize pose and project points from previous frame to search more correspondences
    if(nmatches>=10)
    {
        // Optimize pose with correspondences
        Optimizer::PoseOptimization(&mCurrentFrame);

        for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }

        // Search by projection with the estimated pose
        nmatches += matcher.SearchByProjection(mLastFrame,mCurrentFrame,15,vpMapPointMatches);
    }
    else //Last opportunity
        nmatches = matcher.SearchByProjection(mLastFrame,mCurrentFrame,50,vpMapPointMatches);


    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

    if(nmatches<10)
        return false;

    // Optimize pose again with all correspondences
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
        if(mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]=NULL;
            mCurrentFrame.mvbOutlier[i]=false;
            nmatches--;
        }

    return nmatches>=10;
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;

    // Compute current pose by motion model
    mCurrentFrame.mTcw = mVelocity*mLastFrame.mTcw;

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,15);

    if(nmatches<20)
       return false;

    // Optimize pose with all correspondences
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvpMapPoints.size(); i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }
        }
    }

    return nmatches>=10;
}

bool Tracking::TrackLocalMap()
{
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    // Update Local Map
    UpdateReference();

    // Search Local MapPoints
    SearchReferencePointsInFrustum();

    // Optimize Pose
    mnMatchesInliers = Optimizer::PoseOptimization(&mCurrentFrame);

    // Update MapPoints Statistics
    for(size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); i++)
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // Not insert keyframes if not enough frames from last relocalisation have passed
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mpMap->KeyFramesInMap()>mMaxFrames)
        return false;

    // Reference KeyFrame MapPoints
    int nRefMatches = mpReferenceKF->TrackedMapPoints();

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle;
    // Condition 2: Less than 90% of points than reference keyframe and enough inliers
    const bool c2 = mnMatchesInliers<nRefMatches*0.9 && mnMatchesInliers>15;

    if((c1a||c1b)&&c2)
    {
        // If the mapping accepts keyframes insert, otherwise send a signal to interrupt BA, but not insert yet
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpLocalMapper->InsertKeyFrame(pKF);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchReferencePointsInFrustum()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = NULL;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    mCurrentFrame.UpdatePoseMatrices();

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;        
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }    


    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateReference()
{    
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateReferenceKeyFrames();
    UpdateReferencePoints();
}

void Tracking::UpdateReferencePoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateReferenceKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(size_t i=0, iend=mCurrentFrame.mvpMapPoints.size(); i<iend;i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    int max=0;
    KeyFrame* pKFmax=NULL;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

    }

    mpReferenceKF = pKFmax;
}

bool Tracking::Relocalisation()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalisation is performed when tracking is lost and forced at some stages during loop closing
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs;
    if(!RelocalisationRequested())
        vpCandidateKFs= mpKeyFrameDB->DetectRelocalisationCandidates(&mCurrentFrame);
    else // Forced Relocalisation: Relocate against local window around last keyframe
    {
        boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
        mbForceRelocalisation = false;
        vpCandidateKFs.reserve(10);
        vpCandidateKFs = mpLastKeyFrame->GetBestCovisibilityKeyFrames(9);
        vpCandidateKFs.push_back(mpLastKeyFrame);
    }

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(size_t i=0; i<vpCandidateKFs.size(); i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }        
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(size_t i=0; i<vpCandidateKFs.size(); i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                for(size_t j=0; j<vbInliers.size(); j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(size_t io =0, ioend=mCurrentFrame.mvbOutlier.size(); io<ioend; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=NULL;

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(size_t ip =0, ipend=mCurrentFrame.mvpMapPoints.size(); ip<ipend; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(size_t io =0; io<mCurrentFrame.mvbOutlier.size(); io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {                    
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::ForceRelocalisation()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    mbForceRelocalisation = true;
    mnLastRelocFrameId = mCurrentFrame.mnId;
}

bool Tracking::RelocalisationRequested()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    return mbForceRelocalisation;
}


void Tracking::Reset()
{
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = false;
        mbReseting = true;
    }

    // Wait until publishers are stopped
    ros::Rate r(500);
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(mbPublisherStopped)
                break;
        }
        r.sleep();
    }

    // Reset Local Mapping
    mpLocalMapper->RequestReset();
    // Reset Loop Closing
    mpLoopClosing->RequestReset();
    // Clear BoW Database
    mpKeyFrameDB->clear();
    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NOT_INITIALIZED;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbReseting = false;
    }
}

void Tracking::CheckResetByPublishers()
{
    bool bReseting = false;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        bReseting = mbReseting;
    }

    if(bReseting)
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = true;
    }

    // Hold until reset is finished
    ros::Rate r(500);
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(!mbReseting)
            {
                mbPublisherStopped=false;
                break;
            }
        }
        r.sleep();
    }
}

} //namespace ORB_SLAM

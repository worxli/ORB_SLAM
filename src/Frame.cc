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

#include "Frame.h"
#include "Converter.h"

#include <ros/ros.h>

namespace ORB_SLAM
{
long unsigned int Frame::nNextId=0;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractor(frame.mpORBextractor), cameraFrames(frame.cameraFrames), 
     mTimeStamp(frame.mTimeStamp), mnId(frame.mnId), pluckerLines(frame.pluckerLines), mvpMapPoints(frame.mvpMapPoints),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels), mfScaleFactor(frame.mfScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
     mDescriptors(frame.mDescriptors.clone())
{
    if(!frame.mTcw.empty())
        mTcw = frame.mTcw.clone();
}

Frame::Frame(vector<CameraFrame> cameraFrames, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc)
    :mpORBvocabulary(voc),mpORBextractor(extractor),cameraFrames(cameraFrames),mTimeStamp(timeStamp)
{
    mnId=nNextId++;
    
    // loop all camera frames to extract plucker lines and ORB descriptors
    for(uint i = 0; i<cameraFrames.size(); i++)
    {
        pluckerLines.insert(pluckerLines.end(), cameraFrames[i].pluckerLines.begin(), cameraFrames[i].pluckerLines.end());
        cout << "Add " << cameraFrames[0].pluckerLines.size() << " pluckerlines for frame " << i << " to base frame." << endl;
    }
    
    //Scale Levels Info
    mnScaleLevels = mpORBextractor->GetLevels();
    mfScaleFactor = mpORBextractor->GetScaleFactor();

    mvScaleFactors.resize(mnScaleLevels);
    mvLevelSigma2.resize(mnScaleLevels);
    mvScaleFactors[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<mnScaleLevels; i++)
    {
        mvScaleFactors[i]=mvScaleFactors[i-1]*mfScaleFactor;        
        mvLevelSigma2[i]=mvScaleFactors[i]*mvScaleFactors[i];
    }

    mvInvLevelSigma2.resize(mvLevelSigma2.size());
    for(int i=0; i<mnScaleLevels; i++)
        mvInvLevelSigma2[i]=1/mvLevelSigma2[i];

    // set params to camera frames
    for(uint i = 0; i<cameraFrames.size(); i++)
    {
        cameraFrames[i].SetScaleParams(mnScaleLevels, mvScaleFactors, mvLevelSigma2, mvInvLevelSigma2);
    }
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;

    // set params to camera frames
    for(uint i = 0; i<cameraFrames.size(); i++)
    {
        cameraFrames[i].SetPoseMatrices(mRcw, mtcw, mOw);
    }
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    for(uint i = 0; i<cameraFrames.size(); i++)
    {
	if(cameraFrames[i].isInFrustum(pMP, viewingCosLimit)) 
	    return true;
	return false;
    }
	return false;
}

void Frame::ComputeBoW()
{
    if(cameraFrames[0].mBowVec.empty())
    {
        /* TODO how to do this */
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(cameraFrames[0].mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,cameraFrames[0].mBowVec,cameraFrames[0].mFeatVec,4);
        
    }
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, int minLevel, int maxLevel) const
{

    vector<size_t> vIndices;
    for(uint i = 0; i<cameraFrames.size(); i++)
    {
    	//vIndices.push_back(cameraFrames[i].GetFeaturesInArea(x, y, r, minLevel, maxLevel));
    	vector<size_t> cameraFramevIndices = cameraFrames[i].GetFeaturesInArea(x, y, r, minLevel, maxLevel);
    	vIndices.insert(vIndices.end(), cameraFramevIndices.begin(), cameraFramevIndices.end());
    }
    return vIndices;
}

} //namespace ORB_SLAM

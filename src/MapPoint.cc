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

#include "MapPoint.h"
#include "ORBmatcher.h"
#include "ros/ros.h"

namespace ORB_SLAM
{

long unsigned int MapPoint::nNextId=0;

MapPoint::MapPoint()
{
    mbBad = false;
}

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap, int camera):
    mnFirstKFid(pRefKF->mnId), mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mnBALocalForKF(0),
    mnLoopPointForKF(0), mnCorrectedByKF(0),mnCorrectedReference(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1),
    mbBad(false), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap), camera(camera)
{
    Pos.copyTo(mWorldPos);
    mnId=nNextId++;
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    boost::mutex::scoped_lock lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
     boost::mutex::scoped_lock lock(mMutexFeatures);
     return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mObservations[pKF]=idx;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        cout << "BA Erase 1:" << endl;

        boost::mutex::scoped_lock lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            cout << "BA Erase 2:" << endl;

            mObservations.erase(pKF);
            cout << "BA Erase 3:" << endl;

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;
            cout << "BA Erase 3:" << endl;

            // If only 2 observations or less, discard point
            if(mObservations.size()<=2)
                bBad=true;
            cout << "BA Erase 4:" << endl;

        }
    }

    cout << "BA Erase 5:" << endl;

    if(bBad)
        SetBadFlag();
    cout << "BA Erase 6:" << endl;

}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mObservations.size();
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        cout << "BA Set 1:" << endl;

        boost::mutex::scoped_lock lock1(mMutexFeatures);
        boost::mutex::scoped_lock lock2(mMutexPos);
        cout << "BA Set 2:" << endl;

        mbBad=true;
        obs = mObservations;
        cout << "BA Set 3:" << endl;

        mObservations.clear();
        cout << "BA Set 4:" << endl;

    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        cout << "BA Set loop:" << endl;

        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }
    cout << "BA Set 5:" << endl;


    mpMap->EraseMapPoint(this);
    cout << "BA Set 6:" << endl;

}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    map<KeyFrame*,size_t> obs;
    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        boost::mutex::scoped_lock lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }

    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);

}

bool MapPoint::isBad()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    boost::mutex::scoped_lock lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mnVisible++;
}

void MapPoint::IncreaseFound()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mnFound++;
}

float MapPoint::GetFoundRatio()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->GetDescriptor(mit->second, camera));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        boost::mutex::scoped_lock lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();       
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    cout << "BA Normal 1:" << endl;

    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        boost::mutex::scoped_lock lock2(mMutexPos);
        cout << "BA Normal 2:" << endl;

        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        cout << "BA Normal 3:" << endl;

        Pos = mWorldPos.clone();
        cout << "BA Normal 4:" << endl;

    }
    cout << "BA Normal 5:" << endl;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        cout << "BA Normal 6 loop 1:" << endl;

        KeyFrame* pKF = mit->first;
        cout << "BA Normal 6 loop 2:" << endl;

        cv::Mat Owi = pKF->GetCameraCenter();
        cout << "BA Normal 6 loop 3:" << endl;

        cv::Mat normali = mWorldPos - Owi;
        cout << "BA Normal 6 loop 4:" << endl;

        normal = normal + normali/cv::norm(normali);
        cout << "BA Normal 6 loop 5:" << endl;

        n++;
    }
    cout << "Pos: " << Pos << endl;
    cout << "pRefKF->center: " << pRefKF->GetCameraCenter() << endl;

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    cout << "BA Normal 8:" << endl;

    const float dist = cv::norm(PC);
    cout << "BA Normal 9:" << endl;

    const int level = pRefKF->GetKeyPointScaleLevel(observations[pRefKF], camera);
    cout << "BA Normal 10:" << endl;

    const float scaleFactor = pRefKF->GetScaleFactor();
    cout << "BA Normal 11:" << endl;

    const float levelScaleFactor =  pRefKF->GetScaleFactor(level);
    cout << "BA Normal 12:" << endl;

    const int nLevels = pRefKF->GetScaleLevels();
    cout << "BA Normal 13:" << endl;


    {
        boost::mutex::scoped_lock lock3(mMutexPos);
        mfMinDistance = (1.0f/scaleFactor)*dist / levelScaleFactor;
        mfMaxDistance = scaleFactor*dist * pRefKF->GetScaleFactor(nLevels-1-level);
        mNormalVector = normal/n;
        cout << "BA Normal 14:" << endl;

    }
}

float MapPoint::GetMinDistanceInvariance()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mfMaxDistance;
}

} //namespace ORB_SLAM

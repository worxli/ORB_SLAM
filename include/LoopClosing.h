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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include <boost/thread.hpp>

#include "KeyFrameDatabase.h"

#include "Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"

#include "FramePublisher.h"

namespace ORB_SLAM
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class FramePublisher;


class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    void Run();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();

    void SetFramePublisher(FramePublisher* pFramePub);

protected:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    cv::Mat ComputeE12(KeyFrame *&pKF1, KeyFrame *&pKF2);

    void RefineKP4EasyTriag(cv::KeyPoint &x1, cv::KeyPoint &x2, const cv::Mat &E12, const cv::Mat &K);

    void DoLocalTriangulations(KeyFrame* pKF, vector<bool> &vbMatched);

    void SearchAndFuse(KeyFrameAndPose &CorrectedPosesMap);

    void CorrectLoop();

    void ResetIfRequested();
    bool mbResetRequested;
    boost::mutex mMutexReset;

    Map* mpMap;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;
    std::set<pair<KeyFrame*, KeyFrame*> > msMatchedLoopPairs;//<KF, LoopCandidateKF>

    boost::mutex mMutexLoopQueue;

    std::vector<float> mvfLevelSigmaSquare;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;
    double mScale_cw;

    long unsigned int mLastLoopKFid;
    FramePublisher* mpFramePub;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H

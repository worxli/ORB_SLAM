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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include <ros/ros.h>

#include "Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"

namespace ORB_SLAM
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc):
    mbResetRequested(false), mpMap(pMap), mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mLastLoopKFid(0)
{
    mnCovisibilityConsistencyTh = 3;
    mpMatchedKF = NULL;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{

    ros::Rate r(200);

    while(ros::ok())
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            DetectLoop();

               // Compute similarity transformation [sR|t]
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }

        }

        ResetIfRequested();

        r.sleep();
    }
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLoop()
{
    {
        boost::mutex::scoped_lock lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    DBoW2::BowVector CurrentBowVec = mpCurrentKF->GetBowVector();
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        DBoW2::BowVector BowVec = pKF->GetBowVector();

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframe to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;

        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;

    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }

    mpCurrentKF->SetErase();
    return false;
}

bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    for(int i = 0; i < nInitialCandidates; i++){
        pair<KeyFrame*, KeyFrame*> pairs(mpCurrentKF, mvpEnoughConsistentCandidates[i]);
        msMatchedLoopPairs.insert(pairs);
        cout<<"[LoopClosure] insert new frame pairs" << mpCurrentKF->mnId << " " << mvpEnoughConsistentCandidates[i]->mnId << endl;
    }
    mvpEnoughConsistentCandidates.clear();

    ORBmatcher matcher(0.75,true);
    vector<pair<KeyFrame*, KeyFrame*> > vpMarkedDelete;
    for(set<pair<KeyFrame*, KeyFrame*> >::iterator sit=msMatchedLoopPairs.begin(), send=msMatchedLoopPairs.end(); sit!=send; sit++){
        pair<KeyFrame*, KeyFrame*> pairs = (*sit);
        KeyFrame* pKF_delayedCurrent = pairs.first;
        cout << "[LoopClosing:241] frame pair " << mpCurrentKF->mnId << " "<< pKF_delayedCurrent->mnId << endl;
        if (mpCurrentKF->mnId - pKF_delayedCurrent->mnId < 5) continue;
        // enough future KeyFrame accumulated, check the matching and computeSim3
        KeyFrame* pKF_loopCandidate = pairs.second;
        vpMarkedDelete.push_back(pairs); // delete it from the buffer
        // do feature matching
        vector<MapPoint*> vpMapPointMatches;
        int nmatches = matcher.SearchByBoW(pKF_delayedCurrent,pKF_loopCandidate, vpMapPointMatches);
        cout << "[LoopClosure:277] nmatches " << nmatches << endl;
        if(nmatches<20) continue;

        // if have enough matches, do triangulations
        // Triangulate matched feature points only for efficiency
        vector<bool> vbMatched1; vbMatched1.resize(pKF_delayedCurrent->GetMapPointMatches().size(), false);
        vector<bool> vbMatched2; vbMatched2.resize(pKF_loopCandidate->GetMapPointMatches().size(), false);
        for(unsigned int j = 0; j < vpMapPointMatches.size(); j++){
            if(pKF_delayedCurrent->GetMapPoint(j) == NULL || pKF_delayedCurrent->GetMapPoint(j)->isBad()) continue;
            if(vpMapPointMatches[j] == NULL || vpMapPointMatches[j]->isBad()) continue;

            int idx1 = j;
            int idx2 = vpMapPointMatches[j]->GetIndexInKeyFrame(pKF_loopCandidate);

            vbMatched1[idx1] = true;
            vbMatched2[idx2] = true;
            cout << idx2 << " ";
        }
        cout << endl;
        for(unsigned int j = 0; j < vbMatched1.size(); j++){
            if(vbMatched1[j]) cout << j << " ";
        }
        cout << endl;


        DoLocalTriangulations(pKF_delayedCurrent, vbMatched1);
        cout<< "[LoopClosure:269] 1st triangulation completed" << endl;
        DoLocalTriangulations(pKF_loopCandidate, vbMatched2);
        cout<< "[LoopClosure:271] 2nd triangulation completed" << endl;
        // remove failed triangulated pts
        for(size_t j = 0; j < vpMapPointMatches.size(); j++){
            if(pKF_delayedCurrent->GetMapPoint(j) == NULL || pKF_delayedCurrent->GetMapPoint(j)->isBad()) continue;
            if(vpMapPointMatches[j] == NULL || vpMapPointMatches[j]->isBad()) continue;

            int idx1 = j;
            int idx2 = vpMapPointMatches[j]->GetIndexInKeyFrame(pKF_loopCandidate);

            if(vbMatched1[idx1] && vbMatched2[idx2]) continue;
            vbMatched1[idx1] = false;
            vbMatched2[idx2] = false;
        }
        // Do local BA to refine the map points.
        Optimizer::LocalBundleAdjustment(pKF_delayedCurrent, vbMatched1);
        Optimizer::LocalBundleAdjustment(pKF_loopCandidate, vbMatched2);

        // DEBUG... Draw feature matching results
            vector<cv::KeyPoint> matchedKeyPt1, matchedKeyPt2;
            for(unsigned int j = 0; j < vpMapPointMatches.size(); j++){
                if(pKF_delayedCurrent->GetMapPoint(j) == NULL || pKF_delayedCurrent->GetMapPoint(j)->isBad()) continue;
                if(vpMapPointMatches[j] == NULL || vpMapPointMatches[j]->isBad()) continue;

                int idx1 = j;
                int idx2 = vpMapPointMatches[j]->GetIndexInKeyFrame(pKF_loopCandidate);
                if (idx2==-1 || !(vbMatched1[idx1] && vbMatched2[idx2])) continue;
                matchedKeyPt1.push_back(pKF_delayedCurrent->GetKeyPoint(idx1));
                matchedKeyPt2.push_back(pKF_loopCandidate->GetKeyPoint(idx2));

                cout << j << "th mapPoint " << pKF_delayedCurrent->GetMapPoint(idx1)->GetWorldPos() << "; ";
                    cout << pKF_loopCandidate->GetMapPoint(idx2)->GetWorldPos() << endl;
            }
            cout<< endl;
            if(matchedKeyPt2.size() != 0){
                mpFramePub->DrawFeatureMatches(pKF_delayedCurrent, pKF_loopCandidate, matchedKeyPt1, matchedKeyPt2);
            }
    }
    for(size_t i = 0; i < vpMarkedDelete.size(); i++){
        msMatchedLoopPairs.erase(vpMarkedDelete[i]);
    }
    return false;
}









//    // We compute first ORB matches for each candidate
//    // If enough matches are found, we setup a Sim3Solver
//    ORBmatcher matcher(0.75,true);

//    vector<Sim3Solver*> vpSim3Solvers;
//    vpSim3Solvers.resize(nInitialCandidates);

//    vector<vector<MapPoint*> > vvpMapPointMatches;
//    vvpMapPointMatches.resize(nInitialCandidates);

//    vector<bool> vbDiscarded;
//    vbDiscarded.resize(nInitialCandidates);

//    int nCandidates=0; //candidates with enough matches

//    for(int i=0; i<nInitialCandidates; i++)
//    {
//        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

//        // avoid that local mapping erase it while it is being processed in this thread
//        pKF->SetNotErase();

//        if(pKF->isBad())
//        {
//            vbDiscarded[i] = true;
//            continue;
//        }

//        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);
//        cout << "[LoopClosure:277] nmatches " << nmatches << endl;

//        if(nmatches<20)
//        {
//            vbDiscarded[i] = true;
//            continue;
//        }
//        else
//        {
//            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i]);
//            pSolver->SetRansacParameters(0.99,20,300);
//            vpSim3Solvers[i] = pSolver;
//        }

//        nCandidates++;
//    }

//    bool bMatch = false;
//    cout << "[LoopClosure:287] nCandidates " << nCandidates << endl;

//    // Perform alternatively RANSAC iterations for each candidate
//    // until one is succesful or all fail
//    while(nCandidates>0 && !bMatch)
//    {
//        for(int i=0; i<nInitialCandidates; i++)
//        {
//            if(vbDiscarded[i])
//                continue;
//            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];
//            cout << "[LoopClosure:288] matched KF id: " << pKF->mnId << endl;
//            // Triangulate matched feature points only for efficiency
//            vector<bool> vbMatched1; vbMatched1.resize(mpCurrentKF->GetMapPointMatches().size(), false);
//            vector<bool> vbMatched2; vbMatched2.resize(pKF->GetMapPointMatches().size(), false);
//            vector<MapPoint*>vpMapPointMatches = vvpMapPointMatches[i];
//            for(unsigned int j = 0; j < vpMapPointMatches.size(); j++){
//                if(mpCurrentKF->GetMapPoint(j) == NULL || mpCurrentKF->GetMapPoint(j)->isBad()) continue;
//                if(vpMapPointMatches[j] == NULL || vpMapPointMatches[j]->isBad()) continue;

//                int idx1 = j;
//                int idx2 = vpMapPointMatches[j]->GetIndexInKeyFrame(pKF);

//                vbMatched1[idx1] = true;
//                vbMatched2[idx2] = true;
//            }

//              DoLocalTriangulations(mpCurrentKF, vbMatched1);

////            DoLocalTriangulations(pKF, vbMatched2);

////            Optimizer::LocalBundleAdjustment(mpCurrentKF);
////            Optimizer::LocalBundleAdjustment(pKF);

//            // DEBUG... Draw feature matching results
//                vector<cv::KeyPoint> matchedKeyPt1, matchedKeyPt2;
//                vector<MapPoint*>vpMapPoints = vvpMapPointMatches[i];

//                for(unsigned int j = 0; j < vpMapPoints.size(); j++){
//                    if(mpCurrentKF->GetMapPoint(j)==NULL || mpCurrentKF->GetMapPoint(j)->isBad()) continue;
////                    if(vpMapPoints[j] == NULL || vpMapPoints[j]->isBad()) continue;

//                    int idx1 = j;
////                    int idx2 = vpMapPoints[j]->GetIndexInKeyFrame(pKF);
//                    matchedKeyPt1.push_back(mpCurrentKF->GetKeyPoint(idx1));
////                    matchedKeyPt2.push_back(pKF->GetKeyPoint(idx2));

//                    cout << j << "th mapPoint " << mpCurrentKF->GetMapPoint(idx1)->GetWorldPos() << "; " << endl;
////                    cout << pKF->GetMapPoint(idx2)->GetWorldPos() << endl;
//                }
//                cout<< endl;
//                if(matchedKeyPt2.size() != 0){
//                    mpFramePub->DrawFeatureMatches(mpCurrentKF, pKF, matchedKeyPt1, matchedKeyPt2);
//                }
//            //DO Local BA to optimize the map point locations for similarity transformation matrix computation


//            // Perform 5 Ransac Iterations
//            vector<bool> vbInliers;
//            int nInliers;
//            bool bNoMore;

//            Sim3Solver* pSolver = vpSim3Solvers[i];
//            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

//            // If Ransac reachs max. iterations discard keyframe
//            if(bNoMore)
//            {
//                vbDiscarded[i]=true;
//                nCandidates--;
//            }
//nCandidates--;
//continue;
//            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
//            if(!Scm.empty())
//            {
//                cout << "[LoopClosure:349] Scm is not empty"<<endl;
//                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
//                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
//                {
//                    if(vbInliers[j])
//                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
//                }

//                cv::Mat R = pSolver->GetEstimatedRotation();
//                cv::Mat t = pSolver->GetEstimatedTranslation();
//                const float s = pSolver->GetEstimatedScale();
//                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);


//                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
//                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10);

//                // If optimization is succesful stop ransacs and continue
//                cout << "[LoopClosing:331] nInliers " << nInliers << endl;
//                if(nInliers>=10)
//                {
//                    bMatch = true;
//                    mpMatchedKF = pKF;
//                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
//                    mg2oScw = gScm*gSmw;
//                    mScw = Converter::toCvMat(mg2oScw);

//                    mvpCurrentMatchedPoints = vpMapPointMatches;
//                    break;
//                }
//            }
//        }
//    }

//    if(!bMatch)
//    {
//        for(int i=0; i<nInitialCandidates; i++)
//             mvpEnoughConsistentCandidates[i]->SetErase();
//        mpCurrentKF->SetErase();
//        return false;
//    }

//    // Retrieve MapPoints seen in Loop Keyframe and neighbors
//    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
//    vpLoopConnectedKFs.push_back(mpMatchedKF);
//    mvpLoopMapPoints.clear();
//    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
//    {
//        KeyFrame* pKF = *vit;
//        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
//        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
//        {
//            MapPoint* pMP = vpMapPoints[i];
//            if(pMP)
//            {
//                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
//                {
//                    mvpLoopMapPoints.push_back(pMP);
//                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
//                }
//            }
//        }
//    }

//    // Find more matches projecting with the computed Sim3
//    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

//    // If enough matches accept Loop
//    int nTotalMatches = 0;
//    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
//    {
//        if(mvpCurrentMatchedPoints[i])
//            nTotalMatches++;
//    }

//    if(nTotalMatches>=40)
//    {
//        for(int i=0; i<nInitialCandidates; i++)
//            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
//                mvpEnoughConsistentCandidates[i]->SetErase();
//        return true;
//    }
//    else
//    {
//        for(int i=0; i<nInitialCandidates; i++)
//            mvpEnoughConsistentCandidates[i]->SetErase();
//        mpCurrentKF->SetErase();
//        return false;
//    }

//}

void LoopClosing::DoLocalTriangulations(KeyFrame *pKF, vector<bool> &vbMatched)
{
    //Triangulate only matched map points

    const float fx = pKF->fx;
    const float fy = pKF->fy;
    const float cx = pKF->cx;
    const float cy = pKF->cy;
    const float invfx = 1.0f/fx;
    const float invfy = 1.0f/fy;

    vector<MapPoint*> vMapPoints = pKF->GetMapPointMatches();
    for(size_t i = 0; i < vMapPoints.size(); i++){
        MapPoint* pMP = vMapPoints[i];
        if(!vbMatched[i]) continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();
        // Do conversion for efficient computations
        vector<KeyFrame*> vKeyFrames;
        vector<size_t> vFeatureIndex;
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            int idx = mit->second; // feature index in current keyFrame
            vKeyFrames.push_back(pKFi);
            vFeatureIndex.push_back(idx);
        }
        // Check the best parallax score and find the frames corresponding to the smallest parallax score
        float minParallaxRays = 1.0;
        int index_KF1, index_KF2, index_Feature1, index_Feature2;
        for (size_t j = 0 ; j < vKeyFrames.size(); j++){
            for(size_t k = j+1; k < vKeyFrames.size(); k++){
                KeyFrame* pKF1 = vKeyFrames[j];
                KeyFrame* pKF2 = vKeyFrames[k];
                int idx1 = vFeatureIndex[j];
                int idx2 = vFeatureIndex[k];

                const cv::KeyPoint &kp1 = pKF1->GetKeyPointUn(idx1);
                const cv::KeyPoint &kp2 = pKF2->GetKeyPointUn(idx2);

                // Check parallax between rays
                cv::Mat Rcw1 = pKF1->GetRotation();
                cv::Mat Rwc1 = Rcw1.t();

                cv::Mat Rcw2 = pKF2->GetRotation();
                cv::Mat Rwc2 = Rcw2.t();

                cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx)*invfx, (kp1.pt.y-cy)*invfy, 1.0 );
                cv::Mat ray1 = Rwc1*xn1;
                cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx)*invfx, (kp2.pt.y-cy)*invfy, 1.0 );
                cv::Mat ray2 = Rwc2*xn2;
                const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

                if (cosParallaxRays < minParallaxRays){
                    minParallaxRays = cosParallaxRays;
                    index_KF1 = j;
                    index_KF2 = k;
                    index_Feature1 = idx1;
                    index_Feature2 = idx2;
                }
            }
        }
        if (minParallaxRays > 0.9998) {
            vbMatched[i] = false;
            continue;
        }
//        cout << "[LoopClosing:522] minParallaxRays " << minParallaxRays << endl;
        // Do triangulation
        KeyFrame* pKF1 = vKeyFrames[index_KF1];
        KeyFrame* pKF2 = vKeyFrames[index_KF2];
        const cv::KeyPoint &kp1 = pKF1->GetKeyPointUn(index_Feature1);
        const cv::KeyPoint &kp2 = pKF2->GetKeyPointUn(index_Feature2);

        cv::Mat Rcw1 = pKF1->GetRotation();
        cv::Mat tcw1 = pKF1->GetTranslation();
        cv::Mat Tcw1(3,4,CV_32F);
        Rcw1.copyTo(Tcw1.colRange(0,3));
        tcw1.copyTo(Tcw1.col(3));
        cv::Mat Ow1 = pKF1->GetCameraCenter();
        const float ratioFactor = 1.5f*pKF1->GetScaleFactor();

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));
        cv::Mat Ow2 = pKF2->GetCameraCenter();

        cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx)*invfx, (kp1.pt.y-cy)*invfy, 1.0 );
        cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx)*invfx, (kp2.pt.y-cy)*invfy, 1.0 );

        // Linear Triangulation Method
        cv::Mat A(4,4,CV_32F);
        A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
        A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
        A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
        A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

        cv::Mat w,u,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

        cv::Mat x3D = vt.row(3).t();

        if(x3D.at<float>(3)==0){
            vbMatched[i] = false;
            continue;
        }

        // Euclidean coordinates
        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
        cv::Mat x3Dt = x3D.t();

        //Check triangulation in front of cameras
        float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
        if(z1<=0){
            vbMatched[i] = false;
            continue;
        }

        float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
        if(z2<=0){
            vbMatched[i] = false;
            continue;
        }

        //Check reprojection error in first keyframe
        float sigmaSquare1 = pKF1->GetSigma2(kp1.octave);
        float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
        float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
        float invz1 = 1.0/z1;
        float u1 = fx*x1*invz1+cx;
        float v1 = fy*y1*invz1+cy;
        float errX1 = u1 - kp1.pt.x;
        float errY1 = v1 - kp1.pt.y;
        if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1){
            vbMatched[i] = false;
            continue;
        }

        //Check reprojection error in second keyframe
        float sigmaSquare2 = pKF2->GetSigma2(kp2.octave);
        float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
        float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
        float invz2 = 1.0/z2;
        float u2 = fx*x2*invz2+cx;
        float v2 = fy*y2*invz2+cy;
        float errX2 = u2 - kp2.pt.x;
        float errY2 = v2 - kp2.pt.y;
        if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2){
            vbMatched[i] = false;
            continue;
        }
        //Check scale consistency
        cv::Mat normal1 = x3D-Ow1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = x3D-Ow2;
        float dist2 = cv::norm(normal2);

        if(dist1==0 || dist2==0){
            vbMatched[i] = false;
            continue;
        }

        float ratioDist = dist1/dist2;
        float ratioOctave = pKF1->GetScaleFactor(kp1.octave)/pKF2->GetScaleFactor(kp2.octave);
        if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor){
            vbMatched[i] = false;
            continue;
        }

        // Triangulation is succesfull
//        cout << "[LoopClosing:634] "<< i << "th pts, x3D: " << x3D << endl;
        pMP->SetWorldPos(x3D);
    }
}

void LoopClosing::CorrectLoop()
{
    double t_begin = ros::Time::now().toSec();
    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // Wait until Local Mapping has effectively stopped
    ros::Rate r(1e4);
    while(ros::ok() && !mpLocalMapper->isStopped())
    {
        r.sleep();
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        cv::Mat Tiw = pKFi->GetPose();

        if(pKFi!=mpCurrentKF)
        {            
            cv::Mat Tic = Tiw*Twc;
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
            //Pose corrected with the Sim3 of the loop closure
            CorrectedSim3[pKFi]=g2oCorrectedSiw;
        }

        cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
        cv::Mat tiw = Tiw.rowRange(0,3).col(3);
        g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
        //Pose without correction
        NonCorrectedSim3[pKFi]=g2oSiw;
    }

    // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
    for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
    {
        KeyFrame* pKFi = mit->first;
        g2o::Sim3 g2oCorrectedSiw = mit->second;
        g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

        g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

        vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
        for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
        {
            MapPoint* pMPi = vpMPsi[iMP];
            if(!pMPi)
                continue;
            if(pMPi->isBad())
                continue;
            if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                continue;

            // Project with non-corrected pose and project back with corrected pose
            cv::Mat P3Dw = pMPi->GetWorldPos();
            Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
            Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMPi->SetWorldPos(cvCorrectedP3Dw);
            pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
            pMPi->mnCorrectedReference = pKFi->mnId;
            pMPi->UpdateNormalAndDepth();
        }

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(correctedTiw);

        // Make sure connections are updated
        pKFi->UpdateConnections();
    }    

    // Start Loop Fusion
    // Update matched map points and replace if duplicated
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
        {
            MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
            MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
            if(pCurMP)
                pCurMP->Replace(pLoopMP);
            else
            {
                mpCurrentKF->AddMapPoint(pLoopMP,i);
                pLoopMP->AddObservation(mpCurrentKF,i);
                pLoopMP->ComputeDistinctiveDescriptors();
            }
        }
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    mpTracker->ForceRelocalisation();

    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF,  mg2oScw, NonCorrectedSim3, CorrectedSim3, LoopConnections);

    //Add edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    cout << "[time] LoopClosing run closeLoop " << ros::Time::now() << " " << ros::Time::now().toSec() - t_begin << " secs"<<endl;


    ROS_INFO("Loop Closed!");

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    mpMap->SetFlagAfterBA();

    mLastLoopKFid = mpCurrentKF->mnId;
}

void LoopClosing::SearchAndFuse(KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4);
    }
}


void LoopClosing::RequestReset()
{
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbResetRequested = true;
    }

    ros::Rate r(500);
    while(ros::ok())
    {
        {
        boost::mutex::scoped_lock lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        r.sleep();
    }
}

void LoopClosing::SetFramePublisher(FramePublisher *pFramePub)
{
        mpFramePub = pFramePub;
}

void LoopClosing::ResetIfRequested()
{
    boost::mutex::scoped_lock lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

} //namespace ORB_SLAM

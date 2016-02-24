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
        cout<<"[LoopClosing:233] insert new frame pairs " << mpCurrentKF->mnId << " " << mvpEnoughConsistentCandidates[i]->mnId << endl;
    }
    mvpEnoughConsistentCandidates.clear();

    ORBmatcher matcher(0.75,true);
    vector<pair<KeyFrame*, KeyFrame*> > vpMarkedDelete;
    for(set<pair<KeyFrame*, KeyFrame*> >::iterator sit=msMatchedLoopPairs.begin(), send=msMatchedLoopPairs.end(); sit!=send; sit++){
        pair<KeyFrame*, KeyFrame*> pairs = (*sit);
        KeyFrame* pKF_delayedCurrent = pairs.first;
//        cout << "[LoopClosing:241] frame pair " << mpCurrentKF->mnId << " "<< pKF_delayedCurrent->mnId << endl;
        if (mpCurrentKF->mnId - pKF_delayedCurrent->mnId < 10) continue;
        // enough future KeyFrame accumulated, check the matching and computeSim3
        KeyFrame* pKF_loopCandidate = pairs.second;
        vpMarkedDelete.push_back(pairs); // delete it from the buffer
        // do feature matching
        vector<MapPoint*> vpMapPointMatches;
        int nmatches = matcher.SearchByBoW(pKF_delayedCurrent,pKF_loopCandidate, vpMapPointMatches);
        cout << "[LoopClosure:277] nmatches " << nmatches << endl;
        if(nmatches<10) continue;

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
        }

        DoLocalTriangulations(pKF_delayedCurrent, vbMatched1);
        DoLocalTriangulations(pKF_loopCandidate, vbMatched2);

//        // remove failed triangulated pts
//        for(size_t j = 0; j < vpMapPointMatches.size(); j++){
//            if(pKF_delayedCurrent->GetMapPoint(j) == NULL || pKF_delayedCurrent->GetMapPoint(j)->isBad()) continue;
//            if(vpMapPointMatches[j] == NULL || vpMapPointMatches[j]->isBad()) continue;

//            int idx1 = j;
//            int idx2 = vpMapPointMatches[j]->GetIndexInKeyFrame(pKF_loopCandidate);

//            if(vbMatched1[idx1] && vbMatched2[idx2]) continue;
//            vbMatched1[idx1] = false;
//            vbMatched2[idx2] = false;
//        }
        // Do local BA to refine the map points.
//        Optimizer::LocalBundleAdjustment(pKF_delayedCurrent, vbMatched1);
//        Optimizer::LocalBundleAdjustment(pKF_loopCandidate, vbMatched2);

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

            cv::Mat pos1 = pKF_delayedCurrent->GetMapPoint(idx1)->GetWorldPos();
            cv::Mat pos2 = pKF_loopCandidate->GetMapPoint(idx2)->GetWorldPos();
            cout << j << "th mapPoint " << pos1 << "; " << pos2 << ";; " << cv::norm(pos1) << " " << cv::norm(pos2) << endl;
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

    cv::Mat K = pKF->GetCalibrationMatrix();
int counter = 0;
    vector<MapPoint*> vMapPoints = pKF->GetMapPointMatches();
    for(size_t i = 0; i < vMapPoints.size(); i++){
        MapPoint* pMP = vMapPoints[i];
        if(!vbMatched[i]) continue;
if (counter > 0){
    vbMatched[i] = false;
    continue;
}
counter++;
        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        std::vector<Matrix3x4d> proj_matrices;
        std::vector<Eigen::Vector2d> points;

        //DEBUG
        std::vector<Matrix3x4d> proj_matrix_relative;
        cv::Mat R_ref(3,3,CV_32F); R_ref = pKF->GetRotation();
        cv::Mat t_ref(3,1,CV_32F); t_ref = pKF->GetTranslation();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            int idx = mit->second; // feature index in current keyFrame
            cv::KeyPoint kp = pKFi->GetKeyPointUn(idx);
            // normalize the kp
            cv::Mat xy(3,1,CV_32F);
            xy.at<float>(0,0) = kp.pt.x;
            xy.at<float>(1,0) = 240 - kp.pt.y;
            xy.at<float>(2,0) = 1.0;
            xy = K.inv()*xy; // convert to camera frame

//            cout << "DEBUG [LoopCosing:563] xy " << endl << xy << endl;

            Eigen::Vector2d pt_v2d; pt_v2d << xy.at<float>(0), xy.at<float>(1);
            points.push_back(pt_v2d);
            //
            cv::Mat R = pKFi->GetRotation();
            cv::Mat t = pKFi->GetTranslation();

            Eigen::Matrix<double, 3, 3> R_eigen = Converter::toMatrix3d(R);
            Eigen::Matrix<double, 3, 1> t_eigen = Converter::toVector3d(t);
            Matrix3x4d proj;
            proj << R_eigen, t_eigen;
            proj_matrices.push_back(proj);
//            cout << "DEBUG [LoopClosing:588] proj " << endl << proj << endl;

            //DEBUG
            cv::Mat R12 = R*R_ref.inv();
            cv::Mat t12 = -R12*(-R_ref*R.t()*t+t_ref);
            Matrix3x4d proj_relative;
            proj_relative << Converter::toMatrix3d(R12), Converter::toMatrix3d(t12);
            proj_matrix_relative.push_back(proj_relative);
            //DEBUG
//            cout << "DEBUG [LoopClosing:597] R12 " << endl << R12 << endl;
//            cout << "DEBUG [LoopClosing:598] t12 " << endl << t12.t() << endl;
//            cout << "DEBUG [LoopClosing:575] proj_relative " << endl << proj_relative << endl;
        }
        Eigen::Vector3d x3D = TriangulateMultiViewPoint(proj_matrices, points);

        cv::Mat x3DMat(4,1,CV_32F); x3DMat.at<float>(3) = 1.0;
        Converter::toCvMat(x3D).copyTo(x3DMat.rowRange(0,3));
        cv::Mat poseMat = pKF->GetPose()*x3DMat;
        cout << "DEBUG [LoopClosing:584] x3D " << endl << poseMat.rowRange(0,3) << " " << cv::norm(poseMat.rowRange(0,3)) <<endl;

        Eigen::Vector3d x3D_cam1Frame = TriangulateMultiViewPoint(proj_matrix_relative, points);
        cv::Mat poseRelativeMat = Converter::toCvMat(x3D_cam1Frame);
        cout << "DEBUG [LoopClosing:604] x3D_cam1Frame: "<< endl << poseRelativeMat.t() << " " << cv::norm(poseRelativeMat) << endl << endl;
        pMP->SetWorldPos(Converter::toCvMat(x3D));
    }
}

Eigen::Vector3d LoopClosing::TriangulateMultiViewPoint(
    const std::vector<Matrix3x4d>& proj_matrices,
    const std::vector<Eigen::Vector2d>& points) {

  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

  for (size_t i = 0; i < points.size(); i++) {
    const Eigen::Vector3d point = points[i].homogeneous().normalized();
    const Matrix3x4d term =
        proj_matrices[i] - point * point.transpose() * proj_matrices[i];
    A += term.transpose() * term;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(A);

  return eigen_solver.eigenvectors().col(0).hnormalized();
}

void LoopClosing::RefineKP4EasyTriag(cv::KeyPoint& x1, cv::KeyPoint& x2, const cv::Mat &E12, const cv::Mat &K){
    // Algorithm refer to "Triangulation Made Easy" CVPR 2010
    // Incorrect with the kp locations....TODO: correct it/
    // normalize x1 and x2
    //TODO : Delete cout
//    cout << "[LoopClosing:728] K = " << K << endl;
    cv::Mat _x1 = cv::Mat::eye(3,1,CV_32F);
    _x1.at<float>(0) = x1.pt.x;
    _x1.at<float>(1) = x1.pt.y;
    _x1.at<float>(2) = 1.0;
//    cout << "[LoopClosing:733] _x1 = " << _x1 << endl;
    _x1 = K.inv()*_x1;

    cv::Mat _x2 = cv::Mat::eye(3,1,CV_32F);
    _x2.at<float>(0) = x2.pt.x;
    _x2.at<float>(1) = x2.pt.y;
    _x2.at<float>(2) = 1.0;
    _x2 = K.inv()*_x2;
//    cout << "[Loopclosing:739] _x1 = " << _x1 << "; _x2 = " << _x2 << endl;
    cv::Mat S = cv::Mat::eye(2,3,CV_32F);
//    cout << "[LoopClosing:741] S = " << S << endl;
    cv::Mat n = S*E12*_x2; // 2 x 1
    cv::Mat E_delta = S*E12*S.t();
    cv::Mat n_prime = S*E12.t()*_x1;
    cv::Mat a_Mat = n.t()*E_delta*n_prime; float a = a_Mat.at<float>(0);
    cv::Mat b_Mat = 0.5*(n.t()*n + n_prime.t()*n_prime); float b = b_Mat.at<float>(0);
    cv::Mat c_Mat = _x1.t()*E12*_x2; float c = c_Mat.at<float>(0);
    float d = sqrt(b*b-a*c);
    float lamda = (c/(b+d));
    cv::Mat delta_x1 = lamda*n;
    cv::Mat delta_x2 = lamda*n_prime;
//    cout << "[LoopClosing:752] delta_x12 " << delta_x1 << " " << delta_x2 << endl;
    n = n - E_delta*delta_x2;
    n_prime = n_prime - E_delta.t()*delta_x1;
//    cout << "[LoopClosing:757] delta_x12 " << delta_x1 << " " << delta_x2 << endl;
    cv::Mat coeff1_Mat = (delta_x1.t()*n)/(n.t()*n); float coeff1 = coeff1_Mat.at<float>(0);
    delta_x1 = (coeff1)*n;

    cv::Mat coeff2_Mat = (delta_x2.t()*n_prime)/(n_prime.t()*n_prime); float coeff2 = coeff2_Mat.at<float>(0);
    delta_x2 = (coeff2)*n_prime;

//    cout << "[LoopClosing:761] delta_x12 " << delta_x1 << " " << delta_x2 << endl;
    _x1 -= S.t()*delta_x1;
    _x2 -= S.t()*delta_x2;

    _x1 = K*_x1;
    _x2 = K*_x2;

    x1.pt.x = _x1.at<float>(0);
    x1.pt.y = _x1.at<float>(1);

    x2.pt.x = _x2.at<float>(0);
    x2.pt.y = _x2.at<float>(1);
//    cout << "[LoopClosing:770] " << x1.pt << endl;
//    cout << "[LoopClosing:771] " << x2.pt << endl;
}

cv::Mat LoopClosing::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

cv::Mat LoopClosing::ComputeE12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    return t12x*R12;
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

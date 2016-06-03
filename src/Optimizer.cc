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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "Thirdparty/g2o/g2o/types/sba/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/dense/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

namespace ORB_SLAM
{

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag);
}


void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP, int nIterations, bool* pbStopFlag)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // SET KEYFRAME VERTICES
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }


    const float thHuber = sqrt(5.991);

    // SET MAP POINT VERTICES
    for(size_t i=0, iend=vpMP.size(); i<iend;i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //SET EDGES
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            if(pKF->isBad())
                continue;
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pKF->GetKeyPointUn(mit->second);
            obs << kpUn.pt.x, kpUn.pt.y;

            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
            e->setMeasurement(obs);
            float invSigma2 = pKF->GetInvSigma2(kpUn.octave);
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber);

            e->fx = pKF->fx;
            e->fy = pKF->fy;
            e->cx = pKF->cx;
            e->cy = pKF->cy;

            optimizer.addEdge(e);
        }
    }

    // Optimize!

    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(size_t i=0, iend=vpMP.size(); i<iend;i++)
    {
        MapPoint* pMP = vpMP[i];
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

}

int Optimizer::PoseOptimization(Frame *pFrame)
{    
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    optimizer.setVerbose(false);

    int nInitialCorrespondences=0;

    // SET FRAME VERTEX
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // SET MAP POINT VERTICES
    vector<g2o::EdgeSE3ProjectXYZ*> vpEdges;
    vector<g2o::VertexSBAPointXYZ*> vVertices;
    vector<float> vInvSigmas2;
    vector<size_t> vnIndexEdge;

    const int N = pFrame->mvpMapPoints.size();
    vpEdges.reserve(N);
    vVertices.reserve(N);
    vInvSigmas2.reserve(N);
    vnIndexEdge.reserve(N);

    const float delta = sqrt(5.991);

    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            vPoint->setId(i+1);
            vPoint->setFixed(true);
            optimizer.addVertex(vPoint);
            vVertices.push_back(vPoint);

            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            //SET EDGE
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(i+1)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(delta);

            e->fx = pFrame->fx;
            e->fy = pFrame->fy;
            e->cx = pFrame->cx;
            e->cy = pFrame->cy;

            e->setLevel(0);

            optimizer.addEdge(e);

            vpEdges.push_back(e);
            vInvSigmas2.push_back(invSigma2);
            vnIndexEdge.push_back(i);
        }

    }

    // We perform 4 optimizations, decreasing the inlier region
    // From second to final optimization we include only inliers in the optimization
    // At the end of each optimization we check which points are inliers
    const float chi2[4]={9.210,7.378,5.991,5.991};
    const int its[4]={10,10,7,5};

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdges.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];

            const size_t idx = vnIndexEdge[i];

            if(pFrame->mvbOutlier[idx])
                e->computeError();

            if(e->chi2()>chi2[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else if(e->chi2()<=chi2[it])
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }
        }

        if(optimizer.edges().size()<10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pose.copyTo(pFrame->mTcw);

    return nInitialCorrespondences-nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // SET LOCAL KEYFRAME VERTICES
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // SET FIXED KEYFRAME VERTICES
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // SET MAP POINT VERTICES
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdges;
    vpEdges.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKF;
    vpEdgeKF.reserve(nExpectedSize);

    vector<float> vSigmas2;
    vSigmas2.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdge;
    vpMapPointEdge.reserve(nExpectedSize);

    const float thHuber = sqrt(5.991);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //SET EDGES
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {
                Eigen::Matrix<double,2,1> obs;
                cv::KeyPoint kpUn = pKFi->GetKeyPointUn(mit->second);
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                e->setMeasurement(obs);
                float sigma2 = pKFi->GetSigma2(kpUn.octave);
                float invSigma2 = pKFi->GetInvSigma2(kpUn.octave);
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber);

                e->fx = pKFi->fx;
                e->fy = pKFi->fy;
                e->cx = pKFi->cx;
                e->cy = pKFi->cy;

                optimizer.addEdge(e);
                vpEdges.push_back(e);
                vpEdgeKF.push_back(pKFi);
                vSigmas2.push_back(sigma2);
                vpMapPointEdge.push_back(pMP);
            }
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inlier observations
    for(size_t i=0, iend=vpEdges.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];
        MapPoint* pMP = vpMapPointEdge[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKF[i];
            pKFi->EraseMapPointMatch(pMP);
            pMP->EraseObservation(pKFi);

            optimizer.removeEdge(e);
            vpEdges[i]=NULL;
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }
    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // Check inlier observations
    for(size_t i=0, iend=vpEdges.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];

        if(!e)
            continue;

        MapPoint* pMP = vpMapPointEdge[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKF = vpEdgeKF[i];
            pKF->EraseMapPointMatch(pMP->GetIndexInKeyFrame(pKF));
            pMP->EraseObservation(pKF);
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}

void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, g2o::Sim3 &Scurw,
                                       LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       map<KeyFrame *, set<KeyFrame *> > &LoopConnections)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverCholmod<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    unsigned int nMaxKFid = pMap->GetMaxKFid();


    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;

    // SET KEYFRAME VERTICES
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        int nIDi = pKF->mnId;      

        if(CorrectedSim3.count(pKF))
        {
            vScw[nIDi] = CorrectedSim3[pKF];
            VSim3->setEstimate(CorrectedSim3[pKF]);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }


    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // SET LOOP EDGES
    for(map<KeyFrame *, set<KeyFrame *> >::iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        set<KeyFrame*> &spConnections = mit->second;
        g2o::Sim3 Siw = vScw[nIDi];
        g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            g2o::Sim3 Sjw = vScw[nIDj];
            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // SET NORMAL EDGES
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;
        if(NonCorrectedSim3.count(pKF))
            Swi = NonCorrectedSim3[pKF].inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            if(NonCorrectedSim3.count(pParentKF))
                Sjw = NonCorrectedSim3[pParentKF];
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Slw;
                if(NonCorrectedSim3.count(pLKF))
                    Slw = NonCorrectedSim3[pLKF];
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;
                    if(NonCorrectedSim3.count(pKFn))
                        Snw = NonCorrectedSim3[pKFn];
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // OPTIMIZE

    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, float th2)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    cv::Mat K1 = pKF1->GetCalibrationMatrix();
    cv::Mat K2 = pKF2->GetCalibrationMatrix();

    // Camera poses
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    // SET SIMILARITY VERTEX
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // SET MAP POINT VERTICES
    const int N = vpMatches1.size();
    vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<float> vSigmas12, vSigmas21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        int id1 = 2*i+1;
        int id2 = 2*(i+1);

        int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // SET EDGE x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        cv::KeyPoint kpUn1 = pKF1->GetKeyPointUn(i);
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        float invSigmaSquare1 = pKF1->GetInvSigma2(kpUn1.octave);
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // SET EDGE x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        cv::KeyPoint kpUn2 = pKF2->GetKeyPointUn(i2);
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->GetSigma2(kpUn2.octave);
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=NULL;
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=NULL;
            vpEdges21[i]=NULL;
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=NULL;
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}

void Optimizer::LocalBundleAdjustmentMonocularTest(KeyFrame *pKF, vector<bool>& vbMathced)
{
    const float thHuber = sqrt(5.991);

    //set reference frame as pKF
    cv::Mat R1 = pKF->GetRotation();
    cv::Mat t1 = pKF->GetTranslation();
    cv::Mat Tcw = pKF->GetPose();

    //set up optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    long unsigned int maxKFid = 0;

    // Get keyframes and map points
    list<KeyFrame*> lLocalKeyFrames;
    set<KeyFrame*>  sLocalKeyFrames;
    list<MapPoint*> lLocalMapPoints;
    vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
    for(size_t i = 0; i < vpMapPoints.size(); i++){
        if(!vbMathced[i]) continue;
        lLocalMapPoints.push_back(vpMapPoints[i]);
        MapPoint* pMP = vpMapPoints[i];
        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            if (sLocalKeyFrames.count(pKFi)) continue;
            sLocalKeyFrames.insert(pKFi);
            lLocalKeyFrames.push_back(pKFi);
        }
    }
    if (lLocalKeyFrames.size() == 0 || lLocalMapPoints.size() == 0) return;

    // SET LOCAL KEYFRAME VERTICES
    vector<g2o::EdgeSE3ToSE3*> vpEdges_KFs;
    map<KeyFrame*, cv::Mat> mapKF2CamCenter;

    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();

        cv::Mat RelativeTcw = cv::Mat::eye(4,4,CV_32F);
        cv::Mat R2 = pKFi->GetRotation();
        cv::Mat t2 = pKFi->GetTranslation();

        cv::Mat R12 = R2*R1.inv();
        cv::Mat t12 = -R12*(-R1*R2.t()*t2+t1);
        cv::Mat camCenter = -R12.t()*t12;

        R12.copyTo(RelativeTcw.rowRange(0,3).colRange(0,3));
        t12.copyTo(RelativeTcw.rowRange(0,3).col(3));

        // use pose relative to reference frame
        vSE3->setEstimate(Converter::toSE3Quat(RelativeTcw));
        vSE3->setId(pKFi->mnId);
        if(pKFi->mnId == pKF->mnId){
            vSE3->setFixed(true);
        }
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;

        mapKF2CamCenter[pKFi] = camCenter;
//        cout << "DEBUG noise corrupted cam center: " << endl << camCenter << endl;
    }

    cout << endl;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++){
        // Add edge constraints to reference KF
        KeyFrame* pKFi0 = *lit;
        cv::Mat camCenter0 = mapKF2CamCenter[pKFi0];

//        cout << "DEBUG camCenter0 of frame " << pKFi0->mnId << " : " << endl << camCenter0 << endl;

        for(map<KeyFrame*, cv::Mat>::iterator mit = mapKF2CamCenter.begin(); mit!=mapKF2CamCenter.end(); mit++){
            KeyFrame* pKFi1 = mit->first;
            if(pKFi0->mnId >= pKFi1->mnId) continue;
            cv::Mat camCenter1 = mapKF2CamCenter[pKFi1];

            Eigen::Matrix<double,3,1> obs;
            obs << camCenter1.at<float>(0,0) - camCenter0.at<float>(0,0), camCenter1.at<float>(1,0)-camCenter0.at<float>(1,0), camCenter1.at<float>(2,0)-camCenter0.at<float>(2,0);
//            cout << "DEBUG obs between " << pKFi1->mnId << " and " << pKFi0->mnId << " is: " << endl;
//            cout << obs << endl << endl;

            g2o::EdgeSE3ToSE3* e = new g2o::EdgeSE3ToSE3();
            e->setId(int(0.5*(pKFi0->mnId + pKFi1->mnId)*(pKFi0->mnId + pKFi1->mnId + 1) + pKFi1->mnId));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi0->mnId)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi1->mnId)));
            e->setMeasurement(obs);
            float invSigma2 = 100;
            e->setInformation(Eigen::Matrix<double, 3, 3>::Identity()*invSigma2);
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber);
            optimizer.addEdge(e);
            vpEdges_KFs.push_back(e);
        }
    }


    // SET MAP POINT VERTICES
    const int nExpectedSize = lLocalKeyFrames.size()*lLocalMapPoints.size();

    vector<g2o::EdgeProjectInverseDepth2SE3*> vpEdges;
    vpEdges.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKF;
    vpEdgeKF.reserve(nExpectedSize);

    vector<float> vSigmas2;
    vSigmas2.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdge;
    vpMapPointEdge.reserve(nExpectedSize);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();

        cv::Mat p3d = cv::Mat::eye(4,1,CV_32F);
        pMP->GetWorldPos().copyTo(p3d.rowRange(0,3)); p3d.at<float>(3) = 1.0;
        cv::Mat posInCamFrame = Tcw*p3d;

        cv::Mat Pos_inverseDepth = posInCamFrame.rowRange(0,3)/posInCamFrame.at<float>(2); // 3 x 1
        Pos_inverseDepth.at<float>(2) = 1.0/posInCamFrame.at<float>(2);

//        cout << "DEBUG MapPoint: " << endl << Pos_inverseDepth << endl;

        vPoint->setEstimate(Converter::toVector3d(Pos_inverseDepth));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        optimizer.addVertex(vPoint);
        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //SET EDGES
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
//            cout << "DEBUG observations size: " << observations.size() << endl;
            KeyFrame* pKFi = mit->first;
            bool bKFinLocalList = false;
            for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++){
//                cout << "DEBUG pKFi->mnId " << pKFi->mnId << " " << (*lit)->mnId << endl;
                if (pKFi->mnId == (*lit)->mnId){
                    bKFinLocalList= true;
                    break;
                }
            }

            if(!bKFinLocalList) continue;

            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pKFi->GetKeyPointUn(mit->second);
            obs<<kpUn.pt.x, kpUn.pt.y;
            g2o::EdgeProjectInverseDepth2SE3* e = new g2o::EdgeProjectInverseDepth2SE3();
            if(optimizer.vertex(id)==NULL || optimizer.vertex(pKFi->mnId)==NULL ) continue;
            e->setId(int(0.5*(id + pKFi->mnId)*(id + pKFi->mnId + 1) + pKFi->mnId));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
            e->setMeasurement(obs);
            float sigma2 = pKFi->GetSigma2(kpUn.octave);
            float invSigma2 = pKFi->GetInvSigma2(kpUn.octave);
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber);

            e->fx = pKF->fx; e->fy = pKF->fy;
            e->cx = pKF->cx; e->cy = pKF->cy;

            optimizer.addEdge(e);
            vpEdges.push_back(e);
            vpEdgeKF.push_back(pKFi);
            vSigmas2.push_back(sigma2);
            vpMapPointEdge.push_back(pMP);
        }
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(50);

/*    //Check inlier observations
    for(size_t i=0, iend=vpEdges.size(); i<iend;i++)
    {
        g2o::EdgeProjectInverseDepth2SE3* e = vpEdges[i];
        MapPoint* pMP = vpMapPointEdge[i];

//        cout << "DEBUG edge " << i << " chi2() : "  << e->chi2() << endl;
//        cout << "DEBUG edge " << i << " error(): " << endl << e->error() << endl;

        if(pMP->isBad())
            continue;

        if(!e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKF[i];
            if(pKFi == pKF){
                int id = pMP->GetIndexInKeyFrame(pKF);
                vbMathced[id] = false;
            }
            optimizer.removeEdge(e);
            vpEdges[i]=NULL;
        }
    }*/

//    for(size_t i=0, iend=vpEdges_KFs.size(); i<iend; i++){
//        g2o::EdgeSE3ToSE3* e = vpEdges_KFs[i];
//        cout << "DEBUG edge " << i << " chi2() : "  << e->chi2() << endl;
//        cout << "DEBUG edge " << i << " error(): " << e->error() << endl;
//    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad()) continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        // pos of map pts in inverseDepthParam in camera frame
        cv::Mat inverseDepthParamXYZ = Converter::toCvMat(vPoint->estimate());
        cv::Mat XYZ = cv::Mat::eye(4,1,CV_32F);
        XYZ.at<float>(0) = inverseDepthParamXYZ.at<float>(0)/inverseDepthParamXYZ.at<float>(2);
        XYZ.at<float>(1) = inverseDepthParamXYZ.at<float>(1)/inverseDepthParamXYZ.at<float>(2);
        XYZ.at<float>(2) = 1.0/inverseDepthParamXYZ.at<float>(2);
        XYZ.at<float>(3) = 1.0;
        XYZ = Tcw.inv()*XYZ;
        pMP->SetWorldPos(XYZ.rowRange(0,3));
    }

    //Keyframes
//    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
//    {
//        KeyFrame* pKFi = *lit;
//        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
//        g2o::SE3Quat SE3quat = vSE3->estimate();
//        cv::Mat T = Converter::toCvMat(SE3quat);
//        cout << "DEBUG camcenter_new of frame " << pKFi->mnId << endl << (-1.0)*T.rowRange(0,3).colRange(0,3).t()*T.col(3).rowRange(0,3) << endl;
//        pKFi->SetPose(T*Tcw);
//    }
}



void Optimizer::TestLocalBundleAdjustment()
{
    // Define config parameters
    bool bFixData = false;
    unsigned int nKFs = 3;
    unsigned int nMPs = 10;
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
    float fRollError = 0;//0.5; //degree
    float fPitchError = 0;//0.5;
    float fYawError = 0;//0.5;

    int nMAX_X2 = 2*10*10;
    int nMAX_Y2 = 2*10*10;
    int nMAX_Z2 = 20*20;

    // Generate Intrinsics
    float fx, fy, cx, cy; // mono-front
    fx = 1408.7635631701; fy = 1408.2906649996; cx = 653.2604772699; cy = 389.0180463604;
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    srand (time(NULL));

    // Store all Keyframes and Mappoints
    std::vector<KeyFrame*> vAllKeyFrames;
    std::vector<MapPoint*> vAllMapPoints;

    // Generate test data for KF0;
    KeyFrame* pKF0;

    // generate extra KFs
    for(unsigned int i = 0; i < nKFs; i++){
        KeyFrame* pKFi = new KeyFrame();
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
        cv::Mat Rcw2(3,3,CV_32F);
        Rcw2.at<float>(0,0) = rotationM[0][0];
        Rcw2.at<float>(0,1) = rotationM[0][1];
        Rcw2.at<float>(0,2) = rotationM[0][2];

        Rcw2.at<float>(1,0) = rotationM[1][0];
        Rcw2.at<float>(1,1) = rotationM[1][1];
        Rcw2.at<float>(1,2) = rotationM[1][2];

        Rcw2.at<float>(2,0) = rotationM[2][0];
        Rcw2.at<float>(2,1) = rotationM[2][1];
        Rcw2.at<float>(2,2) = rotationM[2][2];

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

        cout << "DEBUG camCenter " << endl << camCenter << endl;
        cv::Mat tcw2 = -Rcw2*camCenter;
//        cout << "DEBUG tcw2 "<< endl << tcw2 << endl;

        cv::Mat Tcw2 = cv::Mat::eye(4,4,CV_32F);
        Rcw2.copyTo(Tcw2.rowRange(0,3).colRange(0,3));
        tcw2.copyTo(Tcw2.col(3).rowRange(0,3));

        // set pose and ID to KFs
        pKFi->SetPose(Tcw2);
        pKFi->SetCalibrationMatrix(K);
//        cout << "DEBUG generated pose " << endl << pKFi->GetPose() << endl;
        pKFi->mnId = i + nReferenceKF_ID;
        vAllKeyFrames.push_back(pKFi);
        if(i == 0){
            pKF0 = pKFi;
        }
    }

    // generate Map points in KF0 camera frame;
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

//        cout << "DEBUG MPs: " << x << " " << y << " " << z << endl;

        cv::Mat x3D(4,1,CV_32F);
        x3D.at<float>(0) = x; x3D.at<float>(1) = y; x3D.at<float>(2) = z; x3D.at<float>(3) = 1.0;
        // assign ID and world coordinate to generated MP point.
        MapPoint* pMPi = new MapPoint();
        pMPi->mnId = i + nFirstMapPointID;
        pMPi->SetWorldPos(x3D.rowRange(0,3));

        //project pMPi to keyframes to generate KeyPoint location
        for(size_t j = 0; j < vAllKeyFrames.size(); j++){
            KeyFrame* pKF = vAllKeyFrames[j];
            cv::Mat x3DinCurKP =pKF->GetPose()*x3D;
            x3DinCurKP = x3DinCurKP/x3DinCurKP.at<float>(2);
            x3DinCurKP = K*x3DinCurKP.rowRange(0,3);
            cv::KeyPoint kpt;
            kpt.pt.x = x3DinCurKP.at<float>(0)/x3DinCurKP.at<float>(2);
            kpt.pt.y = x3DinCurKP.at<float>(1)/x3DinCurKP.at<float>(2);
            kpt.octave = 2;
            pKF->TestAddKeyPointUn(kpt);
            pKF->TestAddMapPoint(pMPi);
            pMPi->AddObservation(pKF,i);
            pKF->UpdateConnections();
        }
        vAllMapPoints.push_back(pMPi);
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

    // pose translation noise
    for(size_t i = 0; i < vAllKeyFrames.size(); i++){
        KeyFrame* pKFi = vAllKeyFrames[i];
        if (pKFi->mnId == nReferenceKF_ID || pKFi->mnId == nReferenceKF_ID + 1) continue;
//        cout << "DEBUG Previous Pose: " << endl << pKFi->GetPose() << endl;
        cv::Mat camCenter = pKFi->GetRotation().t()*pKFi->GetTranslation()*(-1.0);

        camCenter.at<float>(0) += f_t1Noise;
        camCenter.at<float>(1) += f_t2Noise;
        camCenter.at<float>(2) += f_t3Noise;

        camCenter = (-1.0)*pKFi->GetRotation()*camCenter;

        cv::Mat pose = pKFi->GetPose();
        camCenter.copyTo(pose.rowRange(0,3).col(3));
        pKFi->SetPose(pose);
//        cout << "DEBUG noise corrupted cam center: " << endl << pKFi->GetCameraCenter() << endl;
    }
    // pose rotation noise
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
    bool flag = false;
    LocalBundleAdjustmentMonocularTest(pKF0, vbMatched);

    cout << endl << "==================== DEBUG Optimizer::LocalBA ==================" << endl;

    cout << "Number of KeyFrames: " << nKFs << endl;
    cout << "Number of Map Points: " << nMPs << endl << endl;

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

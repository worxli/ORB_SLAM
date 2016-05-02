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

#include "CameraFrame.h"
#include "Converter.h"
#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>

namespace ORB_SLAM
{
bool CameraFrame::mbInitialComputations=true;
float CameraFrame::cx, CameraFrame::cy, CameraFrame::fx, CameraFrame::fy;
int CameraFrame::mnMinX, CameraFrame::mnMinY, CameraFrame::mnMaxX, CameraFrame::mnMaxY;

CameraFrame::CameraFrame()
{}

//Copy Constructor
CameraFrame::CameraFrame(const CameraFrame &frame)
    :im(frame.im.clone()), mK(frame.mK.clone()), mR(frame.mR.clone()), mt(frame.mt.clone()),
     mDistCoef(frame.mDistCoef.clone()), mXi(frame.mXi), mmapX(frame.mmapX.clone()),
     mmapY(frame.mmapY.clone()), N(frame.N), mvKeys(frame.mvKeys), mvKeysUn(frame.mvKeysUn),
     mDescriptors(frame.mDescriptors.clone()),
     pluckerLines(frame.pluckerLines),vBearings(frame.vBearings),
     mpORBvocabulary(frame.mpORBvocabulary), mpORBextractor(frame.mpORBextractor),
     mfGridElementWidthInv(frame.mfGridElementWidthInv), mfGridElementHeightInv(frame.mfGridElementHeightInv)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];
}

CameraFrame::CameraFrame(cv::Mat &im_, cv::Mat &K, cv::Mat &distCoef, cv::Mat &R, cv::Mat &t, float &xi, cv::Mat &mapX, cv::Mat &mapY, ORBextractor* extractor, ORBVocabulary* voc)
    :im(im_), mK(K.clone()),mDistCoef(distCoef.clone()), mR(R.clone()), mt(t.clone()),mpORBvocabulary(voc),mpORBextractor(extractor),
    mXi(xi), mmapX(mapX.clone()), mmapY(mapY.clone())
{
    // Exctract ORB
    (*mpORBextractor)(im,cv::Mat(),mvKeys,mDescriptors);

    N = mvKeys.size();

    cout << "# features: " << N << endl;

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    //PluckerLine();
    KeyfeatureBearings();


    // This is done for the first created Frame
    if(mbInitialComputations)
    {
        ComputeImageBounds();

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);

        mbInitialComputations=false;
    }

    // Assign Features to Grid Cells
    int nReserve = 0.5*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);


    for(size_t i=0;i<mvKeysUn.size();i++)
    {
        cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }

    //mvbOutlier = vector<bool>(N,false);
}

bool CameraFrame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float PcX = Pc.at<float>(0);
    const float PcY= Pc.at<float>(1);
    const float PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale level acording to the distance
    float ratio = dist/minDistance;

    vector<float>::iterator it = lower_bound(mvScaleFactors.begin(), mvScaleFactors.end(), ratio);
    int nPredictedLevel = it-mvScaleFactors.begin();

    if(nPredictedLevel>=mnScaleLevels)
        nPredictedLevel=mnScaleLevels-1;

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> CameraFrame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, int minLevel, int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(mvKeysUn.size());

    int nMinCellX = floor((x-mnMinX-r)*mfGridElementWidthInv);
    nMinCellX = max(0,nMinCellX);
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    int nMaxCellX = ceil((x-mnMinX+r)*mfGridElementWidthInv);
    nMaxCellX = min(FRAME_GRID_COLS-1,nMaxCellX);
    if(nMaxCellX<0)
        return vIndices;

    int nMinCellY = floor((y-mnMinY-r)*mfGridElementHeightInv);
    nMinCellY = max(0,nMinCellY);
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    int nMaxCellY = ceil((y-mnMinY+r)*mfGridElementHeightInv);
    nMaxCellY = min(FRAME_GRID_ROWS-1,nMaxCellY);
    if(nMaxCellY<0)
        return vIndices;

    bool bCheckLevels=true;
    bool bSameLevel=false;
    if(minLevel==-1 && maxLevel==-1)
        bCheckLevels=false;
    else
        if(minLevel==maxLevel)
            bSameLevel=true;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels && !bSameLevel)
                {
                    if(kpUn.octave<minLevel || kpUn.octave>maxLevel)
                        continue;
                }
                else if(bSameLevel)
                {
                    if(kpUn.octave!=minLevel)
                        continue;
                }

                if(abs(kpUn.pt.x-x)>r || abs(kpUn.pt.y-y)>r)
                    continue;

                vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool CameraFrame::PosInGrid(cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

void CameraFrame::SetScaleParams(int &_mnScaleLevels, vector<float> _mvScaleFactors, vector<float> _mvLevelSigma2, vector<float> _mvInvLevelSigma2)
{
    mnScaleLevels = _mnScaleLevels;
    mvScaleFactors = _mvScaleFactors;
    mvLevelSigma2 = _mvLevelSigma2;
    mvInvLevelSigma2 = _mvInvLevelSigma2;
}

void CameraFrame::SetPoseMatrices(cv::Mat _mRcw, cv::Mat _mtcw, cv::Mat _mOw)
{
    mRcw = _mRcw;
    mtcw = _mtcw;
    mOw = _mOw;
}

void CameraFrame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(mvKeys.size(),2,CV_32F);
    for(unsigned int i=0; i<mvKeys.size(); i++)
    {
        mat.at<float>(i,0)= mmapX.at<float>(mvKeys[i].pt.y, mvKeys[i].pt.x);
        mat.at<float>(i,1)= mmapY.at<float>(mvKeys[i].pt.y, mvKeys[i].pt.x);
    }

    // Undistort points
    mat = mat.reshape(2);
    //cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
    mat = mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(mvKeys.size());
    for (unsigned int i = 0; i < mvKeys.size(); i++) {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x = mat.at<float>(i, 0);
        kp.pt.y = mat.at<float>(i, 1);
        mvKeysUn[i] = kp;
    }
}

void CameraFrame::undistort(const Eigen::Vector2d& p, Eigen::Vector2d& p_u)
{
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;

    cv::Mat mK_inv = mK.inv();

    // Lift points to normalised plane
    mx_d = mK_inv.at<float>(0,0) * p(0) + mK_inv.at<float>(0,2);
    my_d = mK_inv.at<float>(1,1) * p(1) + mK_inv.at<float>(1,2);

    // Apply inverse distortion model
    double k1 = mDistCoef.at<float>(0);
    double k2 = mDistCoef.at<float>(1);
    double p1 = mDistCoef.at<float>(2);
    double p2 = mDistCoef.at<float>(3);

    // Inverse distortion model
    // proposed by Heikkila
    mx2_d = mx_d*mx_d;
    my2_d = my_d*my_d;
    mxy_d = mx_d*my_d;
    rho2_d = mx2_d+my2_d;
    rho4_d = rho2_d*rho2_d;
    radDist_d = k1*rho2_d+k2*rho4_d;
    Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
    Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
    inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

    mx_u = mx_d - inv_denom_d*Dx_d;
    my_u = my_d - inv_denom_d*Dy_d;

    p_u << mx_u, my_u;
}

void CameraFrame::LiftToSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P)
{
    cv::Mat mK_inv = mK.inv();
    double u = p(0);
    double v = p(1);

    // Undistort pixel point
    Eigen::Vector2d m_d, m_u;
    m_d << u,v;

    undistort(m_d, m_u);

    float mx_u = m_u(0);
    float my_u = m_u(1);

    // Lift normalised points to the sphere (inv_hslash)
    double lambda;
    double xi = mXi;

    if (xi == 1.0)
    {
        lambda = 2.0 / (mx_u * mx_u + my_u * my_u + 1.0);
        P << lambda * mx_u, lambda * my_u, lambda - 1.0;
    }
    else
    {
        lambda = (xi + sqrt(1.0 + (1.0 - xi * xi) * (mx_u * mx_u + my_u * my_u))) / (1.0 + mx_u * mx_u + my_u * my_u);
        P << lambda * mx_u, lambda * my_u, lambda - xi;
    }
}

void CameraFrame::PluckerLine()
{
	for(unsigned int i=0; i<mvKeys.size(); i++)
	{
		Eigen::Vector2d p_in;

		p_in << mvKeys[i].pt.x, mvKeys[i].pt.y;
		Eigen::Vector3d P;
		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		cv::cv2eigen(mR,R);
		cv::cv2eigen(mt,t);
		LiftToSphere(p_in, P);
		std::vector<Eigen::Vector3d> pluckerLine;
		Eigen::Vector3d q = R.inverse()*(P-t);
		q.normalize();
		pluckerLine.push_back(q);
		pluckerLine.push_back(pluckerLine[0].cross(-1*R.inverse()*t));
		pluckerLines.push_back(pluckerLine);
	}
}

void CameraFrame::KeyfeatureBearings()
{

	for(unsigned int i=0; i<mvKeys.size(); i++)
	{
		Eigen::Vector2d p_in;
		Eigen::Vector3d bearing;

		p_in << mvKeys[i].pt.x, mvKeys[i].pt.y;

        LiftToSphere(p_in, bearing);
		bearing.normalize(); //needed?
		vBearings.push_back(bearing);
	}
}

void CameraFrame::ComputeImageBounds()
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);

        // Cut away certain pixel width at the boundaries since fisheye distorts coners to infinity
        float cut_image_size = 200;

        mat.at<float>(0,0)=cut_image_size; mat.at<float>(0,1)=cut_image_size;
        mat.at<float>(1,0)=im.cols - cut_image_size; mat.at<float>(1,1)=cut_image_size;
        mat.at<float>(2,0)=cut_image_size; mat.at<float>(2,1)=im.rows - cut_image_size;
        mat.at<float>(3,0)=im.cols - cut_image_size; mat.at<float>(3,1)=im.rows - cut_image_size;

        Eigen::Vector3d empty;
        for (int i = 0; i < mat.rows; ++i) {
            mat.at<float>(i,0)= mmapX.at<float>(mat.at<float>(i,1), mat.at<float>(i,0));
            mat.at<float>(i,1)= mmapY.at<float>(mat.at<float>(i,1), mat.at<float>(i,0));
        }

        // Undistort corners
        mat=mat.reshape(2);
        //cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(floor(mat.at<float>(0,0)),floor(mat.at<float>(2,0)));
        mnMaxX = max(ceil(mat.at<float>(1,0)),ceil(mat.at<float>(3,0)));
        mnMinY = min(floor(mat.at<float>(0,1)),floor(mat.at<float>(1,1)));
        mnMaxY = max(ceil(mat.at<float>(2,1)),ceil(mat.at<float>(3,1)));

    }
    else
    {
        mnMinX = 0;
        mnMaxX = im.cols;
        mnMinY = 0;
        mnMaxY = im.rows;
    }
}

} //namespace ORB_SLAM

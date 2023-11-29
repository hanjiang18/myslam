/**
* This file is part of CCM-SLAM.
*
* Copyright (C): Patrik Schmuck <pschmuck at ethz dot ch> (ETH Zurich)
* For more information see <https://github.com/patriksc/CCM-SLAM>
*
* CCM-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CCM-SLAM is based in the monocular version of ORB-SLAM2 by Raúl Mur-Artal.
* CCM-SLAM partially re-uses modules of ORB-SLAM2 in modified or unmodified condition.
* For more information see <https://github.com/raulmur/ORB_SLAM2>.
*
* CCM-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CCM-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <cslam/Mapping.h>
#include <fstream>
using namespace std;
namespace cslam {

LocalMapping::LocalMapping(ccptr pCC, mapptr pMap, dbptr pDB, viewptr pViewer = nullptr)
    : mpCC(pCC),mpKFDB(pDB),
      mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
      mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
      mClientId(pCC->mClientId),
      mpViewer(pViewer)
{
    mAddedKfs = 0;
    mCulledKfs = 0;

    mCountKFs = 0;
}

void LocalMapping::RunClient()
{
    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Get Communicator Mutex -> Comm cannot publish. Assure no publishing whilst changing data
            if(params::sys::mbStrictLock) while(!mpCC->LockMapping()){
                usleep(params::timings::miLockSleep);
            }

            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            //Visualize
            if(params::vis::mbActive)
                if(mpMap->GetMaxKFid() > 0)
                    mpViewer->DrawMap(mpMap);

            // Check recent MapPoints
            MapPointCullingClient();

            // Triangulate new MapPoints
            CreateNewMapPoints();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            mbAbortBA = false;

            //Map Forgetting
            while(!mpMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}

            mpMap->MapTrimming(mpCurrentKeyFrame);

            mpMap->UnLockMapUpdate();

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                if(mpMap->KeyFramesInMap()>2)
                    Optimizer::LocalBundleAdjustmentClient(mpCurrentKeyFrame,&mbAbortBA,mpMap,mClientId);
            }

            if(params::sys::mbStrictLock)
                mpCC->UnLockMapping();
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(params::timings::client::miMappingRate);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(params::timings::client::miMappingRate);
    }

    SetFinish();
}

void LocalMapping::RunServer()
{
    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            #ifdef LOGGING
            mpCC->mpLogger->SetMapping(__LINE__,mClientId);
            #endif

            while(!mpCC->LockMapping()){usleep(params::timings::miLockSleep);}

            if(mpCC->mbOptActive) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m LocalMapping::Run(...): Optimization active - LocalMapping should be locked" << endl;

            #ifdef LOGGING
            mpCC->mpLogger->SetMapping(__LINE__,mClientId);
            #endif

            while(!mpMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}

            #ifdef LOGGING
            mpCC->mpLogger->SetMapping(__LINE__,mClientId);
            #endif

            // pop KF from queue
            ProcessNewKeyFrame();

            //Visualize
            mpViewer->DrawMap(mpMap);

            // Check recent MapPoints
            MapPointCullingServer();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            // Check redundant local Keyframes
            if(params::mapping::mfRedundancyThres < 1.0 && !CheckNewKeyFrames())
            {
                KeyFrameCullingV3();
                //mpMap->RemoveRedundantData(0,0);
            }

            mpLoopFinder->InsertKF(mpCurrentKeyFrame);
            
            mpMapMatcher->InsertKF(mpCurrentKeyFrame);

            mpKFDB->add(mpCurrentKeyFrame);

            mpMap->ClearBadMPs();

            mpMap->UnLockMapUpdate();

            mpCC->UnLockMapping();

            ofstream keyframesFile; 
             ros::Time right_now = ros::Time::now();
             keyframesFile.open("/home/ccm/ccm1.txt", ios::app);
             //const double stamp = right_now;
            const cv::Mat T_wc = mpCurrentKeyFrame->GetPoseInverse();
            const Eigen::Matrix4d eT_wc = cslam::Converter::toMatrix4d(T_wc);
            Eigen::Matrix4d T_SC;
            T_SC = mpCurrentKeyFrame->mT_SC;
            const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
            const Eigen::Quaterniond q(Tws.block<3,3>(0,0));
            keyframesFile << std::setprecision(6) << right_now << " ";
            keyframesFile << Tws(0,3) << " " << Tws(1,3) << " " << Tws(2,3) << " ";
            keyframesFile << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
            keyframesFile.close();
            #ifdef LOGGING
            mpCC->mpLogger->SetMapping(__LINE__,mClientId);
            #endif
        }
        else
        {
            #ifdef LOGGING
            mpCC->mpLogger->SetMapping(__LINE__,mClientId);
            #endif
        }

        ResetIfRequested();

        usleep(params::timings::server::miMappingRate);
    }
}

void LocalMapping::InsertKeyFrame(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
}

bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    if(mpCC->mSysState == eSystemState::SERVER)
    {
        ++mAddedKfs;
        ++mCountKFs;

        vector<mpptr> vpMPs = mpCurrentKeyFrame->GetMapPointMatches();
        for(vector<mpptr>::iterator vit = vpMPs.begin();vit!=vpMPs.end();++vit)
        {
            mpptr pMPi = *vit;

            if(!pMPi || pMPi->isBad())
                continue;

            if(pMPi->mInsertedWithKF == -1)
            {
                pMPi->mInsertedWithKF = mCountKFs;
                mlpRecentAddedMapPoints.push_back(pMPi);
            }
        }

        mlpRecentAddedKFs.push_back(mpCurrentKeyFrame);
        if(mlpRecentAddedKFs.size() > params::mapping::miNumRecentKFs)
            mlpRecentAddedKFs.pop_front();
    }
    else if(mpCC->mSysState == eSystemState::CLIENT)
    {
        // Compute Bags of Words structures
        mpCurrentKeyFrame->ComputeBoW();

        // Associate MapPoints to the new keyframe and update normal and descriptor
        const vector<mpptr> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            mpptr pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                    {
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        pMP->UpdateNormalAndDepth();
                        pMP->ComputeDistinctiveDescriptors();
                    }
                    else // this can only happen for new stereo points inserted by the Tracking - remnant from ORB_SLAM, could probably be removed
                    {
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }

        // Update links in the Covisibility Graph
        mpCurrentKeyFrame->UpdateConnections();

        // Insert Keyframe in Map
        mpMap->AddKeyFrame(mpCurrentKeyFrame);
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m LocalMapping::ProcessNewKeyFrame(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn=10;
    const vector<kfptr> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        kfptr pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        //myadd
        if(baseline<pKF2->mb)
            continue;
        //----------
        // const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
        // const float ratioBaselineDepth = baseline/medianDepthKF2;

        // if(ratioBaselineDepth<0.01)
        //     continue;

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;
            if(bStereo1)
                // 传感器是双目相机,并且当前的关键帧的这个点有对应的深度
                // 假设是平行的双目相机，计算出双目相机观察这个点的时候的视差角余弦
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                // 传感器是双目相机,并且邻接的关键帧的这个点有对应的深度，和上面一样操作
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));
            
            // 得到双目观测的视差角中最小的那个
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);
            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)  
            {
                // 如果是双目，用视差角更大的那个双目信息来恢复，直接用已知3D点反投影了
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)  
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                // 单目情况下
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                // 假设测量有一个像素的偏差，2自由度卡方检验阈值是5.991
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                // 双目情况
                float u1 = fx1*x1*invz1+cx1;
                // 根据视差公式计算假想的右目坐标
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;     
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                // 自由度为3，卡方检验阈值是7.8
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;

            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            mpptr pMP{new MapPoint(x3D,mpCurrentKeyFrame,mpMap,mClientId,mpComm,mpCC->mSysState,-1)};

            pMP->AddObservation(mpCurrentKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn=10;
    const vector<kfptr> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<kfptr> vpTargetKFs;
    for(vector<kfptr>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        kfptr pKFi = *vit;
        if(pKFi->isBad() || pKFi->mFuseTargetForKF == mpCurrentKeyFrame->mId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mFuseTargetForKF = mpCurrentKeyFrame->mId;

        // Extend to some second neighbors
        const vector<kfptr> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<kfptr>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            kfptr pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mFuseTargetForKF==mpCurrentKeyFrame->mId || pKFi2->mId==mpCurrentKeyFrame->mId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }

    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<mpptr> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<kfptr>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        kfptr pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<mpptr> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<kfptr>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        kfptr pKFi = *vitKF;

        vector<mpptr> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<mpptr>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            mpptr pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mFuseCandidateForKF == mpCurrentKeyFrame->mId)
                continue;
            pMP->mFuseCandidateForKF = mpCurrentKeyFrame->mId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);

    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        mpptr pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(kfptr &pKF1, kfptr &pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        if(mpCC->mSysState == eSystemState::CLIENT)
            usleep(params::timings::client::miMappingRate);
        else if(mpCC->mSysState == eSystemState::SERVER)
            usleep(params::timings::server::miMappingRate);
        else KILLSYS

    }
}

void LocalMapping::ResetIfRequested()
{    
    unique_lock<mutex> lock(mMutexReset);

    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();

        if(mpLoopFinder)
        {
            mpLoopFinder->RequestReset();
        }

        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LocalMapping::MapPointCullingClient()
{
    // Check Recent Added MapPoints
    list<mpptr>::iterator lit = mlpRecentAddedMapPoints.begin();
    const idpair nCurrentKFid = mpCurrentKeyFrame->mId;

    int nThObs;
    nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        mpptr pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid.first-(int)pMP->mFirstKfId.first)>=2 && nCurrentKFid.second == pMP->mFirstFrame.second && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid.first-(int)pMP->mFirstKfId.first)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::MapPointCullingServer()
{
    // Check Recent Added MapPoints
    list<mpptr>::iterator lit = mlpRecentAddedMapPoints.begin();

    int nThObs;
    nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        mpptr pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if((mCountKFs - pMP->mInsertedWithKF) >= 3 && pMP->mId.second == mpCC->mClientId && pMP->Observations() <= cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if((mCountKFs - pMP->mInsertedWithKF) >= 3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

int sum=0;
void LocalMapping::KeyFrameCullingV3()  
{
    //This version: randomly pick a KF and check for redundancy
    kfptr pKFc = mpMap->GetRandKfPtr();
    if(!pKFc)
        return; //safety check

    //we don't check KFs in mlpRecentAddedKFs, since the neighbors will probably not be allowed for culling.
    list<kfptr>::iterator lit1 = std::find(mlpRecentAddedKFs.begin(),mlpRecentAddedKFs.end(),pKFc);
    if(lit1 != mlpRecentAddedKFs.end())
    {
        //give it a second try -- if not successful return to not spend ages in this method.

        pKFc = mpMap->GetRandKfPtr();
            if(!pKFc)
                return; //safety check

        lit1 = std::find(mlpRecentAddedKFs.begin(),mlpRecentAddedKFs.end(),pKFc);
        if(lit1 != mlpRecentAddedKFs.end())
            return;
    }

    if(mspKFsCheckedForCulling.count(pKFc))
        return;
    else
        mspKFsCheckedForCulling.insert(pKFc);

    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<kfptr> vpLocalKeyFrames = pKFc->GetVectorCovisibleKeyFrames();

    //vector<kfptr> vpLocalKeyFrames =mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<kfptr>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        kfptr pKF = *vit;
        
        if(pKF->mId.first==0 || pKF->mId.first==1) //don't cull 0, since it's the origin, and also not one, because this was the other KF used for initialization. The systen won't experience problems if culling 1, nevetheless we don't do it.
            continue;

        list<kfptr>::iterator lit2 = std::find(mlpRecentAddedKFs.begin(),mlpRecentAddedKFs.end(),pKF);
        if(lit2 != mlpRecentAddedKFs.end())
            continue;
        const vector<mpptr> vpMapPoints = pKF->GetMapPointMatches();

        const int thObs=3;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            mpptr pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                     if(pKF->passDepth[i]>40.0 || pKF->passDepth[i]<0)
                             continue;
                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<kfptr, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<kfptr, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            kfptr pKFi = mit->first;

                            if(pKFi->isBad()) continue;

                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }
        
        //cout<<"mapping culling : "<<pKF->mId.first<<" "<<pKF->mId.second<<"redundant "<<nRedundantObservations<<"0.9nmaps : "<<0.9*nMPs<<endl;
        if(nRedundantObservations>params::mapping::mfRedundancyThres*nMPs)
        {
            pKF->SetBadFlag();
            //cout<<"kf culling : "<<pKF->mId.first<<" "<<pKF->mId.second<<"  "<<++sum<<endl;
            ++mCulledKfs;
        }
    }
}

void LocalMapping::ClearCovGraph(size_t MapId)
{
    mpViewer->ClearCovGraph(MapId);
}

} //end ns

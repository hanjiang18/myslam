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

#include <cslam/Frame.h>

namespace cslam {

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractor(frame.mpORBextractor),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     N(frame.N), mvKeys(frame.mvKeys), mvKeysUn(frame.mvKeysUn),
     mbf(frame.mbf), mvDepth(frame.mvDepth),
     mb(frame.mb),mThDepth(frame.mThDepth), mvuRight(frame.mvuRight),	
     mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec), mDescriptors(frame.mDescriptors.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mId(frame.mId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

Frame::Frame(const cv::Mat &imGray, const double &timeStamp, extractorptr pExtractor, vocptr pVoc, cv::Mat &K, cv::Mat &distCoef, size_t ClientId)
    :mpORBvocabulary(pVoc),mpORBextractor(pExtractor), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone())
{
    // Frame ID
    mId = make_pair(nNextId++,ClientId);
    // Scale Level Info
    mnScaleLevels = mpORBextractor->GetLevels();
    mfScaleFactor = mpORBextractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractor->GetScaleFactors();
    mvInvScaleFactors = mpORBextractor->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractor->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    mvpMapPoints = vector<mpptr>(N,nullptr);
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, extractorptr pExtractor, vocptr pVoc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,size_t ClientId)
:mpORBvocabulary(pVoc),mpORBextractor(pExtractor),mpORBextractorRight(static_cast<extractorptr>(nullptr)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Step 1 帧的ID 自增
    mId=make_pair(nNextId++,ClientId);

    // Step 2 计算图像金字塔的参数 
	//获取图像金字塔的层数
    mnScaleLevels = mpORBextractor->GetLevels();
	//获取每层的缩放因子
    mfScaleFactor = mpORBextractor->GetScaleFactor();    
	//计算每层缩放因子的自然对数
    mfLogScaleFactor = log(mfScaleFactor);
	//获取各层图像的缩放因子
    mvScaleFactors = mpORBextractor->GetScaleFactors();
	//获取各层图像的缩放因子的倒数
    mvInvScaleFactors = mpORBextractor->GetInverseScaleFactors();
	//TODO 也是获取这个不知道有什么实际含义的sigma^2
    mvLevelSigma2 = mpORBextractor->GetScaleSigmaSquares();
	//计算上面获取的sigma^2的倒数
    mvInvLevelSigma2 = mpORBextractor->GetInverseScaleSigmaSquares();

    /** 3. 提取彩色图像(其实现在已经灰度化成为灰度图像了)的特征点 \n Frame::ExtractORB() */

    // ORB extraction
	// Step 3 对图像进行提取特征点, 第一个参数0-左图， 1-右图
    ExtractORB(0,imGray);

	//获取特征点的个数
    N = mvKeys.size();

	//如果这一帧没有能够提取出特征点，那么就直接返回了
    if(mvKeys.empty())
        return;

	// Step 4 用OpenCV的矫正函数、内参对提取到的特征点进行矫正
    UndistortKeyPoints();

	// Step 5 获取图像的深度，并且根据这个深度推算其右图中匹配的特征点的视差
    ComputeStereoFromRGBD(imDepth);

    // 初始化本帧的地图点
    mvpMapPoints = vector<mpptr>(N,static_cast<mpptr>(nullptr));
	// 记录地图点是否为外点，初始化均为外点false
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
	//  Step 5 计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
    if(mbInitialComputations)
    {
		//计算去畸变后图像的边界
        ComputeImageBounds(imGray);

        // 表示一个图像像素相当于多少个图像网格列（宽）
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
		// 表示一个图像像素相当于多少个图像网格行（高）
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

		//给类的静态成员变量复制
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
		// 猜测是因为这种除法计算需要的时间略长，所以这里直接存储了这个中间计算结果
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

		//特殊的初始化过程完成，标志复位
        mbInitialComputations=false;
    }

    // 计算假想的基线长度 baseline= mbf/fx
    // 后面要对从RGBD相机输入的特征点,结合相机基线长度,焦距,以及点的深度等信息来计算其在假想的"右侧图像"上的匹配点
    mb = mbf/fx;

	// 将特征点分配到图像网格中 
    AssignFeaturesToGrid();
}
void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(const cv::Mat &im)
{
    (*mpORBextractor)(im,cv::Mat(),mvKeys,mDescriptors);
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    // 判断是左图还是右图
    if(flag==0)
        // 左图的话就套使用左图指定的特征点提取器，并将提取结果保存到对应的变量中 
        // 这里使用了仿函数来完成，重载了括号运算符 ORBextractor::operator() 
        (*mpORBextractor)(im,				//待提取特征点的图像
							  cv::Mat(),		//掩摸图像, 实际没有用到
							  mvKeys,			//输出变量，用于保存提取后的特征点
							  mDescriptors);	//输出变量，用于保存特征点的描述子
    else
        // 右图的话就需要使用右图指定的特征点提取器，并将提取结果保存到对应的变量中 
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

 void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth){
    /** 主要步骤如下:.对于彩色图像中的每一个特征点:<ul>  */
    // mvDepth直接由depth图像读取`
	//这里是初始化这两个存储“右图”匹配特征点横坐标和存储特征点深度值的vector
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

	//开始遍历彩色图像中的所有特征点
    for(int i=0; i<N; i++)
    {
        /** <li> 从<b>未矫正的特征点</b>提供的坐标来读取深度图像拿到这个点的深度数据 </li> */
		//获取校正前和校正后的特征点
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

		//获取其横纵坐标，注意 NOTICE 是校正前的特征点的
        const float &v = kp.pt.y;
        const float &u = kp.pt.x;
		//从深度图像中获取这个特征点对应的深度点
        //NOTE 从这里看对深度图像进行去畸变处理是没有必要的,我们依旧可以直接通过未矫正的特征点的坐标来直接拿到深度数据
        const float d = imDepth.at<float>(v,u);

		//
        /** <li> 如果获取到的深度点合法(d>0), 那么就保存这个特征点的深度,并且计算出等效的\在假想的右图中该特征点所匹配的特征点的横坐标 </li>
         * \n 这个横坐标的计算是 x-mbf/d
         * \n 其中的x使用的是<b>矫正后的</b>特征点的图像坐标
         */
        if(d>0)
        {
			//那么就保存这个点的深度
            mvDepth[i] = d;
            //cout<<"depth "<<d<<endl;
			//根据这个点的深度计算出等效的、在假想的右图中的该特征点的横坐标
			//TODO 话说为什么要计算这个嘞，计算出来之后有什么用?可能是为了保持计算一致
            mvuRight[i] = kpU.pt.x-mbf/d;
        }//如果获取到的深度点合法
    }//开始遍历彩色图像中的所有特征点
    /** </ul> */
 }

cv::Mat Frame::UnprojectStereo(const int &i){
    const float z=mvDepth[i];
    if(z>0){
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
		//计算在当前相机坐标系下的坐标
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
		//然后计算这个点在世界坐标系下的坐标，这里是对的，但是公式还是要斟酌一下。首先变换成在没有旋转的相机坐标系下，最后考虑相机坐标系相对于世界坐标系的平移
        return mRwc*x3Dc+mOw;
    }
    else    
        return cv::Mat();
}


void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(mpptr pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
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
    {
        return false;
    }

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this->shared_from_this());

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

} //end ns

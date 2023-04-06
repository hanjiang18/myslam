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

#ifndef CSLAM_FRAME_H_
#define CSLAM_FRAME_H_

//C++
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/ORBextractor.h>
#include <cslam/MapPoint.h>
#include <cslam/KeyFrame.h>
#include <cslam/Converter.h>

//Thirdparty
#include <thirdparty/DBoW2/DBoW2/BowVector.h>
#include <thirdparty/DBoW2/DBoW2/FeatureVector.h>

using namespace estd;

namespace cslam{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 75

//forward decs
class MapPoint;
class KeyFrame;
//--------------

class Frame : public boost::enable_shared_from_this<Frame>
{
public:
    typedef boost::shared_ptr<Frame> frameptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
public:

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, extractorptr pExtractor, vocptr pVoc, cv::Mat &K, cv::Mat &distCoef, size_t ClientId);

     // Constructor for RGB-D cameras.	
    /**
     * @brief 为RGBD相机准备的帧构造函数
     * 
     * @param[in] imGray        对RGB图像灰度化之后得到的灰度图像
     * @param[in] imDepth       深度图像
     * @param[in] timeStamp     时间戳
     * @param[in] extractor     特征点提取器句柄
     * @param[in] voc           ORB特征点词典的句柄
     * @param[in] K             相机的内参数矩阵
     * @param[in] distCoef      相机的去畸变参数
     * @param[in] bf            baseline*bf
     * @param[in] thDepth       远点和近点的深度区分阈值
     */
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, extractorptr pExtractor, vocptr pVoc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,size_t ClientId);
    // Extract ORB on the image.
    void ExtractORB(const cv::Mat &im);

    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

     // Extract ORB on the image. 0 for left image and 1 for right image.
    /**
     * @brief 提取图像的ORB特征，提取的关键点存放在mvKeys，描述子存放在mDescriptors
     * 
     * @param[in] flag          标记是左图还是右图。0：左图  1：右图
     * @param[in] im            等待提取特征点的图像
     */
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    /**
     * @brief 当某个特征点的深度信息或者双目信息有效时,将它反投影到三维世界坐标系中
     * 
     * @param[in] i     特征点的ID
     * @return cv::Mat  反投影后得到的特征点的反投影点的世界坐标
     */
    cv::Mat UnprojectStereo(const int &i);

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(mpptr pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

public:
    // Vocabulary used for relocalization.
    vocptr mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    extractorptr mpORBextractor, mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    ///baseline x fx
    float mbf;

    // Stereo baseline in meters.
    ///相机的基线长度,单位为米
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
	//TODO 这里它所说的话还不是很理解。尤其是后面的一句。
    //而且,这个阈值不应该是在哪个帧中都一样吗?
    ///判断远点和近点的深度阈值
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys;//, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysRight;
    std::vector<float> mvuRight;
     ///对应的深度
    std::vector<float> mvDepth;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors,mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<mpptr> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    idpair mId;

    // Reference Keyframe.
    kfptr mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

} //end namespace


#endif

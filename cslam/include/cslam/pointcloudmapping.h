/*
 * @Author: hanjiang18 1763983820@qq.com
 * @Date: 2023-03-26 20:16:11
 * @LastEditors: hanjiang18 1763983820@qq.com
 * @LastEditTime: 2023-06-04 21:13:36
 * @FilePath: /ccmslam_ws/src/ccm_slam-master/cslam/include/cslam/pointcloudmapping.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include <cslam/Converter.h>
#include "pointCloud.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cslam/KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <cslam/estd.h>
using namespace estd;

using namespace std;
namespace cslam{
 class KeyFrame ;
class PointCloudMapping
{
public:
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

public:
    PointCloudMapping( double resolution_,double meank_,double thresh_ );
    void save();
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( kfptr kf,idpair idk);
    void shutdown();
    void viewer();
    static void viewerOneOff(pcl::visualization::PCLVisualizer &viewer);
    void inserttu( cv::Mat& color, cv::Mat& depth,int idk);
    PointCloud::Ptr generatePointCloud(kfptr kf);
    int loopcount = 0;
    vector<kfptr> currentvpKFs;
    bool cloudbusy;
    bool loopbusy=false;
    void updatecloud();
    bool bStop = false;
    PointCloud::Ptr globalMap;
     vector<PointCloude>     pointcloud;
     vector<kfptr>       keyframes;
     int cnt = 0;
protected:
    
    
    boost::shared_ptr<thread>  viewerThread;   
    
    bool    shutDownFlag    =false;
    mutex   shutDownMutex;  
    
    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;
   
    // data to generate point clouds
    
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    vector<cv::Mat>         colorImgks;
    vector<cv::Mat>         depthImgks;
    vector<int>             ids;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;
    
    double resolution = 0.04;
    double meank = 50;
    double thresh = 1;
    pcl::VoxelGrid<PointT>  voxel;
    pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
};
}

#endif // POINTCLOUDMAPPING_H

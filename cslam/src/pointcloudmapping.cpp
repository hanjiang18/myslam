#include "cslam/pointcloudmapping.h"

namespace cslam{
int currentloopcount = 0;
 PointCloudMapping:: PointCloudMapping(double resolution_,double meank_,double thresh_){
    this->resolution = resolution_;
    this->meank = thresh_;
    this->thresh = thresh_;
    statistical_filter.setMeanK(meank);
    statistical_filter.setStddevMulThresh(thresh);
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );
    //viewerThread = boost::make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::viewerOneOff(pcl::visualization::PCLVisualizer &viewer){
    viewer.setBackgroundColor(1.0, 1.0, 1.0);
}

void PointCloudMapping::insertKeyFrame(kfptr kf,idpair idk  )
{
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    PointCloude pointcloude;
    pointcloude.pcID = idk;
    pointcloude.T =cslam::Converter::toSE3Quat(kf->GetPose());        
    pointcloude.pcE = generatePointCloud(kf);
    pointcloud.push_back(pointcloude);
    //cout<<"pointcloud "<<pointcloud.size()<<endl;
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(kfptr kf)//,Eigen::Isometry3d T
{
    //cout<<"generatePointCloud"<<endl;
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr

    for(int i=0;i<kf->passDepth.size();i++){
        float d=kf->passDepth[i];
        if (d < 0.01 || d>5)
                continue;
        PointT p;
        p.z = d;
        //cout<<"depth-------------------    "<<d<<endl;
        p.x =kf->passx[i];
        p.y = kf->passy[i];

        p.b = kf->passblue[i]; 
        p.g = kf->passgreen[i]; 
        p.r = kf->passred[i]; 

        tmp->points.push_back(p);
    }
    //cout<<"size  "<<tmp->points.size()<<endl;
    return tmp;
}

void PointCloudMapping::viewer()
{
    //pcl::visualization::CloudViewer viewer("viewer");
    //viewer.runOnVisualizationThreadOnce(viewerOneOff);
    while(1)
    { 
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        if(loopbusy || bStop)
        {
            continue;
        }
        if(lastKeyframeSize == N)
            cloudbusy = false;
        //cloudbusy = true;
        
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p (new PointCloud);
            pcl::transformPointCloud( *(pointcloud[i].pcE), *p, pointcloud[i].T.inverse().matrix());
            *globalMap += *p;
        }
      
        // depth filter and statistical removal 
         PointCloud::Ptr tmp1 ( new PointCloud );
        
         statistical_filter.setInputCloud(globalMap);
         statistical_filter.filter( *tmp1 );
        //cout<<"first "<<globalMap->size()<<endl;
          PointCloud::Ptr tmp(new PointCloud());
          voxel.setInputCloud( tmp1 );
          voxel.filter( *globalMap );
        //cout<<"show------------------------------"<<endl;
        //viewer.showCloud( globalMap );
        //cout<<"show  ::::  "<<globalMap->size()<<endl;
        //viewer->updatePointCloud<PointT>(globalMap,"global");
        // viewer->spinOnce(100);
        lastKeyframeSize = N;
        cloudbusy = false;
        //cout<<"cloudbus"<<cloudbusy<<"   loop busy"<<loopbusy<<endl;
    }
}

void PointCloudMapping::save()
{
	pcl::io::savePCDFile( "/home/ccm/ccmslam_ws/src/ccm_slam-master/cslam/result.pcd", *globalMap );
	cout<<"globalMap save finished"<<endl;
}

void PointCloudMapping::updatecloud()
{
    cout<<"come"<<endl;
	if(!cloudbusy)
	{
        cout<<"come1"<<endl;
		loopbusy = true;
		cout<<"------------mp updating---------"<<endl;
        PointCloud::Ptr tmp1(new PointCloud);
        cout<<"所有 ："<<pointcloud.size()<<"   匹配 ： "<<currentvpKFs.size()<<endl;
        cout<<"size 1 :"<<globalMap->size()<<endl;
		for (int i=0;i<currentvpKFs.size();i++)
		{
		    for (int j=0;j<pointcloud.size();j++)
		    {   
				if(pointcloud[j].pcID==currentvpKFs[i]->mId) 
				{   
                    //cout<<"pointcloud[j].pcID: " <<pointcloud[j].pcID.first<<"  "<<pointcloud[j].pcID.second<<endl;
					Eigen::Isometry3d T = cslam::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
					PointCloud::Ptr cloud(new PointCloud);
					pcl::transformPointCloud( *pointcloud[j].pcE, *cloud, T.inverse().matrix());
					*tmp1 +=*cloud;
					continue;
				}
			}
		}

        
        cout<<"------------finish map----------"<<endl;
        PointCloud::Ptr tmp2(new PointCloud());
        voxel.setInputCloud( tmp1 );
        voxel.filter( *tmp2 );
        cout<<"curr : "<<tmp2->size()<<endl;
        globalMap->swap( *tmp2 );
        cout<<"swap: "<<globalMap->size()<<endl;
        //globalMap->clear();
        cout<<"swap"<<endl;
        loopbusy = false;
        loopcount++;
	}
}
}

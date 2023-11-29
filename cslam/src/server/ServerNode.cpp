/*
 * @Author: hanjiang18 1763983820@qq.com
 * @Date: 2023-03-13 01:50:05
 * @LastEditors: hanjiang18 1763983820@qq.com
 * @LastEditTime: 2023-11-13 01:23:09
 * @FilePath: /ccmslam_ws/src/ccm_slam-master/cslam/src/server/ServerNode.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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


#include <cslam/server/ServerSystem.h>
#include "std_msgs/String.h"
#include <fstream>
#include <cslam/KeyFrame.h>
#include <cslam/Converter.h>
using namespace std;

boost::shared_ptr<cslam::ServerSystem> pSSys;

void callback(const std_msgs::String::ConstPtr& msg ){
    cout<<msg->data<<endl;
    if(msg->data == "end"){
        pSSys->shutdown();
        //ROS_INFO("I heard: [%s]", msg->data.c_str());
        ros::shutdown();
    }
        
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "CSLAM server node");
    if(argc != 2)
    {
        cerr << endl << "Usage: rosrun cslam clientnode path_to_vocabulary" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle Nh;
    ros::NodeHandle NhPrivate("~");

    //boost::shared_ptr<cslam::ServerSystem> pSSys{new cslam::ServerSystem(Nh,NhPrivate,argv[1])};
    pSSys.reset(new cslam::ServerSystem(Nh,NhPrivate,argv[1]));
    pSSys->InitializeClients();

    ros::Subscriber  sub=Nh.subscribe("/end",10,callback);
    ROS_INFO("started CSLAM server node...");

    ros::MultiThreadedSpinner MSpin(2);

    MSpin.spin();
    pSSys->save();

    ros::waitForShutdown();
    vector<boost::shared_ptr<cslam::KeyFrame>> curkf = pSSys->mpMap0->GetAllKeyFrames();
    ofstream keyframesFile; 
    ros::Time right_now = ros::Time::now();
   keyframesFile.open("/home/ccm/ccm.txt", ios::app);
    //f << setprecision(6) << right_now << setprecision(7)<<" "<<t[0]<<" "<<t[1]<<" "<<t[2]<<" "<<Q.w()<<" "<<Q.x()<<" "<<Q.y()<<" "<<Q.z()<<endl;
    for(auto &kf : curkf){
        boost::shared_ptr<cslam::KeyFrame> temp = kf;
        const double stamp = temp->mTimeStamp;
        const cv::Mat T_wc = temp->GetPoseInverse();
        const Eigen::Matrix4d eT_wc = cslam::Converter::toMatrix4d(T_wc);
        Eigen::Matrix4d T_SC;
        T_SC = temp->mT_SC;
        const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
        const Eigen::Quaterniond q(Tws.block<3,3>(0,0));
        keyframesFile << std::setprecision(11) << stamp << " ";
        keyframesFile << Tws(0,3) << " " << Tws(1,3) << " " << Tws(2,3) << " ";
        keyframesFile << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        cout<<"Tws-----------"<<endl;
    }
    keyframesFile.close();
    return 0;
}

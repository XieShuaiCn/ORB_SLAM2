/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void init(ros::NodeHandle nh);
    geometry_msgs::TransformStamped findTransform(string child, string parent);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    
    ros::Publisher c_pub; //camera_pose publisher
    ros::Publisher tv_pub;
    ros::Publisher tc_pub;
    
    cv::Mat pose;
    tf::TransformBroadcaster br;
    
    geometry_msgs::TransformStamped WtV;
    geometry_msgs::TransformStamped WtC;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

//my function for publishing various things (original)
void publish(cv::Mat toPublish, ros::Publisher Publisher, tf::TransformBroadcaster br, string parent, string child, bool publishTransform) {
    if (toPublish.empty()) {return;}
    
    cv::Mat Rotation = toPublish.rowRange(0,3).colRange(0,3); //getting rotation matrix
	cv::Mat Translation = toPublish.rowRange(0,3).col(3); //getting
	
	tf::Matrix3x3 RotationMatrix(Rotation.at<float>(0,0),Rotation.at<float>(0,1),Rotation.at<float>(0,2),
		                         Rotation.at<float>(1,0),Rotation.at<float>(1,1),Rotation.at<float>(1,2),
		                         Rotation.at<float>(2,0),Rotation.at<float>(2,1),Rotation.at<float>(2,2));

	tf::Vector3 TranslationVector(Translation.at<float>(0), Translation.at<float>(1), Translation.at<float>(2));
	
	//tf::Quaternion Quaternion;           
    //RotationMatrix.getRotation(Quaternion); //converting rotation matrix into quaternion
    
    tf::Transform TF_Transform = tf::Transform(RotationMatrix, TranslationVector); 
    
    if (publishTransform) {                             //change this to old frame time
	br.sendTransform(tf::StampedTransform(TF_Transform, ros::Time::now(), parent, child)); //sending TF Transform
    }
    
    geometry_msgs::PoseStamped MessageToPublish;
	MessageToPublish.pose.position.x = TF_Transform.getOrigin().x();
	MessageToPublish.pose.position.y = TF_Transform.getOrigin().y();
	MessageToPublish.pose.position.z = TF_Transform.getOrigin().z();
	MessageToPublish.pose.orientation.x = TF_Transform.getRotation().x();
	MessageToPublish.pose.orientation.y = TF_Transform.getRotation().y();
	MessageToPublish.pose.orientation.z = TF_Transform.getRotation().z();
	MessageToPublish.pose.orientation.w = TF_Transform.getRotation().w();

	MessageToPublish.header.stamp = ros::Time::now();
	MessageToPublish.header.frame_id = child;
	Publisher.publish(MessageToPublish); //publishing pose
}

void ImageGrabber::init(ros::NodeHandle nh)
{    
    //advertising my publishers
    c_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose",1000); //robot_pose == camera_optical_frame
    tv_pub = nh.advertise<geometry_msgs::TransformStamped>("error/transV",1000);
    tc_pub = nh.advertise<geometry_msgs::TransformStamped>("error/transC",1000);
    
    
    sub = nh.subscribe("/vicon/firefly_sbx/firefly_sbx", 1, &ImageGrabber::callback, this);
}

//finding transform between two frames to find rms error
geometry_msgs::TransformStamped ImageGrabber::findTransform(string child, string parent) {
    geometry_msgs::TransformStamped transformStamped;
    
    try{
      transformStamped = mpSLAM->mpTracker->tfBuffer.lookupTransform(parent, child, ros::Time(0));
      return transformStamped;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ROS_INFO("Transform Exception!");
      //return false; //this could be an issue
      return transformStamped;
    }

}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }
    
    // publishing bare bones
    
    
    // getting camera pose
    pubPose = true;
    if (pose.empty()) {pubPose = false;} //publishing camera_pose
    if (pubPose) {
    cv::Mat TWC = mpSLAM->mpTracker->mCurrentFrame.mTcw.inv();
    publish(TWC, c_pub, br, "init_link", "camera_optical_frame", true);
    }
    
    //////// getting rms error //////
    WtV = ImageGrabber::findTransform("vicon/firefly_sbx/firefly_sbx", "world");
    WtC = ImageGrabber::findTransform("camera_frame", "world");
    
    tv_pub.publish(WtV);
    tc_pub.publish(WtC);

}

//test..

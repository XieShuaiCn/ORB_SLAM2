/*
        Andre Ruas
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"System.h"
#include <iostream>
#include <sstream>
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TransformStamped.h"
#include <cmath>        // std::abs

using namespace std;

geometry_msgs::PoseStamped Vicon;
geometry_msgs::PoseStamped Pose;
double restartTimer;
double timeToWait = 150;
int cycles;

class ImageGrabber
{

public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void init(ros::NodeHandle nh);
    geometry_msgs::TransformStamped findTransform(string child, string parent);
    
    void callback(const geometry_msgs::TransformStamped& SubscribedTransform);
    void callback_fcu(const sensor_msgs::Imu fcu);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    
    //defining my publishers
    ros::Publisher c_pub; //camera_pose publisher
    ros::Publisher m_pub; //mVelocity (ORB_SLAM2 VO change) publisher
    ros::Publisher p_pub; //pVelocity (my IMU change) publisher
    ros::Publisher v_pub; //world->vicon absolute state publisher
    ros::Publisher i_pub; //vicon->init_link semi-static transform publisher
    ros::Publisher t_pub; //framerate publisher
    ros::Publisher tv_pub; 
    ros::Publisher tc_pub; 
    ros::Publisher tr_pub; //tracking publisher 
    ros::Publisher pt_pub;
    ros::Publisher tmm_pub;
    ros::Publisher mop_pub;
    ros::Publisher nl_pub; //number of tracking losses
    ros::Publisher tl_pub; //time spend lost
        
    ros::Subscriber sub;
    ros::Subscriber sub_fcu;

    bool pubPose;
    cv::Mat pose;
    bool sendTransform;
        
    tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped InitLink_to_World_Transform;
    
    double startTime;
    double loopTime;
    double rate;
    
    geometry_msgs::TransformStamped WtV; //world->vicon orientation
    geometry_msgs::TransformStamped WtC; //world->camera orientation
    geometry_msgs::TransformStamped WtI; //world->imu4 orientation
    geometry_msgs::TransformStamped WtFv; //world->fcu_vt orientation
    geometry_msgs::TransformStamped WtFo; //world->fcu_ot orientation
    geometry_msgs::TransformStamped WtO;  //world->Camera_optical_frame
    
    int lastTrackingState;
    bool resetBool;
    
    double trackingLossStart = 0;
    double trackingLossEnd = 0;
    double trackingLostTime = 0;
    double trackingLostTimeTotal = 0;
    int numTrackingLosses = 0;
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
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    
    igb.init(nh);
   
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    // Stop all threads
    SLAM.Shutdown();
    
    //removed saving camera trajectory from here

    ros::shutdown();
    return 0;
}

//my function for publishing various things (original)
void publish(cv::Mat toPublish, ros::Publisher Publisher, tf::TransformBroadcaster br, string parent, string child, bool publishTransform, ros::Time stamp) {
    if (toPublish.empty()) {return;}
    
    cv::Mat Rotation = toPublish.rowRange(0,3).colRange(0,3); //getting rotation matrix
	cv::Mat Translation = toPublish.rowRange(0,3).col(3); //getting 
	
	tf::Matrix3x3 RotationMatrix(Rotation.at<float>(0,0),Rotation.at<float>(0,1),Rotation.at<float>(0,2),
		                         Rotation.at<float>(1,0),Rotation.at<float>(1,1),Rotation.at<float>(1,2),
		                         Rotation.at<float>(2,0),Rotation.at<float>(2,1),Rotation.at<float>(2,2));

	tf::Vector3 TranslationVector(Translation.at<float>(0), Translation.at<float>(1), Translation.at<float>(2));
    
    tf::Transform TF_Transform = tf::Transform(RotationMatrix, TranslationVector); 
    
    if (publishTransform) {                             //change this to old frame time
	br.sendTransform(tf::StampedTransform(TF_Transform, stamp, parent, child)); //sending TF Transform
    }
    
    geometry_msgs::PoseStamped MessageToPublish;
	MessageToPublish.pose.position.x = TF_Transform.getOrigin().x();
	MessageToPublish.pose.position.y = TF_Transform.getOrigin().y();
	MessageToPublish.pose.position.z = TF_Transform.getOrigin().z();
	MessageToPublish.pose.orientation.x = TF_Transform.getRotation().x();
	MessageToPublish.pose.orientation.y = TF_Transform.getRotation().y();
	MessageToPublish.pose.orientation.z = TF_Transform.getRotation().z();
	MessageToPublish.pose.orientation.w = TF_Transform.getRotation().w();

	MessageToPublish.header.stamp = stamp;
	MessageToPublish.header.frame_id = child;
	Publisher.publish(MessageToPublish); //publishing pose
}

void ImageGrabber::callback(const geometry_msgs::TransformStamped& SubscribedTransform)
{
    Vicon.header = SubscribedTransform.header; //sending a transform between world and vicon/firefly_sbx/firefly_sbx
    Vicon.pose.position.x = SubscribedTransform.transform.translation.x; //this could be incorrect
    Vicon.pose.position.y = SubscribedTransform.transform.translation.y;
    Vicon.pose.position.z = SubscribedTransform.transform.translation.z;
    Vicon.pose.orientation = SubscribedTransform.transform.rotation;
    
    //here I am publishing the transform between Init_Link and World, this transform was originally hard-coded but is now automatically set
    // Init_Link is where ORB_SLAM creates its first keyframe, which in stereo is typically the first frame.
    if (sendTransform) {
        InitLink_to_World_Transform = SubscribedTransform;
        
        InitLink_to_World_Transform.header.frame_id = "world";
        InitLink_to_World_Transform.child_frame_id = "local";
        sendTransform = false;
    }
    
    InitLink_to_World_Transform.header.stamp = SubscribedTransform.header.stamp;
    br.sendTransform(InitLink_to_World_Transform); //sending semi-static TF Transform (represents world->init_link)
    
    i_pub.publish(InitLink_to_World_Transform); //publishing pose;
    
    v_pub.publish(Vicon); //publishing absolute pose;
	br.sendTransform(SubscribedTransform); //sending TF Transform (represents world->Vicon_pose)
}

void ImageGrabber::callback_fcu(sensor_msgs::Imu fcu)
{

    //geometry_msgs::TransformStamped WtFv; //world->fcu_vt orientation
    
    WtFv.header.frame_id = "world"; //sending a transform between world and fcu_vt
    WtFv.child_frame_id = "fcu_vt";
    WtFv.header.stamp = fcu.header.stamp;
    WtFv.transform.translation.x = Vicon.pose.position.x;
    WtFv.transform.translation.y = Vicon.pose.position.y;
    WtFv.transform.translation.z = Vicon.pose.position.z;
    WtFv.transform.rotation = fcu.orientation;
    
	br.sendTransform(WtFv); //sending TF Transform (represents world->fcu pose)
   
    WtO = ImageGrabber::findTransform("camera_optical_frame", "world");
    
    WtFo.header.frame_id = "world"; //sending a transform between world and fcu_ot
    WtFo.child_frame_id = "fcu_ot";
    WtFo.header.stamp = fcu.header.stamp;
    WtFo.transform.translation.x = WtO.transform.translation.x;
    WtFo.transform.translation.y = WtO.transform.translation.y;
    WtFo.transform.translation.z = WtO.transform.translation.z;
    WtFo.transform.rotation = fcu.orientation;
    
	br.sendTransform(WtFo); //sending TF Transform (represents world->fcu pose)
}


void ImageGrabber::init(ros::NodeHandle nh)
{   
    resetBool = false;
    sendTransform = true;
    lastTrackingState = -1;
    trackingLostTimeTotal = 0;
    
    //advertising my publishers
    c_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose",1000); //robot_pose == camera_optical_frame
    m_pub = nh.advertise<geometry_msgs::PoseStamped>("mVelocity",1000);
    p_pub = nh.advertise<geometry_msgs::PoseStamped>("pVelocity",1000);
    v_pub = nh.advertise<geometry_msgs::PoseStamped>("vicon/data",1000);
    i_pub = nh.advertise<geometry_msgs::TransformStamped>("vicon/init_link",1000);
    t_pub = nh.advertise<std_msgs::Float64>("diag/TrackTime",1000);
    tv_pub = nh.advertise<geometry_msgs::TransformStamped>("diag/transV",1000);
    tc_pub = nh.advertise<geometry_msgs::TransformStamped>("diag/transC",1000);
    tr_pub = nh.advertise<std_msgs::Int8>("diag/trackingState",1000);
    pt_pub = nh.advertise<std_msgs::Int8>("diag/pointsTracked",1000);
    tmm_pub = nh.advertise<std_msgs::Float64>("diag/TwMMtime",1000);
    mop_pub = nh.advertise<std_msgs::Bool>("diag/MoP",1000);
    tl_pub = nh.advertise<std_msgs::Float64>("diag/timeSpentLost",1000);
    nl_pub = nh.advertise<std_msgs::Int8>("diag/trackLossCount",1000);
    
    sub = nh.subscribe("/vicon/firefly_sbx/firefly_sbx", 1, &ImageGrabber::callback, this);
    sub_fcu = nh.subscribe("/fcu/imu", 1, &ImageGrabber::callback_fcu, this);
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
      return transformStamped;
    }

}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    startTime = ros::Time::now().toSec();
    
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
        pose = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
	pose =  mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }
    // Checking if tracking is lost ///
    if ((lastTrackingState == 2) && (mpSLAM->mpTracker->mState == 3)) {trackingLossStart = ros::Time::now().toSec();}
    
    // Checking if tracking is no longer lost
    if ((lastTrackingState == 3) && (mpSLAM->mpTracker->mState == 2)) 
    {
    trackingLossEnd = ros::Time::now().toSec();
    trackingLostTime = trackingLossEnd - trackingLossStart;
    trackingLostTimeTotal = trackingLostTimeTotal + trackingLostTime;
    numTrackingLosses = numTrackingLosses + 1;
    }
    lastTrackingState = mpSLAM->mpTracker->mState;
    
    
    //checking reset
    if (mpSLAM->mpTracker->resetBool) {sendTransform = true;}
    mpSLAM->mpTracker->resetBool = false;
    
    
    ////// Publishing pose, mVelocity, pVelocity ///////
    
    pubPose = true;
    if (pose.empty()) {pubPose = false;} //publishing camera_pose
    if (pubPose) {
    cv::Mat TWC = mpSLAM->mpTracker->mCurrentFrame.mTcw.inv();
    publish(TWC, c_pub, br, "init_link", "camera_optical_frame", true, msgLeft->header.stamp);
    
    //geometry_msgs::PoseStamped Pose; //saving pose to global variable so fcu can use it
    Pose.pose.position.x = TWC.rowRange(0,3).col(3).at<float>(0);
    Pose.pose.position.y = TWC.rowRange(0,3).col(3).at<float>(1);
    Pose.pose.position.z = TWC.rowRange(0,3).col(3).at<float>(2);
    }
    
    cv::Mat mVelocity = mpSLAM->mpTracker->mVelocity; //publishing mVelocity
    publish(mVelocity, m_pub, br, "", "imu4", false, msgLeft->header.stamp);
    
    cv::Mat pVelocity = mpSLAM->mpTracker->pVelocity; //publishing pVelocity
    publish(pVelocity, p_pub, br, "", "imu4", false, msgLeft->header.stamp); //use msgLeft->header.stamp for current time
    
    //////// getting rms error //////
    WtV = ImageGrabber::findTransform("vicon/firefly_sbx/firefly_sbx", "world");
    WtC = ImageGrabber::findTransform("camera_frame", "world");
    
    tv_pub.publish(WtV);
    tc_pub.publish(WtC);
    
    //publishing tracking state
    int ORBstate = mpSLAM->mpTracker->mState;
    std_msgs::Int8 ROSstate;
    ROSstate.data = ORBstate;
    tr_pub.publish(ROSstate);
    
    //calculating frame rate / delay and publishing it
    loopTime = ros::Time::now().toSec() - startTime;
    rate = loopTime;
    t_pub.publish(rate); //publishing frame rate of ORB_SLAM2
    
    //publishing number of tracked points
    int points = mpSLAM->mpTracker->mnMatchesInliers;
    std_msgs::Int8 ROSpts;
    ROSpts.data = points;
    pt_pub.publish(ROSpts);
    
    //calculating frame rate / delay and publishing it
    tmm_pub.publish(mpSLAM->mpTracker->TwMMtime); //publishing time it takes to track with motion model
    
    // publishing if we are using m or p velocity
    //mop_pub.publish(mpSLAM->mpTracker->usePvel);
    
    //publishing time spent lost
    tl_pub.publish(trackingLostTimeTotal);
    
    //publishing number of tracking losses
    std_msgs::Int8 losses;
    losses.data = numTrackingLosses;
    nl_pub.publish(losses);
   
    
    
} //end












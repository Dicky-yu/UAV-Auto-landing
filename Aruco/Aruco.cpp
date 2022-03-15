#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <chrono>
#include <iomanip>
#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <istream>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include "kinetic_math.hpp"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;


geometry_msgs::PoseStamped Aruco_pose_realsense;
static double fx, fy, cx, cy; //focal length and principal point
static Vec4 CamParameters;
cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);    //(rows, cols, type)
cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

double TimerLastT;
bool Aruco_init = false;
bool Aruco_found = false;

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg){
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
    cameraMatrix.at<double>(0,0) = fx;
    cameraMatrix.at<double>(1,1) = fy;
    cameraMatrix.at<double>(0,2) = cx;
    cameraMatrix.at<double>(1,2) = cy;
    CamParameters << fx,fy,cx,cy;
}
void Aruco_PosePub(Vec3 Aruco_xyz, Vec4 Aruco_Q){
    Aruco_pose_realsense.header.stamp = ros::Time::now();
    Aruco_pose_realsense.header.frame_id = "map";
    Aruco_pose_realsense.pose.position.x = Aruco_xyz(0);
    Aruco_pose_realsense.pose.position.y = Aruco_xyz(1);
    Aruco_pose_realsense.pose.position.z = Aruco_xyz(2);
    Aruco_pose_realsense.pose.orientation.w = Aruco_Q(0);
    Aruco_pose_realsense.pose.orientation.x = Aruco_Q(1);
    Aruco_pose_realsense.pose.orientation.y = Aruco_Q(2);
    Aruco_pose_realsense.pose.orientation.z = Aruco_Q(3);
}
void Aruco_process(Mat image_rgb){
    cv::Mat ArucoOutput = image_rgb.clone();
    std::vector<int> markerIds;
    std::vector<Vec8I> markerConerABCDs;
    Vec2I markerCenter,last_markerCenter;
    Vec8I markerConerABCD;
    Vec8I last_markerConerABCD;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<cv::Point2f> markerCorner;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Vec3d rvec, tvec;
    rvecs.clear();tvecs.clear();
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image_rgb, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    if (markerIds.size() > 0){
        markerConerABCDs.clear();
        Aruco_init = true;
        Aruco_found = true;
        cv::aruco::drawDetectedMarkers(ArucoOutput, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.045f, cameraMatrix, distCoeffs, rvecs, tvecs);
        for(unsigned int i=0; i<markerIds.size(); i++){
            cv::aruco::drawAxis(ArucoOutput, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            markerCorner = markerCorners[i];
            for (unsigned int j=0; j<markerCorner.size();j++){
                cv::Point2f MC = markerCorner[j];
                markerConerABCD[j*2] = MC.x;
                markerConerABCD[j*2+1] = MC.y;
            }
            markerConerABCDs.push_back(markerConerABCD);
        }
    }else{Aruco_found = false;}
    if (Aruco_init){
        if(Aruco_found){
            rvec = rvecs.front();
            tvec = tvecs.front();
            Quaterniond Arucoq;
            Arucoq = rpy2Q(Vec3(rvec[0],rvec[1],rvec[2]));
            Aruco_PosePub(Vec3(tvec(0),tvec(1),tvec(2)),Vec4(Arucoq.w(),Arucoq.x(),Arucoq.y(),Arucoq.z()));
        }
    }
    cv::imshow("Aruco", ArucoOutput);
    cv::waitKey(1);
}
void camera_rgb_cb(const sensor_msgs::CompressedImageConstPtr &rgb){
    /* Image initialize */
    cv::Mat image_rgb;
    try{
        image_rgb = cv::imdecode(cv::Mat(rgb->data),1);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Aruco_process(image_rgb);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "Aruco");
    ros::NodeHandle nh;
    //ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/color/camera_info",1,camera_info_cb);
    ros::Subscriber camera_rgb_sub = nh.subscribe<CompressedImage>("/camera/color/image_raw/compressed",1,camera_rgb_cb);
    ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ArucoPose",1);
    
    while(ros::ok()){
        ros::spinOnce();
        ArucoPose_pub.publish(Aruco_pose_realsense);
    }
    return 0;
}

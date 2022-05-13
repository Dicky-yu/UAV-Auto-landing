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
geometry_msgs::PoseStamped Arucow_pose;
static double fx, fy, cx, cy; //focal length and principal point
static Vec4 CamParameters;
cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);    //(rows, cols, type)
cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

double TimerLastT;
bool Aruco_init = false;
bool Aruco_found = false;


void Aruco_PosePub(Vec3 Aruco_xyz, Vec4 Aruco_Q)
{
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

void Arucow_PosePub(Vec3 world)
{
    Arucow_pose.header.stamp = ros::Time::now();
    Arucow_pose.header.frame_id = "world";
    Arucow_pose.pose.position.x = world(0);
    Arucow_pose.pose.position.y = world(1);
    Arucow_pose.pose.position.z = world(2);
}

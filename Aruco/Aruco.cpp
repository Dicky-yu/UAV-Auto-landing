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

#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include "kinetic_math.hpp"
#include "movement.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;


geometry_msgs::PoseStamped Aruco_pose_realsense;
geometry_msgs::PoseStamped Arucow_pose;
static double fx, fy, cx, cy; //focal length and principal point
static Vec4 CamParameters;
cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);  //(rows, cols, type)
cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

double ugvinfo_x,ugvinfo_y,ugvinfo_z,ugvinfo_ow,ugvinfo_ox,ugvinfo_oy,ugvinfo_oz;
//static UGVpose ugvinfo;

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
    Aruco_pose_realsense.pose.position.z = (Aruco_xyz(2)*2);
    Aruco_pose_realsense.pose.orientation.w = Aruco_Q(0);
    Aruco_pose_realsense.pose.orientation.x = Aruco_Q(1);
    Aruco_pose_realsense.pose.orientation.y = Aruco_Q(2);
    Aruco_pose_realsense.pose.orientation.z = Aruco_Q(3);
    //thie is a different Aruco_xyz, this Aruco_xyz is the "passed variable" from other place
}

void ugvbody_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)// in simulation, if your camera does not have a published that
                                                          // tells u where the camera it, you could just manually input these values (via the defined sdf files)
{
    ugvinfo_x = msg->pose[2].position.x;      // 1.0
    ugvinfo_y = msg->pose[2].position.y;      // 0
    ugvinfo_z = msg->pose[2].position.z;      // 0.12
    ugvinfo_ow = msg->pose[2].orientation.w;
    ugvinfo_ox = msg->pose[2].orientation.x;
    ugvinfo_oy = msg->pose[2].orientation.y;
    ugvinfo_oz = msg->pose[2].orientation.z;
}

void Arucow_PosePub(Vec3 world)
{
    Arucow_pose.header.stamp = ros::Time::now();
    Arucow_pose.header.frame_id = "world";
    Arucow_pose.pose.position.x = world(0);
    Arucow_pose.pose.position.y = world(1);
    Arucow_pose.pose.position.z = world(2);   // +0.16

}

void c2w_process(Eigen::Matrix<double, 3, 1> Aruco_xyz)  //here, the function get a variable call Aruco_xyz
                                                         //but it is just a name, there is no value, it is just a "shell"

                                                         //as I mentioned, when u are only using camera, dun need body frame
{                                                        // just express the tranformation directly from camera to world
    cout <<"camera to world frame" <<endl;

     double x = Aruco_xyz(0); // camera coordinate x
     double y = Aruco_xyz(1); // camera coordinate y
     double z = (Aruco_xyz(2)*2); // the distance between center of the object and camera

     Eigen::Matrix<double, 4, 1> camera (x,y,z,1), body, world, offset (0, -0.17,3.14,0);

     Eigen::Matrix<double, 4, 4> camera_to_body;  //(i,j,k,1)
     camera_to_body << 0, -0.173648, -0.984808, 0.16,
                       1, 0, 0, 0,
                       0, -0.984808, 0.173648, 0.078,
                       0, 0, 0, 1;


     Eigen::Matrix<double, 3, 3> matrix_for_q;
     Eigen::Quaterniond q2r_matrix(ugvinfo_ow, ugvinfo_ox, ugvinfo_oy, ugvinfo_oz);
     matrix_for_q = q2r_matrix.toRotationMatrix();

     Eigen::Matrix<double, 4, 4> body_to_world;
     body_to_world << matrix_for_q(0,0), matrix_for_q(0,1), matrix_for_q(0,2), (ugvinfo_x+1.0),
                      matrix_for_q(1,0), matrix_for_q(1,1), matrix_for_q(1,2), ugvinfo_y,
                      matrix_for_q(2,0), matrix_for_q(2,1), matrix_for_q(2,2), ugvinfo_z,
                      0, 0, 0, 1;

     cout<<body_to_world<<endl;

//   body_coord = matrix_camera_to_body * camera_coord + offset; //(i,j,k,1) from camera coordinate to body coordinate

     world = body_to_world * camera_to_body * camera;  //(u,v,w,1) from body coordinate to world coordinate
//   waypts world_pt = {world(0), world(1), world(2)};

//   return world_pt;      // actually you could define this function as "void", no need to return, cos the final "arucow_pose" is a global variable

     Arucow_PosePub(Vec3(world(0), world(1), world(2)));

     cout << "processing" <<endl;
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

    Eigen::Matrix<double, 3, 1> Aruco_xyz;  // the aruco output
    Aruco_xyz(0) = tvec(0);
    Aruco_xyz(1) = tvec(1);
    Aruco_xyz(2) = tvec(2);

    //when u initiate a variable in a function, it is only "local", other function will not know
    //so even u have the same name of a variable somewhere else, there are all different
    //thie aruco_xyz is only for "here"
    c2w_process(Aruco_xyz);   // transformation function -> passing opencv output to c2w_process

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
//  ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/color/camera_info",1,camera_info_cb);
    ros::Subscriber camera_rgb_sub = nh.subscribe<CompressedImage>("/camera/color/image_raw/compressed",1,camera_rgb_cb);
    ros::Subscriber ugvbody_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",10, ugvbody_cb);
    ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ArucoPose",1);
    ros::Publisher ArucowPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ArucowPose",1);
    ros::Publisher arucofound_pub = nh.advertise<std_msgs::Bool>("/aruco_found",1);
    
//  ros::Rate rate(30);
    while(ros::ok()){
        ArucoPose_pub.publish(Aruco_pose_realsense);
        ArucowPose_pub.publish(Arucow_pose);

        std_msgs::Bool aruco_found;
        if(Aruco_found)
          aruco_found.data = true;
        else
          aruco_found.data = false;
        arucofound_pub.publish(aruco_found);

        ros::spinOnce();// when you spinOnce, ROS will call all the callback function: subscriber=>camera_info_cb && camera_rgb_cb
    }
    return 0;
}


//spinOnce -> camera_rgb_cb -> aruco_process -> transformation process -> Aruco_PosePub: datatype from vec3 to geometry_msgs::PoseStamped -> then back to main while loop -> publisher

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numeric>

#include <fstream>     //file
#include <iostream>    //input/output
#include <istream>
#include <sstream>     //string
#include <chrono>      //time
#include <iomanip>     //manipulator
#include <string>
#include <algorithm>
#include <vector>
#include "kinetic_math.hpp"

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

static cv::Mat frame, res;     //create camera frame

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
    rvecs.clear();
    tvecs.clear();
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

    }
    cout << "Get image data" << endl;
    Aruco_process(image_rgb);
}

int main(int argc, char** argv)   //argument count, argument value
{
     int stateSize = 6;   //[x,y,z,v_x,v_y,v_z]
     int measSize = 3;    //[z_x,z_y,z_z]
     int contrSize = 0;

     unsigned int type = CV_32F;   //32-bit floating-point no.
     cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

     cv::Mat state(stateSize, 1, type);
     cv::Mat measure(measSize, 1, type);
     //cv::Mat processNoise(stateSize, 1, type)


     //Transition State Matrix A
     // Note: set dT at each processing step!
     // [ 1 0 0  dT 0  0  ]
     // [ 0 1 0  0  dT 0  ]
     // [ 0 0 1  0  0  dT ]
     // [ 0 0 0  1  0  0  ]
     // [ 0 0 0  0  1  0  ]
     // [ 0 0 0  0  0  1  ]
     cv::setIdentity(kf.transitionMatrix);


     // Measure Matrix H
     // [ 1 0 0 0 0 0 ]
     // [ 0 1 0 0 0 0 ]
     // [ 0 0 1 0 0 0 ]

     cv::setIdentity(kf.measurementMatrix);
     kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
     kf.measurementMatrix.at<float>(0) = 1.0f;
     kf.measurementMatrix.at<float>(7) = 1.0f;
     kf.measurementMatrix.at<float>(14) = 1.0f;


     // Process Noise Covariance Matrix Q
     // [ Ex  0   0    0     0     0    ]
     // [ 0   Ey  0    0     0     0    ]
     // [ 0   0   Ez   0     0     0    ]
     // [ 0   0   0    Ev_x  0     0    ]
     // [ 0   0   0    0     Ev_y  0    ]
     // [ 0   0   0    0     0     Ev_z ]

     cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));

     kf.processNoiseCov.at<float>(0) = 1e-2;
     kf.processNoiseCov.at<float>(7) = 1e-2;
     kf.processNoiseCov.at<float>(14) = 1e-2;
     kf.processNoiseCov.at<float>(21) = 5.0f;
     kf.processNoiseCov.at<float>(28) = 5.0f;
     kf.processNoiseCov.at<float>(35) = 5.0f;

     // Measurement Noise Covariance Matrix R
     cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

     // Post Error Convariance P
     cv::setIdentity(kf.errorCovPost, cv::Scalar(1));

 // <<<< Kalman Filter


 // <<<< Camera settings
     int idx = 0;
     //cv::VideoCapture cap;     //Camera capture
     //cap = VideoCapture(".mp4");

     //if(!cap.read(frame) || key == 27)
     //{
         //cerr << "Unable to read next frame." << endl;
         //cerr << "Exiting..." << endl;
         //exit(EXIT_FAILURE);
     //}
     //cap.set(CV_CAP_PROP_FRAME_WIDTH, 840);
     //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 640);


     ros::init(argc, argv, "Aruco");
     ros::NodeHandle nh;   //starting a node

     message_filters::Subscriber<sensor_msgs::CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
     message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
     typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
     message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
     sync.registerCallback(boost::bind(&camera_rgb_cb, _1, _2));

     ros::Subscriber camera_info_sub = nh.subscribe("camera/aligned_depth_to_color/camera_info", 1, camera_info_cb);
     ros::Subscriber camera_rgb_sub = nh.subscribe<CompressedImage>("/camera/color/image_raw/compressed",1,camera_rgb_cb);
     ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ArucoPose", 1);


     //
     bool found = false;
     bool measured = false;
     int notFoundCount = 0;
     double dT;
     double ticks = 0;
     int i=0;

     cv::Point center_true;
     cv::Point center_pred;
     double depth = 0, depth_ = 0;
     double fps;
     double fps_average = 0;
     double time_start, time_end;

     cv::Rect predRect;
     cv::Rect tempRect;

     while(ros::ok())
     {
         time_start = ros::Time::now().toSec();
         if(!frame.empty())
         {
             double precTick = ticks;
             double ticks = cv::getTickCount();

             dT = (ticks - precTick) / cv::getTickFrequency(); //seconds


             if (found)
             {
                 // Transition Matrix A
                 kf.transitionMatrix.at<float>(3) = dT;
                 kf.transitionMatrix.at<float>(10) = dT;
                 kf.transitionMatrix.at<float>(17) = dT;

                 // <<<< Matrix A
                 cout << "dT:" << endl << dT << endl;

        // Prediction
                 state = kf.predict();
                 // cout << "State post:" << endl << state << endl;

        // Print predict box

                 predRect.x = state.at<float>(0) - predRect.width / 2;
                 predRect.y = state.at<float>(1) - predRect.height / 2;
                 predRect.width = tempRect.width;
                 predRect.height = tempRect.height;

                 cv::Point center;
                 center.x = state.at<float>(0);
                 center.y = state.at<float>(1);
                 double z_center = state.at<float>(2);


                 //save<<counter<<endl;
                 //save<<predRect.x <<endl;
                 //save<<predRect.y <<endl;
                 //save.close();


                 //cv::Scalar color(rand()&255, rand()&255, rand()&255);
                 cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
                 center_pred = cv::Point(predRect.x+predRect.width/2, predRect.y+predRect.height/2);

                 //char temp_depth[40];
                 //sprintf(temp_depth, "%.2f", z_c_temp);
                 //string d = "Predicted z_C: ";
                 //string textoutputonframe =d+temp_depth + " m";
                 //cv::Point placetext = cv::Point((predRect.x-10),(predRect.y+predRect.height+24));
                 //cv::putText(res, textoutputonframe, placetext,cv::FONT_HERSHEY_COMPLEX_SMALL,1,CV_RGB(255,0,0));
                 depth_ = z_center;
             }



      // Kalman Update
             bool got = false;

             time_end=ros::Time::now().toSec();
             cout <<"ms: " << time_end-time_start << endl;
             if(markerIds.size() = 0)     // not detect the Aruco marker
             {
                 notFoundCount++;
                 measured = false;
                 cout << "notFoundCount:" << notFoundCount << endl;
                 if(notFoundCount>100)
                 {
                     found = false;
                 }
             }
             else
             {

                 measured = true;
                 notFoundCount = 0;
                 measure.at<float>(0) = Aruco_pose_realsense.pose.position.x;
                 measure.at<float>(1) = Aruco_pose_realsense.pose.position.y;
                 measure.at<float>(2) = Aruco_pose_realsense.pose.position.z;

                 if (!found) // First detection!
                 {
                     // >>>> Initialization
                     kf.errorCovPre.at<float>(0) = 1; // px
                     kf.errorCovPre.at<float>(7) = 1; // px
                     kf.errorCovPre.at<float>(14) = 1;
                     kf.errorCovPre.at<float>(21) = 1;
                     kf.errorCovPre.at<float>(28) = 1; // px
                     kf.errorCovPre.at<float>(35) = 1; // px


                     state.at<float>(0) = measure.at<float>(0);
                     state.at<float>(1) = measure.at<float>(1);
                     state.at<float>(2) = measure.at<float>(2);
                     state.at<float>(3) = 0;
                     state.at<float>(4) = 0;
                     state.at<float>(5) = 0;
                     // <<<< Initialization

                     kf.statePost = state;  //corrected state

                     found = true;
                 }
                 else
                 {
                     kf.correct(measure);  // Kalman Correction

                    // cout << "Measure matrix:" << meas << endl;

                     cv::Point center;
                     center.x = state.at<float>(0);
                     center.y = state.at<float>(1);
                     double z_center = state.at<float>(2);

                     cv::Rect Rect;
                     Rect.width = tempRect.width;
                     Rect.height = tempRect.height;
                     Rect.x = state.at<float>(0) - Rect.width / 2;
                     Rect.y = state.at<float>(1) - Rect.height / 2;
                     center_true=cv::Point(Rect.x+Rect.width/2, Rect.y+Rect.height/2);

                     cv::rectangle(res, Rect, CV_RGB(0,255,0), 1);
                 }

             }
             cv::Mat display;

             geometry_msgs::PointStamped send;
             if(measured)
             {
                 cout<<"show measure: "<<endl;
                 send.point.x = center_true.x;
                 send.point.y = center_true.y;
                 send.point.z = depth;
             }
             else
             {

                 cout<<"show predict"<<endl;
                 cv::rectangle(res, predRect, CV_RGB(255,0,0), 1);
                 send.point.x = center_pred.x;
                 send.point.y = center_pred.y;
                 send.point.z = depth_;
             }

             cv::imshow("KF", res);
             cv::waitKey(20);    // waits for x milliseconds for a key press on a OpenCV window

         }
         ros::spinOnce();
         ArucoPose_pub.publish(Aruco_pose_realsense);
     }
     ros::spin();
     return 0;
}

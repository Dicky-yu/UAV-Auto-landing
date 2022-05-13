#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>

using namespace std;

void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ofstream save("/home/yu/aruco_FYP/src/Aruco/src/log/uav_pose.txt",ios::app);
    save<<"Time:"<<ros::Time::now()<<endl;
    save<<"x:"<<pose->pose.position.x<<endl;
    save<<"y:"<<pose->pose.position.y<<endl;
    save<<"z:"<<pose->pose.position.z<<endl;
    save.close();
}

void arucocp_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ofstream save("/home/yu/aruco_FYP/src/Aruco/src/log/arucocp.txt",ios::app);
    save<<"Time:"<<ros::Time::now()<<endl;
    save<<"x:"<<pose->pose.position.x<<endl;
    save<<"y:"<<pose->pose.position.y<<endl;
    save<<"z:"<<pose->pose.position.z<<endl;
    save<<"orientation w:"<<pose->pose.orientation.w<<endl;
    save<<"orientation x:"<<pose->pose.orientation.x<<endl;
    save<<"orientation y:"<<pose->pose.orientation.y<<endl;
    save<<"orientation z:"<<pose->pose.orientation.z<<endl;
    save.close();
}

void arucowp_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    ofstream save("/home/yu/aruco_FYP/src/Aruco/src/log/arucowp.txt",ios::app);
    save<<"Time:"<<ros::Time::now()<<endl;
    save<<"x:"<<pose->pose.position.x<<endl;
    save<<"y:"<<pose->pose.position.y<<endl;
    save<<"z:"<<pose->pose.position.z<<endl;
    save<<"orientation w:"<<pose->pose.orientation.w<<endl;
    save<<"orientation x:"<<pose->pose.orientation.x<<endl;
    save<<"orientation y:"<<pose->pose.orientation.y<<endl;
    save<<"orientation z:"<<pose->pose.orientation.z<<endl;
    save.close();
}

void ugvbody_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    ofstream save("/home/yu/aruco_FYP/src/Aruco/src/log/ugvpose.txt",ios::app);
    save<<"Time: "<<ros::Time::now()<<endl;
    save<<"x:"<<msg->pose[2].position.x <<endl;
    save<<"y:"<<msg->pose[2].position.y <<endl;
    save<<"z:"<<msg->pose[2].position.z <<endl;
    save<<"orientation w:"<<msg->pose[2].orientation.w <<endl;
    save<<"orientation x:"<<msg->pose[2].orientation.x <<endl;
    save<<"orientation y:"<<msg->pose[2].orientation.y <<endl;
    save<<"orientation z:"<<msg->pose[2].orientation.z <<endl;
    save.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "info_recorder");
    ros::NodeHandle nh;

    ros::Subscriber uav_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                   ("/mavros/local_position/pose", 1, uav_pose_cb);
    ros::Subscriber arucocp_info_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                       ("/ArucoPose", 1, arucocp_cb);
    ros::Subscriber arucowp_info_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                        ("/ArucowPose", 1, arucowp_cb);
    ros::Subscriber ugvbody_sub = nh.subscribe<gazebo_msgs::ModelStates>
                                   ("/gazebo/model_states",100, ugvbody_cb);

    ros::Rate rate(20.0);
    while(ros::ok())
    {
      cout << "recording" <<endl;

      ros::spinOnce();
      rate.sleep();
    }
    ros::spin();
    return 0;
}

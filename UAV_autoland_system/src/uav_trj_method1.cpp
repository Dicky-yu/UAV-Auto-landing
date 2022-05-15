#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/Thrust.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include "std_msgs/String.h"
#include "utils/yamlRead.h"
#include "utils/eigen_typedef.h"
#include "movement/generalmove.h"
#include "movement/circletrj.h"
#include "aruco detection.h"


#define PI (3.1415926)

using namespace std;

enum Mission_STATE {
    IDLE,
    TAKEOFFP1,
    TAKEOFFP2,
    DETECTION,
    APPROACH1,
    APPROACH2,
    RECOVERY,
    LAND
} mission_state=IDLE;

static mavros_msgs::State current_state;
static mavros_msgs::AttitudeTarget UAV_AttitudeTarget;
double uav_lp_x,uav_lp_y,uav_lp_z;
double uav_lp_qx,uav_lp_qy,uav_lp_qz,uav_lp_qw;
double ugvlp_x,ugvlp_y,ugvlp_z,ugvlp_ow,ugvlp_ox,ugvlp_oy,ugvlp_oz;
double uavposition_x,uavposition_y,uavposition_z;

double angle_xy;
double distance_uavugv, distance_x;
double arucocp_x,arucocp_y,arucocp_z,arucocp_ow,arucocp_ox,arucocp_oy,arucocp_oz;
double arucowp_x,arucowp_y,arucowp_z,arucowp_ow,arucowp_ox,arucowp_oy,arucowp_oz;

double height, length, speed, frequency;
bool   force_start;
bool   shut_down = true;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void uav_lp_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    ofstream save("/home/gh034/Aruco/src/aruco_scan/src/log/uav_pose.txt",ios::app);
    uav_lp_x = pose->pose.position.x;
    uav_lp_y = pose->pose.position.y;
    uav_lp_z = pose->pose.position.z;
    uav_lp_qx = pose->pose.orientation.x;
    uav_lp_qy = pose->pose.orientation.y;
    uav_lp_qz = pose->pose.orientation.z;
    uav_lp_qw = pose->pose.orientation.w;
    save<<ros::Time::now() << ";" << pose->pose.position.x << ";" << pose->pose.position.y << ";" << pose->pose.position.z <<endl;
}

void ugv_lp_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    ofstream save("/home/gh034/Aruco/src/aruco_scan/src/log/ugv_pose.txt",ios::app);
    ugvlp_x = msg->pose[2].position.x+1;      // 1.0
    ugvlp_y = msg->pose[2].position.y;      // 0
    ugvlp_z = msg->pose[2].position.z;      // 0.12
    ugvlp_ow = msg->pose[2].orientation.w;
    ugvlp_ox = msg->pose[2].orientation.x;
    ugvlp_oy = msg->pose[2].orientation.y;
    ugvlp_oz = msg->pose[2].orientation.z;
    save<<ros::Time::now() << ";" << ugvlp_x << ";" << msg->pose[2].position.y << ";" << msg->pose[2].position.z <<endl;
}

void arucopose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    arucocp_x = pose->pose.position.x;
    arucocp_y = pose->pose.position.y;
    arucocp_z = pose->pose.position.z;
    arucocp_ow = pose->pose.orientation.w;
    arucocp_ox = pose->pose.orientation.x;
    arucocp_oy = pose->pose.orientation.y;
    arucocp_oz = pose->pose.orientation.z;
}

void arucowpose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    ofstream save("/home/gh034/Aruco/src/aruco_scan/src/log/arucowp.txt",ios::app);
    arucowp_x = pose->pose.position.x;
    arucowp_y = pose->pose.position.y;
    arucowp_z = pose->pose.position.z;
    arucowp_ow = pose->pose.orientation.w;
    arucowp_ox = pose->pose.orientation.x;
    arucowp_oy = pose->pose.orientation.y;
    arucowp_oz = pose->pose.orientation.z;
    save<<ros::Time::now() << ";" << pose->pose.position.x << ";" << pose->pose.position.y << ";" << pose->pose.position.z <<endl;
}

static bool scanornot;
void arucofound_cb(const std_msgs::Bool::ConstPtr& msg){
    scanornot = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh("~");

    cout  << "get parameter:" << endl;
    nh.getParam("force_start", force_start);
    cout << force_start << endl;
    nh.getParam("height", height);
    height = 1.2;                           // 1.2 meter
    cout << "height:" << height << endl;
    nh.getParam("speed", speed);
    speed = 1.0;                            // 1.0 degrees per second
    cout << "speed :" << speed << endl;
//    nh.getParam("length", length);
//    length = 4.0;
//    cout << "length:" << length << endl;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 1, state_cb);
    ros::Subscriber uavposlp_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 1, uav_lp_cb);
    ros::Subscriber arucofound_sub = nh.subscribe<std_msgs::Bool>("/aruco_found", 1, arucofound_cb);
    ros::Subscriber ArucoPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/ArucoPose", 1, arucopose_cb);
    ros::Subscriber ArucowPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/ArucowPose", 1, arucowpose_cb);
    ros::Subscriber ugvlp_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",100, ugv_lp_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Publisher uav_attitudetarget = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(40.0);   // max 50 hz
    frequency = 40;
    //wait for FCU connection
    if(force_start)
    {
        cout << "force start " << endl;
    }
    else
    {
        cout << "Waiting for FCU connection " << endl;
        while(ros::ok() && !current_state.connected){
            ros::spinOnce();
            rate.sleep();
            cout << "Waiting for FCU connection " << endl;
        }
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x=0.0;
    pose.pose.orientation.y=0.0;
    pose.pose.orientation.z=0.0;
    pose.pose.orientation.w=1.0;

    //send a few setpoints before starting
    if(force_start)
    {
        cout << "force start " << endl;
    }
    else
    {
        for(int i = 100; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    cout << "change last_request A" << endl;
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        //update the uav position
        //from motion capture system
        //local postion estimator(in simulation platform)
        uavposition_x = uav_lp_x;
        uavposition_y = uav_lp_y;
        uavposition_z = uav_lp_z;

        angle_xy = atan((ugvlp_y - arucowp_y)/(ugvlp_x - arucowp_x));
        distance_uavugv = sqrt(pow((ugvlp_y - arucowp_y),2)+pow((ugvlp_x - arucowp_x),2));

        /*offboard and arm*****************************************************/
        if(force_start)
        {
            static bool once=true;
            if(once)
            {
                mission_state = TAKEOFFP1;
                last_request = ros::Time::now();
                cout << "force start the mission " << endl;
                once = false;
            }
        }
        else
        {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
            {
                if( set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                        (ros::Time::now() - last_request > ros::Duration(1.0))){
                    if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                        mission_state = TAKEOFFP1;
                    }
                    last_request = ros::Time::now();
                }
            }
        }

        /*takeoff*****************************************************/
        //PLEASE DEFINE THE LANDING PARAMETER HERE
        if(mission_state==TAKEOFFP1)
        {
            static generalMove takeoff1(ros::Time::now().toSec(),
                                        0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, height-0.5,0.0,
                                        (height-0.5)/0.1);
            takeoff1.getPose(ros::Time::now().toSec(),pose);
            if(takeoff1.finished())
            {
                cout << "Takeoff P1 finished" << endl;
                mission_state = TAKEOFFP2;
                last_request = ros::Time::now();
            }
        }
        if(mission_state==TAKEOFFP2)
        {
            static generalMove takeoff2(ros::Time::now().toSec(),
                                        0.0, 0.0, height-0.5,0.0,
                                        0.0, 0.0, height,0.0,
                                        5);
            takeoff2.getPose(ros::Time::now().toSec(),pose);
            if(takeoff2.finished())
            {
                mission_state = DETECTION;
                cout << "Takeoff P2 finished" << endl;
                last_request = ros::Time::now();
            }
        }

        if(mission_state==DETECTION)
        {
            if(ros::Time::now()-last_request > ros::Duration(1.0));
            {
              if(scanornot == true)
              {
                 mission_state = APPROACH1;
                 cout << "Hover finished" << endl;
                 last_request = ros::Time::now();
              }
              else {
                 static generalMove straight_path(ros::Time::now().toSec(),
                                                      uav_lp_x, uav_lp_y, uav_lp_z,0.0,
                                                      (uav_lp_x+0.05), uav_lp_y, (uav_lp_z-0.3),0.0, // (uav_lp_x+arucocp_z-0.1),(arucowp_z-0.1) OK
                                                      2);
                 straight_path.getPose(ros::Time::now().toSec(),pose);
                 if(straight_path.finished())
                 {
                     mission_state = APPROACH1;
                     cout << "start approaching" << endl;
                     cout << uav_lp_x << endl;
                     cout << uav_lp_z << endl;
                 }
              }
            }
        }

        if(mission_state==APPROACH1)
        {
          if(scanornot == true)
          {
                pose.pose.position.x = arucowp_x + distance_uavugv*(cos(angle_xy))*(speed/frequency);
                pose.pose.position.y = arucowp_y + distance_uavugv*(sin(angle_xy))*(speed/frequency);
                pose.pose.position.z = 0.6;
                //pose.pose.position.x = arucowp_x + 0.1;
                //pose.pose.position.y = arucowp_y + vector_y*(sin(angle_xy))*(speed/frequency);
                //pose.pose.position.z = arucowp_z - 0.2;
                cout << "approaching" << endl;
                cout << "distance" << arucocp_z << endl;
                cout << arucowp_z << endl;
                if(arucowp_z <= 0.3)
                {
                     mission_state = APPROACH2;
                     cout << "enter horizontal flight" << endl;
                }

                if(arucocp_z< 0.25 && arucowp_z <=0.26)
                {
                    mission_state = LAND;
                    cout << "landing" << endl;
                    cout << arucocp_z << endl;
                }
           }
        }
        if(mission_state==APPROACH2)
          {
              if(scanornot == true)
              {
                  pose.pose.position.x = arucowp_x + distance_uavugv*(cos(angle_xy))*(speed/frequency);    // 1.5 no ground effect
                  pose.pose.position.y = arucowp_y + distance_uavugv*(sin(angle_xy))*(speed/frequency);
                  pose.pose.position.z = 0.25;

                  cout << "flying horizontally........" << endl;
                  cout << arucocp_z << endl;
                  cout << arucowp_z << endl;
                  if(arucocp_z< 0.25 && arucowp_z <=0.26)
                  {
                      mission_state = LAND;
                      cout << "landing" << endl;
                      cout << arucocp_z << endl;
                      cout << arucowp_z << endl;
                  }
              }
              else{
                  if(ros::Time::now()-last_request > ros::Duration(6.0))
                  {
                    mission_state=RECOVERY;
                    cout << "too low" << endl;
                  }
              }
         }

        if(mission_state==RECOVERY)   // not detect aruco
        {
             pose.pose.position.x = arucowp_x + distance_uavugv*(cos(angle_xy))*(speed/frequency);
             pose.pose.position.y = arucowp_y + distance_uavugv*(sin(angle_xy))*(speed/frequency);
             pose.pose.position.z = arucowp_z + 0.2;
             if(arucowp_z >= 0.4){
                  mission_state = APPROACH1;
                  cout << "reached 0.5m or above in height" << endl;
                  cout << arucocp_z << endl;
                  cout << arucowp_z << endl;
             }
        }

        if(mission_state==LAND)
        {
            if(shut_down)
            {
              UAV_AttitudeTarget.thrust = 0;
              uav_attitudetarget.publish(UAV_AttitudeTarget);
              return 0;
            }

        }
        //cout  << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << endl;

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

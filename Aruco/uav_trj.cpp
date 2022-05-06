#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
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
    APPROACH,
    ARRIVE,
    HOVER,
    LANDING,
    END,
} mission_state=IDLE;

mavros_msgs::State current_state;
double uav_lp_x,uav_lp_y,uav_lp_z;
double uav_lp_qx,uav_lp_qy,uav_lp_qz,uav_lp_qw;
double uavposition_x,uavposition_y,uavposition_z;

double arucocp_x,arucocp_y,arucocp_z,arucocp_ow,arucocp_ox,arucocp_oy,arucocp_oz;
double arucowp_x,arucowp_y,arucowp_z,arucowp_ow,arucowp_ox,arucowp_oy,arucowp_oz;

double height, length, speed;

bool   force_start;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void uav_lp_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    uav_lp_x = pose->pose.position.x;
    uav_lp_y = pose->pose.position.y;
    uav_lp_z = pose->pose.position.z;
    uav_lp_qx = pose->pose.orientation.x;
    uav_lp_qy = pose->pose.orientation.y;
    uav_lp_qz = pose->pose.orientation.z;
    uav_lp_qw = pose->pose.orientation.w;
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
    arucowp_x = pose->pose.position.x;
    arucowp_y = pose->pose.position.y;
    arucowp_z = pose->pose.position.z;
    arucowp_ow = pose->pose.orientation.w;
    arucowp_ox = pose->pose.orientation.x;
    arucowp_oy = pose->pose.orientation.y;
    arucowp_oz = pose->pose.orientation.z;
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
    height = 1.0;                           // meter
    cout << "height:" << height << endl;
    nh.getParam("speed", speed);
    speed = 0.3;                            // degrees per second
    cout << "speed :" << speed << endl;
    nh.getParam("length", length);
    length = 4.0;
    cout << "length:" << length << endl;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber uavposlp_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, uav_lp_cb);
    ros::Subscriber arucofound_sub = nh.subscribe<std_msgs::Bool>("/aruco_found", 1, arucofound_cb);
    ros::Subscriber ArucoPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/ArucoPose", 1, arucopose_cb);
    ros::Subscriber ArucowPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/ArucowPose", 1, arucowpose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    //triggle (vio platform test)
    //    ros::Publisher tri_start_pub = nh.advertise<std_msgs::String>("tri_start", 10);
    //    ros::Publisher tri_end_pub = nh.advertise<std_msgs::String>("tri_end", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
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
                mission_state = APPROACH;
                cout << "Takeoff P2 finished" << endl;
                last_request = ros::Time::now();
            }
        }

        if(mission_state==APPROACH)
        {
          if(scanornot == true){
            if(arucocp_z > 0.6 && arucowp_z > 0.4)  // approaching UGV       //arucowp_z > 0.4
            {
                static generalMove straight_path(ros::Time::now().toSec(),
                                                    uav_lp_x, uav_lp_y, uav_lp_z,0.0,
                                                    (uav_lp_x+arucocp_z), uav_lp_y, (arucowp_z-0.1),0.0, // (uav_lp_x+arucocp_z-0.1),(arucowp_z-0.1) OK
                                                    (arucocp_z/speed));

                straight_path.getPose(ros::Time::now().toSec(),pose);
                cout << "approaching" << endl;
                cout << arucocp_z << endl;
                cout << arucowp_z << endl;

                //last_request = ros::Time::now();
            }

            else if(arucowp_z<=0.4 && arucocp_z<=0.6)            // < 0.33 & <= 0.2 is okay
            {
                if(arucowp_z<=0.3 && arucocp_z<0.5)     // 0.12<=arucowp_z<=0.25
                {
                     mission_state = LANDING;
                     cout << "ready to land" << endl;
                     cout << arucocp_z << endl;
                     cout << arucowp_z << endl;
                }
                else
                {
                    static generalMove straight_path(ros::Time::now().toSec(),
                                                         uav_lp_x, arucowp_y, arucowp_z,0.0,
                                                         (uav_lp_x+arucocp_z+0.2), arucowp_y, (arucowp_z-0.3),0.0,
                                                         4);
                    straight_path.getPose(ros::Time::now().toSec(),pose);
                    cout << "almost arrive" << endl;
                    cout << arucocp_z << endl;
                    cout << arucowp_z << endl;
                }
            }
          }
//            else
//            {
//                static generalMove straight_path(ros::Time::now().toSec(),
//                                                    uav_lp_x, arucowp_y, arucowp_z,0.0,
//                                                    (uav_lp_x+0.2), arucowp_y, arucowp_z-0.05,0.0,
//                                                    2);
//                straight_path.getPose(ros::Time::now().toSec(),pose);
//                cout << "holdsssssssss" << endl;

//            }

//          else   // not detect aruco
//          {
//                if(ros::Time::now()-last_request > ros::Duration(6.0))
//                {
//                    static generalMove straight_path(ros::Time::now().toSec(),
//                                                        uav_lp_x, uav_lp_y, uav_lp_z,0.0,
//                                                        uav_lp_x, 0.0, (uav_lp_z+0.6),0.0,
//                                                        3);
//                    straight_path.getPose(ros::Time::now().toSec(),pose);
//                    if(straight_path.finished())
//                    {
//                        mission_state = APPROACH;
//                        cout << "too low" << endl;
//                        last_request = ros::Time::now();
//                    }
//                }
//          }

//            if(scanornot == false && uav_lp_z < 0.3)  // UAV too low while aruco not scanned
//            {
//                static generalMove straight_path(ros::Time::now().toSec(),
//                                                    uav_lp_x, uav_lp_y, uav_lp_z,0.0,
//                                                    uav_lp_x, uav_lp_y, (uav_lp_z+0.2),0.0,
//                                                    5);
//                straight_path.getPose(ros::Time::now().toSec(),pose);
//                cout << "too low !!!" << endl;
//            }


        }

//        if(mission_state==HOVER)
//        {
//            if(ros::Time::now()-last_request > ros::Duration(1.0))
//            {
//                mission_state = LANDING;
//                cout << "Hover finished" << endl;
//                last_request = ros::Time::now();
//            }
//        }

        //PLEASE DEFINE THE LANDING PARAMETER HERE
        if(mission_state==LANDING)
        {
            //double secs = (ros::Time::now() - last_request).toSec();
            //cout << secs << endl;
            //pose.pose.position.z = 0.05-(secs)*0.1;
            if(arucowp_z < 0.3)
            {
                arucowp_z=0.3;
                arm_cmd.request.value = false;
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success)
                {
                    mission_state=END;
                    cout << "Landing P2 finished" << endl;
                    return 0;//break the control UAV will land automatically
                }
            }
        }

        if(mission_state==END)
        {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = -0.3;
            return 0;
        }
        //cout  << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << endl;

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

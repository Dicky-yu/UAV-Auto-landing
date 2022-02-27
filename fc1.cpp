#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include "mavros_msgs/PositionTarget.h" // Mavros topic to control vel and pos
#include "object_detector/States.h" // Custom msgs of type States contain predicted position and yaw angle
#include "drone_controller/Error.h" // Custom msgs of type Error
#include <mavros_msgs/CommandTOL.h> // Service for landing


int    FSM_state = 0;
int    Mission_state = 0;
int    Mission_stage = 0;
int    Current_Mission_stage = 0;
bool   FSMinit = false;
bool   Force_start   = false;
bool   ShutDown = false;


static mavros_msgs::State current_state;
static mavros_msgs::PositionTarget position_pub;
static mavros_msgs::mavros_msgs::CommandTOL land_client;
//static <class> sub subscribe to the predicted location

//cb function for uav pose
void uav_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UAV_pose_sub.pose.position.x = pose->pose.position.x;
    UAV_pose_sub.pose.position.y = pose->pose.position.y;
    UAV_pose_sub.pose.position.z = pose->pose.position.z;
    UAV_pose_sub.pose.orientation.w = pose->pose.orientation.w;
    UAV_pose_sub.pose.orientation.x = pose->pose.orientation.x;
    UAV_pose_sub.pose.orientation.y = pose->pose.orientation.y;
    UAV_pose_sub.pose.orientation.z = pose->pose.orientation.z;
    UAV_lp << UAV_pose_sub.pose.position.x,UAV_pose_sub.pose.position.y,UAV_pose_sub.pose.position.z,
              UAV_pose_sub.pose.orientation.w,UAV_pose_sub.pose.orientation.x,UAV_pose_sub.pose.orientation.y,UAV_pose_sub.pose.orientation.z;
}

void subcallback(const class::type) //class and type pub from the object detect
{
    //distance between UAV and the predicted location
    float dx= pose.position.x - predeicted.x
    float dy= pose.position.y - predeicted.y
    float dz= pose.position.z - predeicted.z
    float dtheta= pose.theta - predeicted.theta
}
void uav_state_sub(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int zini = pose.position.z

void Finite_stage_mission(){  // Main FSM
    //if (current_state != )
    if (dx&&dy<=0.5){ //close to the target
        pose.position.z = zini - 0.1;
    }
    else {
        zpos = zini
    }
    if ()



int main(int argc, char *argv)
{
    ros::inti(argc, argv, "nodename");
    ros::NodeHandle nh;
    //sub kalman filter output
    ros::Subscriber sub = nh.subscribe<class>("topic", 10, subcallback)
    //sub uav state
    ros::Subscriber state_sub = nh.subscribe<mavros_msg::State>("/mavros/state", 10, uav_state_sub)
    //sub uav pose
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, uav_pose_sub);
    //pub target position msg
    ros::Publisher uav_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::ServiceClient land_client = po_nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    mavros_msgs::SetMode offb_set_mode,posctl_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

   while(ros::ok()){
        if (!System_init){
        System_initT = ros::Time::now().toSec();
        init_time = ros::Time::now();
        cout << " System Initialized" << " Force_start: " << Force_start << endl;
        System_init = true;
        }

        /* offboard and arm ****************************************************/
        if((ros::Time::now() - init_time < ros::Duration(5.0))){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
        //Set Offboard trigger duration here
        uav_pos_pub.publish(UAV_pose_pub);
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
        }else{
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
        }
        }
        }

        // Position target object to publish
        mavros_msgs::PositionTarget pos;

        //FRAME_LOCAL_NED to move WRT to body_ned frame
        pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

        pos.header.stamp = ros::Time::now(); // Time header stamp
        pos.header.frame_id = "base_link"; // "base_link" frame to compute odom
        pos.type_mask = 1987; // Mask for Vx, Vy, Z pos and Yaw rate
        pos.position.z = predicted.z;
        pos.position.x = predicted.x;
        pos.position.y = predicted.y;
        //pos.yaw_rate = predicted.vthe;

        //printf("Proportional Vx, Vy, Vthe and Zpos values at (%f,%f,%f,%f) \n", vx, vy, vthe, zpos);
        uav_pos_pub.publish(pos);



        /* FSM initailize */
        if (current_state.mode == "OFFBOARD" && current_state.armed && !FSMinit){
            FSMinit = true;
            Mission_stage = 1;
            UAV_takeoffP = UAV_lp;
            cout << "UAV_takeoff_Position: " << UAV_takeoffP[0] << " " << UAV_takeoffP[1] << " " << UAV_takeoffP[2] << endl;
            cout << "Mission stage = 1 Mission start!" <<endl;
        }
        if (Force_start){
            FSMinit = true;
            Mission_stage = 1;
            UAV_takeoffP = UAV_lp;
            cout << "UAV_takeoff_Position: " << UAV_takeoffP[0] << " " << UAV_takeoffP[1] << " " << UAV_takeoffP[2] << endl;
            cout << "------------------Dangerous!------------------" << endl;
            cout << "Mission stage = 1 Mission start!" << endl;
            Force_start = false;
        }
        /* FSM *****************************************************************/
        Finite_stage_mission();
        uav_pub(pub_trajpose,pub_pidtwist);
        if(ShutDown){
            cout << "Vehicle Soft Shut Down" << endl;
            pub_pidtwist = false;
            pub_trajpose = false;
            UAV_AttitudeTarget.thrust -= 0.005;
            cout << "thrust: " << UAV_AttitudeTarget.thrust << endl;
            uav_AttitudeTarget.publish(UAV_AttitudeTarget);
        }
        //if(pub_pidtwist ||ForcePIDcontroller){uav_vel_pub.publish(UAV_twist_pub);}
        //if(pub_trajpose&&!ForcePIDcontroller){uav_pos_pub.publish(UAV_pose_pub);}
        /*Mission information cout**********************************************/
        if(coutcounter > 50 && FSMinit && !ShutDown){ //reduce cout rate

            cout << "Status: "<< armstatus() << "    Mode: " << current_state.mode <<endl;
            cout << "---------------------------------------------------" << endl;
            coutcounter = 0;
        }else{coutcounter++;}
        /* ROS timer */
        // auto currentT = ros::Time::now().toSec();
        // cout << "System_Hz: " << 1/(currentT-LastT) << endl;
        // LastT = currentT;
        ros::spinOnce();
        loop_rate.sleep();
        }
    }

}



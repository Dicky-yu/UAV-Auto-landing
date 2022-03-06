#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include "mavros_msgs/PositionTarget.h" // Mavros topic to control vel and pos
#include "object_detector/States.h" // Custom msgs of type States contain predicted position and yaw angle
#include "drone_controller/Error.h" // Custom msgs of type Error
#include <mavros_msgs/CommandTOL.h> // Service for landing

bool objectdetector(){
    if (Aruco_pose_raelsense.pose.position.x && Aruco_pose_raelsense.pose.position.y && Aruco_pose_raelsense.pose.position.z && !=0){
        return 1
    }
    else {
        return 0
    }
}

void systemstate (){
    if (ros::ok & !System_init){
        mission_stage = 0 //takeoff
    }
    if (objectdetector()){ //recieved target position
        mission_stage = 1 //
        pub.publish(Aruco_pose_realsense)
    }
    if (UAV_pose_sub.pose.position.x-Aruco_pose_realsense.pose.position.x<=0.5 && UAV_pose_sub.pose.position.y-Aruco_pose_realsense.pose.position.y<=0.5 ){
        mission_stage = 2  //decending

    }
    if (UAV_pose_sub.pose.position.z <= 0.5){
        mission_stage = 3 //landing
        mavros_msgs::CommandTOL land_cmd; // Set all the descend parameters to Zero
        land_cmd.request.yaw = 0;
        land_cmd.request.latitude = 0;
        land_cmd.request.longitude = 0;
        land_cmd.request.altitude = 0;
        // When it lands, everything goes to zero
        if (!(land_client.call(land_cmd) && land_cmd.response.success))
        {
            // Publish the service of landing
            ROS_INFO("Landing");
            // Print final Error
            printf("Error at Vx, Vy, Theta and Z are (%f,%f,%f,%f) \n", ErX, ErY, ErTheta, ErZ);
            pub1.publish(er);
            ros::shutdown(); // Shutdowm the node
        }
        }

    }


}

# PolyU FYP AAE03b

## Antonomous Landing of UAV on Mobile Platform

This project aims to develop a simple, low-cost, and advanced autonomous landing system for the UAV without relying on GNSS and onboard sensors.


### Clone the work
1. Clone our repository

   cd ~/xxx/src

   git clone https://github.com/Dicky-yu/UAV-Auto-landing.git

2. Compile

   cd ~/xxx

   catkin_make

3. Launch our world

   ./sim.sh
   
4. Launch aruco detection module

   cd ~/UAV_autoland_system
   
   source devel/setup.bash
   
   rosrun fyp Aruco
   
5. Launch uav control module
   
   rosrun fyp uav_trj_method1 / rosrun fyp uav_trj_method2
   
6. Run generic keyboard teleop for moving robot (UGV)
   
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py

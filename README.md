# PolyU FYP AAE03b

## Antonomous Landing of UAV on Mobile Platform

This project aims to develop a simple, low-cost, and advanced autonomous landing system for the UAV without relying on GNSS and onboard sensors. An ArUco marker, which is a simple, low-cost fiducial marker, is used for visual-based object detection. Moreover, compared to the studies which have already been proposed, the stereo camera which works as the sensor in our study is installed on the mobile platform instead of the UAV. This alleviates the burden of UAV during the landing process. Our novel vision-based position estimation method is integrated with a path planning algorithm to generate trajectory and perform autonomous landing.


### Clone the work
1. Clone our repository
   ```
   cd ~/xxx/src

   git clone https://github.com/Dicky-yu/UAV-Auto-landing.git
   ```
2. Compile
   ```
   cd ~/xxx

   catkin_make
   ```
3. Launch our world
   ```
   ./sim.sh
   ```
4. Launch aruco detection module
   ```
   cd ~/UAV_autoland_system
   
   source devel/setup.bash
   
   rosrun fyp Aruco
   ```
5. Launch uav control module
   ```
   rosrun fyp uav_trj_method1   // method 1 
   
   rosrun fyp uav_trj_method2   // method 2
   ```
6. Run generic keyboard teleop for moving robot (UGV)
   ```
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```

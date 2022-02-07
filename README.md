# permobil_arm
## Installation
This project is contains new content for the integration project permobil_arm_meta (https://github.com/Permobil-Senior-Design/permobil_arm_meta)
## Launch Files
Loads the xarm and rviz with the Moveit task constructor enabled
```
roslaunch permobil_arm xarm_rviz.launch
```
Simulates the pickup of a cylinder using the GPD
```
roslaunch permobil_arm gpd_demo.launch
```
Loads the xarm in a gazebo simulation with the cylinder
```
roslaunch permobil_arm gazebo_spawn_object_xarm.launch
```
Given a gazebo simulation with depth camera, collects a pointcloud image 
```
roslaunch permobil_arm collect_pcd.launch
```
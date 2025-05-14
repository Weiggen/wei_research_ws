https://github.com/qq44642754a/Yolov8_ros

1. Environment
* Single Target 
```
roslaunch formation_ctrl test_formation.launch
```
* Multi-targets
```
roslaunch formation_ctrl 2T3R.launch
```
2. Record the bag
* Single Target 
```
rosbag record -O [record_name] /iris_1/THEIF/Plot /iris_1/utility /iris_2/utility /iris_3/utility /iris_1/mavros/setpoint_velocity/cmd_vel /iris_2/mavros/setpoint_velocity/cmd_vel /iris_3/mavros/setpoint_velocity/cmd_vel /iris_1/local/position /iris_2/local/position /iris_3/local/position
```
* Multi-targets
```
rosbag record -O [record_name] /iris_1/TEIF/target_1/Plot /iris_1/TEIF/target_2/Plot /iris_1/utility /iris_2/utility /iris_3/utility /iris_1/mavros/setpoint_velocity/cmd_vel /iris_2/mavros/setpoint_velocity/cmd_vel /iris_3/mavros/setpoint_velocity/cmd_vel /iris_1/local/position /iris_2/local/position /iris_3/local/position
```
3. Control
```
roslaunch voronoi_cbsa CBSA.launch
```

4. Three agents takeoff & offboard
```
rosrun formation_ctrl cmd_node
```
switch to offboard mode:
```
z \\arm the uavs which in our team.
1 \\takeoff
3 \\offboard
```
5. Estimation
```
roslaunch state_estimation consensusEstimation.launch
```
6. Moving Target
* Single Target 
```
roslaunch formation_ctrl target_kb_ctrl.launch
>> 3 \\ takeoff
>> o \\ move to given height(position)
>> e \\ move along the given trajectory
```
* Multi-targets
```
rosrun voronoi_cbsa control_target_1.py 
>> s
```
```
rosrun voronoi_cbsa control_target_2.py 
>> s
```

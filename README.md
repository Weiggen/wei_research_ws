https://github.com/qq44642754a/Yolov8_ros

# Run Simulation
1. Open the environment and the turtleBots in gazebo.
```
roslaunch voronoi_cbsa 2T3R_turtle.launch 
```
2. Record the ros bag
```
cd ~/wei_research_ws/src/voronoi_cbsa/bag
rosbag record -O [file name] /tb_1/TEIF/target_1/Plot /tb_1/TEIF/target_2/Plot /tb_1/utility /tb_2/utility /tb_3/utility /tb_1/cmd_vel /tb_2/cmd_vel /tb_3/cmd_vel /iris_1/local/position /tb_2/local/position /tb_3/local/position

```
3. Estimation nodes
```
roslaunch state_estimation consensusEstimation.launch
```
4. Agents' control node and the real-time pygame image
```
roslaunch voronoi_cbsa CBSA_tb.kaunch
```

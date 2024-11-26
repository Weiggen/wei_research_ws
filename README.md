# Run Simulation
1. Open the environment and the drones in gazebo.
```
roslaunch formation_ctrl test_formation.launch
```
2. UAVs' control node
```
roslaunch voronoi_cbsa CBSA.kaunch
```
3. Our agents(3 on the outside) flight mode control node
```
rosrun formation_ctrl cmd_node
```
4. Estimation nodes
```
roslaunch state_estimation consensusEstimation.launch
```
5. Target's flight mode switching & motion control nodes
```
roslaunch formation_ctrl target_kb_ctrl.launch
```

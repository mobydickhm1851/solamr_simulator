# SOLamr Simulator
This package is aimed to develope SOLamr in Gazebo with the following feature : 

1. [Auto Connector](https://hackmd.io/i2IS9tFnQKqGmrkJv2_ePQ)  
2. [Shelft Finder](./src/solamr_pkgs/src/ObjectRecognition.md)
3. Linked Drive controller (Right below)

## Simulation Environment
### Large shelft (XL)
Up till now, three cooperative models have been constructed and tested, which are:
- Skid Drive SOLamr : SOLamrs are fixed in position relative to the shelft
- Linked Drive SOLamr : SOLamrs are connected to the shelft by rods, free rotation relative to the shelft is allowed.

Since Skid Drive SOLamr is not suit for heavy loading (friction on the wheels are too large) and the control scheme is also not robust, Skid Drive SOLamr is depricated. 
If you still want to launch it, launch this file:

```
roslaunch amr_gazebo solamr_skid_shelftx.launch
```

Otherwise, to launch the more preferable version of SOLamr, you can either launch 

```
roslaunch amr_gazebo solamr_linked_shelftx.launch
```

or launch 

```
roslaunch amr_gazebo solamr_linked_2in1.launch
```

The main difference is that for `solamr_linked_shelftx.launch`, we can test **auto connection** function, which will allow SOLamr to find the designated shelft (S or XL) and to connect with them automatically. However, due to the physics model in Gazebo, force transimission by means of contacts can easily failed and result in weired action (e.g. jumping).

In `solamr_linked_2in1.launch`, this problem is solved by using continuous joints to connect SOLamr and the shelft. But in return, SOLamr and the shelft are nor seperable.

### Small Shelft (S)
To simulate SOLamr carrying a small shelft (only available in `solamr_skid_shelftx`), switch the argument `shelft_type` from *shelft_XL* (big shelft with two SOLamr) to *shelft_S*.

### No shelft two SOLamr
To launch only two SOLamr, simply 

```
roslaunch amr_gazebo solamr_only.launch
```
This launch is also suggessted to use if testing Linked Drive Controller

### Linked Drive Controller
Linked Drive Controller controls the cmd_vel of follower SOLamr. 

- To test Linked Drive Controller in rviz, launch :
   ```
   roslaunch linked_diff_drive linked_drive_demo.launch
   ```
   Arrows representing leading and following SOLamr will be shown in rviz.
   | ![](https://i.imgur.com/40wr3Gp.gif) | red box:<br>Reachable poses<br>selected from<br>potential poses| 
   | -------- | -------- |

- To impelement Linked Drive Controller in Gazebo, launch : 
   ```
   roslaunch amr_gazebo solamr_only.launch
   ```
   To spawn two SOLamr, one as leading vehicle, one as following vehicle.
   Leading vehicle can be controlled in the following two ways:
   1. Controlling using keyboard (teleop_key.py)
      ```
      roslaunch amr_gazebo linked_drive_error_test.launch
      ```
      This launch file will initiate these 3 nodes (mecel. not listed):
      - Keyboard teleop (leading vehicle)
      - Linked Drive Controller (following vehicle)
      - Error estimation (pyplot animation)
      
   2. Controlling using DWA
      ```
      roslaunch amr_gazebo nav_core.launch
      ```
      This launch file will initiate these nodes (mecel. not listed):
      - Move base (leading vehicle : Global planner / DWA local planner)
      - Linked Drive Controller (following vehicle)
      - Error estimation (pyplot animation)

For more detailed explanation and insight for Linked Drive Controller, please see [Linked Drive Controller note](https://hackmd.io/7LaMkuFtQnqDLp_9-IAt6w?both)

### factory environment
To employ the factory-like environment, add the following argument while launching Gazebo `empty_world.launch`

```
<arg name="world_name" value="$(find amr_gazebo)/worlds/factory_sim.world"/>
```

## Keyboard/Joystick Control
To run teleop control node 
```
rosrun amr_gazebo teleop_key_2in1.py
```

```
Control Your SOLamr!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
Block for connection : t/b for CW/CCW
Switch between solamr 1 / solamr 2 
control both using same command : 1 / 2 / 3
CTRL-C to quit

credit: Turtlebot
```

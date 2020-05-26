# SOLamr Simulator
This package is aimed to develope SOLamr in Gazebo with the following feature : 

1. [Auto Connector](https://hackmd.io/i2IS9tFnQKqGmrkJv2_ePQ)  
2. [Shelft Finder](./src/solamr_pkgs/src/ObjectRecognition.md)
3. Navigation

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

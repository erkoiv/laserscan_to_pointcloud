### Installation of base BARN dataset benchmarking stack

This package is written in Python 3.8.10 for ROS Noetic and Ubuntu 20.04 LTS.

### Clone Jackal package sources 
```
cd catkin_ws/src
git clone https://github.com/jackal/jackal.git
git clone https://github.com/jackal/jackal_simulator.git
git clone https://github.com/jackal/jackal_desktop.git
git clone https://github.com/ros-visualization/interactive_marker_twist_server.git
```

### Install dependencies with `rosdep` : 
```
cd ..
rosdep install --from-paths . --ignore-src
```

### Build and source workspace

```
catkin_make
source devel/setup.bash
```

### Clone BARN dataset time trial package

```
cd src
git clone https://github.com/dperille/jackal_timer
```

### Build and source workspace again

```
cd ..
catkin_make
source devel/setup.bash
```

Make sure ```catkin_ws/src/jackal_timer/scripts/run_trials.py``` is executable:
```
chmod +x run_trials.py
```

In ```catkin_ws/src/jackal/jackal_navigation/launch/include/amcl.launch``` file make the following changes:

- Under ```<arg name="scan_topic" default="$(eval optenv('JACKAL_LASER_TOPIC', 'front/scan'))" />``` add

```
<arg name="initial_pose_x" default="0.0" />
<arg name="initial_pose_y" default="0.0" />
<arg name="initial_pose_a" default="0.0" />
```

- Replace the three lines under ```<!-- Initial pose mean -->``` with
```
<param name="initial_pose_x" value="$(arg initial_pose_x)" />
<param name="initial_pose_y" value="$(arg initial_pose_y)" />
<param name="initial_pose_a" value="$(arg initial_pose_a)" />
```

In order to make the ```time_trial``` package work with ROS Noetic, in ```catkin_ws/src/jackal_timer/scripts/traversal_timer.py``` 

replace 

```!/usr/bin/env python``` 

with 

```!/usr/bin/env python3```

### Download dataset

Download the BARN dataset with:
```wget https://www.cs.utexas.edu/~xiao/BARN/BARN_dataset.zip```

Create a folder ```data``` in ```catkin_ws/src/jackal_timer``` and extract the contents of the download directly into it.

**The default stack for the BARN dataset time trials should now be installed**

You can test it by opening two terminal windows and starting ```roscore``` in one of them.

In the other one navigate into the ```catkin_ws/src/jackal_timer/scripts``` folder and run

```python3 run_trials.py```

**NOTE** that due to the way University of Texas has structured their stack, exiting the ```run_trials.py``` program via Ctrl+C will usually freeze the terminal, and the terminal window must be force closed instead. ```roscore``` can be kept running and any finished time trial data will still be available in the ```time_results.txt``` file in the package root folder. Otherwise the program will keep running until all time trial worlds have been finished and will then auto-exit.

### Installing the ```laserscan_to_pointcloud``` package

Navigate to ```catkin_ws/src``` and clone the package:

```
git clone https://github.com/erkoiv/laserscan_to_pointcloud
cd ..
catkin_make
```

Make sure the ```laser_assembler``` package is installed:

```sudo apt-get install ros-noetic-laser-assembler```

Open ```catkin_ws/src/jackal_timer/launch/time_trial.launch```.

To the end of the file before ```</launch>``` add:

```
  <!-- laserscan to pointcloud -->
  <include file="$(find laserscan_to_pointcloud)/launch/start.launch" />
```

This will allow the package to start automatically when ```run_trials.py``` is executed.

Replace the line

```
<param name="base_local_planner" value="base_local_planner/TrajectoryPlannerROS">
```

with

```
<param name="base_local_planner" value="">
```

This will disable the default local planner. An error message will be shown at launch, but this can be ignored.

Now when the simulation launches, the robot will not move, because it is not receiving velocity commands.

### Modifying ```laserscan_to_pointcloud``` node to send velocity commands

Open ```catkin_ws/src/laserscan_to_pointcloud/scripts/node.py```.

You will see a section to initialize the planner before calling it.

The laserscan data comes in the form of an array of ROS ```point``` messages, which are sets of x, y, and z coordinates. This array is deconstructed into two arrays containing the x and y coordinates respectively. ```x[0]``` will correspond to ```y[0]``` and so forth.

In order to drive the robot the planner must output at least a linear (forward) speed, ```vel_cmd.linear.x``` and an angular speed ```vel_cmd.angular.z```. 

These are then published to the robot.

Because Python is used instead of C++, ```catkin_make``` does not need to be called after each change to the file.
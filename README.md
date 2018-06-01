# Self Driving Robot - NCTU ARG

## Hardware

|Branch | Status |
|-------|--------|
|IMU |[Sparkfun 9Dof Razor IMU|
|GPS|[G-STAR IV (BU-353S4)|

### IMU


### GPS


## Installation

### package

```
$ sudo apt-get install ros-kinetic-gazebo-ros-*
```

## Function

### pre-work

```
$ cd gazebo_test/catkin_ws
$ source environment.sh
```
Note:
Do it everytime as you open new terminals


### Pure Pursuit

```
Open gazebo with one mobile robot
$ roslaunch launch_gazebo duckiebot.launch 

Run pure pursuit algorithm
$ roslaunch pure_pursuit pure_pursuit.launch lookahead:=0.5

Open Rviz to see the odometry
$ roslaunch pure_pursuit viz.launch 
```
Note:
To change the path, please modify the following file

gazebo_test/catkin_ws/src/pure_pursuit/config/path/path.yaml

### Obstacle Avoidance

```
Open gazebo with one mobile robot & Point cloud clustering
$ roslaunch launch_gazebo obstacle.launch

Obstacle coordinate processing
$ roslaunch launch_gazebo coordinat_tf.launch

Building global map (Not done yet)
$ rosrun coordinate map.py

Run RRT algorithm
$ rosrun rrt rrt.py

Run pure pursuit algorithm
$ roslaunch pure_pursuit pure_pursuit_gps.launch lookahead:=0.8
```

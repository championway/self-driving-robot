# Self Driving Robot - NCTU ARG

|Branch | Developer |
|-------|--------|
|Master |NCTU ARG Lab|
|devel-david|[David Chen](https://github.com/championway)|
|devel-sean|[Sean Lu](https://github.com/seanNCTU)|

# Requirements

- ROS kinetic (Ubuntu 16.04)
- PCL (Point Cloud Library)

# Hardware

## Sensor

### IMU
- Sparkfun 9Dof Razor IMU

### GPS
- G-STAR IV (BU-353S4) 

### LIDAR
- Velodyne VLP-16 

# Installation

## Package

```
$ sudo apt-get install ros-kinetic-gazebo-ros-*
```

# How to build

```
$ cd
$ git clone https://github.com/championway/self-driving-robot
$ cd ~/self-driving-robot/catkin_ws
$ source /opt/ros/kinetic/setup.bash
$ catkin_make --pkg robotx_msgs
$ catkin_make
```
Note:
Do the following everytime as you open new terminals

```
$ cd self-driving-robot/catkin_ws
$ source environment.sh
```


## Kinematics

### Joystick control

```
```

### Wheel odometry

```
```

## Path Planning

### RRT

```
```

### Pure Pursuit

```
```
## Perception

### Point Cloud

```
```
### Mapping

```
```
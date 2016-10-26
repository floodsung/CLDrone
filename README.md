# CLDrone:Autonomous Quadrotor Simulation Research Platform

![Snapshot 1](snapshot1.jpg)
![Snapshot 2](snapshot2.jpg)

This is a project based on Pixhawk SITL, DRCSim and apriltag. The goal is to create a convenient simulation platform to test intelligent algorithms on quadrotors. It consists of a quadrotor, a vehicle with apriltag and a drcsim environment. The quadrotor is equiped with two cameras, one forward camera and one downward camera.



## Related Project

- [Apriltags](https://github.com/personalrobotics/apriltags)
- [PX4](https://github.com/PX4/Firmware)
- [DRCsim](https://bitbucket.org/osrf/drcsim)

## Test Platform

- Ubuntu 14.04 
- ROS indigo
- Gazebo 4 

## Installation



```
# install prerequisite
## install mavros but from shadow repo to get latest version earlier
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu/ trusty main" > /etc/apt/sources.list.d/ros-shadow.list'
sudo apt-get update
sudo apt-get -y install ros-indigo-mavros ros-indigo-mavros-extras

## install control_toolbox
sudo apt-get install ros-indigo-control-toolbox

# install the git
mkdir -p quadrotor_platform/src
cd quadrotor_platform/src
catkin_init_workspace
git clone https://github.com/songrotek/CLDrone-Autonomous-Quadrotor-Simulation-Research-Platform.git
cd ..
catkin_make 
```

## How to work

```
# init simulation platform
roslaunch px4 gazebo_px4_polaris_tracker.launch

# init vehicle control
rosrun rqt_robot_steering rqt_robot_steering

# init image viewer
rosrun rqt_image_view rqt_image_view

```

## How to develop

### About control
There is a rostopic for the quadrotor's velocity control using body coordinate:
**/CLDrone/body_axis_velocity/cmd_vel**

you can make a rosnode to publish quadrotor's velocity to this topic to control the quadrotor.

### About Camera View

The camera topics are:
/front_cam/camera/image
/downward_cam/camera/image

it is easy to access the image and do any processing such as object detection.

## Lisense
BSD Lisense


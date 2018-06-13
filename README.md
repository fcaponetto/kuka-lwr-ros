
# kuka-lwr-ros
[![Build Status](https://travis-ci.com/fcaponetto/kuka-lwr-ros.svg?token=d6T2YmvspfmdmqrMCGYv&branch=master)](https://travis-ci.com/fcaponetto/kuka-lwr-ros/branches)

A ROS package to control the KUKA LWR 4 (both simulation and physical robot). **This was originally forked from** [CentroEPiaggio/kuka-lwr](https://github.com/CentroEPiaggio/kuka-lwr). and **at a later time** from [epfl-lasa/kuka-lwr](https://github.com/epfl-lasa/kuka-lwr-ros)


# Installation
Do the following steps:
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ catkin_init_workspace
$ git clone https://github.com/fcaponetto/kuka-lwr-ros.git
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```
$  wstool init
$  wstool merge kuka-lwr-ros/dependencies.rosinstall 
$  wstool up 
```
* Query and installs all libraries and packages 
```
$ rosdep install --from-paths . --ignore-src --rosdistro kinetic 
```

* Install [**Gazebo**](http://gazebosim.org/), follow this [**link**](http://gazebosim.org/tutorials?tut=install_ubuntu&) for 
instructions on how to install it on ubuntu. Make sure that the ros libraries of Gazebo are also installed:
```
$ sudo apt-get install ros-kinetic-gazebo7-*
```

## Build System

* catkin is the ROS build system to generate executables, libraries, and interfaces
* We suggest to use the Catkin Command Line Tools.  

Use **catkin build** instead of **catkin_make**.  
Navigate to your catkin workspace with:
```
> cd ~/catkin_ws
```
Build specific package with:
```
> catkin build package_name 
```
Or all packages with:
```
> catkin build
```
Whenever you build a new package, update your environment:
```
> source devel/setup.bash
```
If necessary, clean the entire build and devel space with:
```
> catkin clean
```

## Clion integration
Create a CMakelists.txt in catkin_ws folder in oder to include the src subfolder:
```
project(kuka)
cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 14)

#TODO find a way to link the executable directly to standard ros folder instead that cmake-build-debug (Clion defauld output)
#set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/../install")
#set(CATKIN_DEVEL_PREFIX "${CMAKE_SOURCE_DIR}/../devel")

add_subdirectory(src)
```

# Description

Set of packages for simulating and controlling the KUKA Light Weight Robot (LWR).


* [**kuka_lwr**](https://github.com/fcaponetto/kuka-lwr-ros/tree/master/kuka_lwr) contains URDF robot description, hardware interface, controllers with configuration files.

* [**lwr_ros_client**](https://github.com/fcaponetto/kuka-lwr-ros/tree/master/lwr_ros_client) basic implemenation of action handling such that it is easy to call different types of policies.

* [**robot_motion_generation**](https://github.com/fcaponetto/kuka-lwr-ros/tree/master/robot_motion_generation)  utilities such as filters for smoothing robot motion.

* [**kuka-lwr-ros-examples**](https://github.com/fcaponetto/kuka-lwr-ros-examples) set of examples.


# Quick Start (Simulation)
Download the [**kuka-lwr-ros-examples**](https://github.com/fcaponetto/kuka-lwr-ros-examples) into your catkin_ws and 
after compling them are ready to run the lwr_simple_example. 

Open a new terminal and run the following:
```sh
$ roslaunch lwr_simple_example sim.launch
```
This will run the simulator and the Gazebo simulator and ROS Rviz visualiser GUIs should both open. If the Gazebo 
window does not open this is because a flag is set in the sim.launch file. In the
caption below Rviz is on the left and Gazebo is on the right.

![alt text](readme/gazebo_rviz.png "Gazebo and Rviz GUIs")

Now that the simulations are up and running we are ready to control the robot.
In on terminal run the following:
```sh
$ roslaunch lwr_simple_example client.launch
```
and in another:
```sh
$ roslaunch lwr_simple_example console.launch
```

You should have the following triptych view in your console
![alt text](readme/console.png "Triptych console view")

Notice on the bottom right console the heading is "KUKA PLANNING INTERFACE" and a prompt **Cmd>**. This is
the main interface from which you will be starting and stopping policies to be run on the robot. If you
press tab (in this console window) a list of possible actions (robot policies) will be displayed, which in the
simple example case are; **go_front**, **go_home**,
**go_left**, **grav_comp** and **linear** (note that grav_comp only works on the real physical robot).

# Quick start (real robot)
You first have to make sure that your network configurations are set correctly and you can ping the robot. 
If you are unsure about this, take a look at [**Network setup**](https://github.com/fcaponetto/kuka-lwr-ros/wiki/Network-setup)

Once the robot is turned on and you have loaded your script open the FRI such that in the KUKA interface
 you see the following message: **FRI-Ctrl FRI successfully opened**.

In a terminal run the following:
```sh
$ roslaunch lwr_simple_example real.launch
```
You should see an Rviz window with the Robot displayed in the correct configuration.
![alt text](readme/real_rviz.png "real rviz view")
Now in another terminal run:
```sh
$ roslaunch lwr_fri lwr_fri_console.launch
```
In the terminal you should see the following:
![alt text](readme/fri_console.png "fri console")

Here you can see all the different state information of the robot. You can notice that
the FRI State is in MONITOR mode which means that you cannot control the robot for the moment.
Next go to this console and press tab.  You will see that a **control** cmd is avaiable.
Now we are going to change the FRI State to COMMAND which will allow use to run the simple
example (see quick start simulation).
In the terminal type the follwoing:
```sh
FRI> control 
```
In the terminal where you run real.lauch you will see the lign: **Waiting for script...**. This statement means
that you have to press the green button on the KUKA interface panel until you hear a click originating from the robot.

![alt text](readme/command_mode.png "fri in command mode")

Once this is down you will see in the FRI terminal that the FRI State is now in COMMAND mode and that the
Drives are in state GO.

To send actions to the robot proceed as in Quick start (**simulation**). Essentially open two new terminals and launch
the simple client and server nodes.

ROSBuzz
=========================

Description:
============

ROSBuzz is a ROS node encompassing Buzz Virtual Machine (BVM) and interfacing with ROS ecosystem for mobile robots. The only node of the package is `rosbuzz_node`. It can be used in simulation-in-the-loop using Gazebo and was tested over many platforms (Clearpath Husky, DJI M100, Intel Aero, 3DR Solos, Pleidis Spiris, etc.). More information about ROSBuzz peripheral nodes is available in [1].

What is Buzz?
=============

Buzz is a novel programming language for heterogeneous robots swarms. Buzz advocates a compositional approach, by offering primitives to define swarm behaviors both in a bottom-up and in a top-down fashion. Its official documentation and code are available [Buzz](https://github.com/MISTLab/Buzz).

Requirements
============

* Buzz:

Follow the required steps in [Buzz](https://github.com/MISTLab/Buzz).

* ROS **base** binary distribution (Indigo or higher):

Follow the required steps in [ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu). Note that the guidance and camera node of DJI for the M100 require to use the Indigo distribution.

* ROS additionnal dependencies:

```
$ sudo apt-get install ros-<distro>-mavros ros-<distro>-mavros-extras
```

Compilation
===========

```
mkdir -p ROS_WS/src
cd ROS_WS/src
git clone https://github.com/MISTLab/ROSBuzz rosbuzz
cd ..
catkin_make -DSIM=1 -DKIN=1
```
Compilation Flags
=================
Flags to be passed during compilation.

| Flag  | Rationale                                      |
|-------|------------------------------------------------|
| -DSIM | Indicates Compilation for robot or Simulation. |
| -DKIN | Indicates compilation for ROS Distro Kinetic   |
    
Run
===
To run the ROSBuzz package using the launch file, execute the following:

    $ roslaunch rosbuzz rosbuzz.launch
    
Have a look at the launch file to understand what parameters are available to suit your usage. All topics and services names are listed in `launch_config/topics.yaml`. Note : Before launching the ROSBuzz node, verify all the parameters in the launch file. A launch file using gdb is available too (rosbuzzd.launch).

A launch file for a groundstation is also available `launch/groundstation.launch`. It uses the robot ID = 0, which is detected as a groundstation by our Buzz scripts. It also has hardcoded GPS coordinates to avoid the need of a GPS sensor on the groundstation. While a groundstation is never required to deploy a swarm with ROSBuzz, it opens a websocket on ROS, useful to monitor the swarm and it creates a rosbag of the flight.

* Buzz scripts: Several behavioral scripts are included in the "buzz_Scripts" folder, such as "graphformGPS.bzz" uses in [1] and the "testaloneWP.bzz" to control a single drone with a ".csv" list of waypoints. The script "empty.bzz" is a template script.

Publishers
-----------

* Messages from Buzz (BVM):
The node publishes `mavros_msgs/Mavlink` message "outMavlink".

* Command to the flight controller:
The node publishes `geometry_msgs/PoseStamped message` "setpoint_position/local".

* Other information from the swarw:
The node publishes:
    - "bvmstate" (`std_msgs/String`)
    - "neighbours_pos" (`rosbuzz_msgs/neigh_pos`)
    - "fleet_status" (`mavros_msgs/Mavlink`)

Subscribers
-----------

* Information from the Robot controller (mavros compliant):
The node subscribes to:
    - "global_position/global" (`sensor_msgs/NavSatFix message`)
    - "global_position/rel_alt" (`std_msgs/Float64`)
    - "local_position/pose" (`geometry_msgs/PoseStamped`)
    - "battery" (`sensor_msgs/BatteryState`)
    - either "extended_state" (`mavros_msgs/ExtendedState`) or "state" (`mavros_msgs/State`)

* Messages to Buzz (BVM):
The node subscribes to `mavros_msgs/Mavlink` incoming message with name "inMavlink".

Services
-------

* Remote Controller:
The package offers a service "buzzcmd" (`mavros_msgs/CommandLong`) to control it. In the "misc" folder, a bash script shows how to control the swarm state from the command line.

References
------
* [1] ROS and Buzz : consensus-based behaviors for heterogeneous teams. St-Onge, D., Shankar Varadharajan, V., Li, G., Svogor, I. and Beltrame, G. arXiv : https://arxiv.org/abs/1710.08843

* [2] Over-The-Air Updates for Robotic Swarms. Submitted to IEEE Software (August 2017). 8pgs. Shankar Varadharajan, V., St-Onge, D., Guß, C. and Beltrame, G.

Visual Studio Code
--------------------
To activate highlights of the code in Visual Studio Code or Roboware add the following to settings.json:
```
    "files.associations": {
        "*.bzz":"python"
    }
```

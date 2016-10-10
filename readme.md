ROS Implemenation of Buzz
=========================

What is Buzz?
=============

Buzz is a novel programming language for heterogeneous robots swarms.

Buzz advocates a compositional approach, by offering primitives to define swarm behaviors both in a bottom-up and in a top-down fashion.

Bottom-up primitives include robot-wise commands and manipulation of neighborhood data through mapping/reducing/filtering operations.

Top-down primitives allow for the dynamic management of robot teams, and for sharing information globally across the swarm.

Self-organization results from the fact that the Buzz run-time platform is purely distributed.

The language can be extended to add new primitives (thus supporting heterogeneous robot swarms) and can be laid on top of other frameworks, such as ROS.

More information is available at http://the.swarming.buzz/wiki/doku.php?id=start.

Downloading ROS Package
=======================

    $ git clone https://github.com/MISTLab/ROSBuzz.git rosbuzz

Requirements
============

* Buzz : 

You can download the development sources through git:

    $ git clone https://github.com/MISTLab/Buzz.git buzz

* ROS binary distribution (Indigo or higher) with catkin (could be used with older versions of ROS with catkin but not tested)


You need the following package:

* mavros_msgs : 

You can install using apt-get:

    $ sudo apt-get install ros-<distro>-mavros ros-<distro>-mavros-extras

Compilation
===========

To compile the ros package, execute the following:

    $ cd catkin_ws
    $ catkin_make
    $ source devel/setup.bash
    
Run
===

    $ rosrun rosbuzz rosbuzz_node

Publisher
=========

* Messages from Buzz (BVM):
The package publishes mavros_msgs/Mavlink message with a topic name of "outMavlink". 

Subscribers
-----------

* Current position of the Robot:
The package subscribes' to sensor_msgs/NavSatFix message with a topic name of "current_pos".

* Messages to Buzz (BVM):
The package subscribes' to mavros_msgs/Mavlink message with a topic name of "inMavlink".

* Battery status:
The package subscribes' to mavros_msgs/BatteryStatus message with a topic name of "battery_state".

Service
-------

* Remote Controller command:
The package offers service using mavros_msgs/CommandInt service with name "djicmd_rc".

Client
------

* Flight controller client:
This package is a client of mavros_msgs/CommandInt service with name "djicmd".
 

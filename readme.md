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

Description:
============

Rosbuzz package is the ROS version of Buzz. The package contains a node called “rosbuzz_node”, which implements buzz virtual machine (BVM) as a node in ROS.


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
To run the ROSBuzz package using the launch file, execute the following:

    $ roslaunch rosbuzz rosbuzz.launch
    
Note : Before launching the ROSBuzz node, verify all the parameters in the launch file. A launch file using gdb is available also (rosbuzzd.launch).

* Buzz scripts: Several behavioral scripts are included in the "buzz_Scripts" folder, such as "graphformGPS.bzz" uses in the ICRA publication below and the "testaloneWP.bzz" to control a single drone with a ".csv" list of waypoints. The script "empty.bzz" is a template script.

Publisher
=========

* Messages from Buzz (BVM):
The package publishes mavros_msgs/Mavlink message "outMavlink".

* Command to the flight controller:
The package publishes geometry_msgs/PoseStamped message "setpoint_position/local".

Subscribers
-----------

* Current position of the Robot:
The package subscribes to sensor_msgs/NavSatFix message "global_position/global", to a std_msgs/Float64 message "global_position/rel_alt" and to a geometry_msgs/PoseStamped message "local_position/pose".

* Messages to Buzz (BVM):
The package subscribes to mavros_msgs/Mavlink message with a topic name of "inMavlink".

* Status:
The package subscribes to mavros_msgs/BatteryStatus message "battery" and to either a mavros_msgs/ExtendedState message "extended_state" or a mavros_msgs/State message "state".

Service
-------

* Remote Controller:
The package offers a mavros_msgs/CommandLong service "buzzcmd" to control its state. In the "misc" folder a bash script shows how to control the Buzz states from the command line.

References
------
* ROS and Buzz : consensus-based behaviors for heterogeneous teams. Submitted to the Internaional Conference on Robotics and Automation (September 2017). 6pgs. St-Onge, D., Shankar Varadharajan, V., Li, G., Svogor, I. and Beltrame, G. arXiv : https://arxiv.org/abs/1710.08843

* Over-The-Air Updates for Robotic Swarms. Submitted to IEEE Software (August 2017). 8pgs. Shankar Varadharajan, V., St-Onge, D., Guß, C. and Beltrame, G.

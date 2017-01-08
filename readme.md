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

Rosbuzz package is the ROS version of Buzz. The package contains a node called “rosbuzz_node”, which implements buzz as a node in ROS. The rosbuzz_node requires certain parameters to run. These parameters are supplied to the node from the ros parameter server. The package also contains a launch file called “rosbuzzm100.launch”, which initializes the required parameters. The required parameters are the .bzz file location, presence of Remote Controller client, Remote Controller service name, Flight Controller client name, Robot ID and Number of Robots.

* The .bzz file location parameter is a string with name “bzzfile_name”, the node compiles and parses the specified .bzz file into .basm, .bo and .bdbg files. The node runs the parsed .bo byte file.

* The presence of remote controller client is a bool parameter with name “rcclient”,  this  specifies  whether there is a remote controller present to issue bypassing commands like takeoff, land, goto location and go home. If there is no remote controller present then this parameter could be set to “false”. If a remote controller exists, this parameter could be set “True” and the service topic offered by the remote controller should be specified to the parameter named “rcservice_name”.

* The flight controller client present in the node, that is used to send commands to the flight controller. The commands that could be sent are takeoff, land, goto location and go home. The topic name used to communicate with the flight controller should be set to the parameter named “fcclient_name”.

* The next parameter that the rosbuzz_node depends on is the robot ID of type “int”, this parameter should be set to the parameter named “robot_id” in the parameter server. This parameter is by default set to ‘0’, if not specified.

* Hot swaps of code can be integrated into the ROSBuzz node directly from a text editor. The ROSBuzz node monitors the file supplied i.e. the name specified in "bzzfile_name". When this file is modified and saved, the new script is tested and once test passes. The new script is set to the current robot and sent to all the robots in the network. To ensure convergence, i.e. all the robots recived the code, number of robots present in the network is needed. The number of robots has to be specified under the parameter name "No_of_Robots".    

An example template of parameter setting could be found in the launch file “launch/rosbuzzm100.launch”. 
The launch file named "launch/rosbuzz_2_parallel.launch" can be used to launch two nodes in parellel for testing ROSBuzz node on a single Machine.


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

    $ roslaunch rosbuzz rosbuzzm100.launch
    
Note : Before launching the ROSBuzz node, verify all the parameter in the launch file.

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
The package offers service using mavros_msgs/CommandInt service with name "rc_cmd".

Client
------

* Flight controller client:
This package is a client of mavros_msgs/CommandInt service with name "fc_cmd".
 

#! /bin/bash
function takeoff {
	rosservice call $1/buzzcmd 0 22 0 0 0 0 0 0 0 0
}
function land {
	rosservice call $1/buzzcmd 0 21 0 0 0 0 0 0 0 0
}
function arm {
	rosservice call $1/buzzcmd 0 400 0 1 0 0 0 0 0 0
}
function disarm {
	rosservice call $1/buzzcmd 0 400 0 0 0 0 0 0 0 0
}
function timesync {
        rosservice call $1/buzzcmd 0 777 0 0 0 0 0 0 0 0
}
function testWP {
	rosservice call $1/buzzcmd 0 16 0 0 0 0 0 45.45782 -- -73.63608 10
}
function record {
	rosbag record $1/flight_status $1/global_position $1/users_pos $1/local_position $1/neighbours_pos /power_status /guidance/obstacle_distance /guidance/front/depth/image_rect/compressedDepth /guidance/right/depth/image_rect/compressedDepth /guidance/front/depth/points /guidance/right/depth/points /guidance/front/right/image_rect/compressed /guidance/front/left/image_rect/compressed /guidance/right/right/image_rect/compressed /guidance/right/left/image_rect/compressed /guidance/front/left/camera_info /guidance/front/right/camera_info /guidance/right/right/camera_info /guidance/right/left/camera_info

}
function clean {
	sudo rm /var/log/upstart/robot*
	sudo rm /var/log/upstart/dji*
	sudo rm /var/log/upstart/x3s*
}
function startrobot {
	sudo service dji start
}
function stoprobot {
	sudo service dji stop
}
function updaterobot {
	rosrun robot_upstart uninstall dji
	if [ "$1" = 0 ] && [ "$2" = "X" ]
	then
	  echo "Installing launch file for TX-ubuntu16 with usb2serial"
	  echo "With xbeemav"
	  rosrun robot_upstart install --logdir /media/key/ROS_WS/log/ dji_sdk_mistlab/launch_robot/m100TX16US.launch
	elif [ "$1" = 1 ] && [ "$2" = "X" ]
	then
	  echo "Installing launch file for TX-ubuntu16"
	  echo "With xbeemav"
	  rosrun robot_upstart install --logdir /media/key/ROS_WS/log/ dji_sdk_mistlab/launch_robot/m100TX16.launch
	elif [ "$1" = 2 ] && [ "$2" = "X" ]
	then
	  echo "Installing launch file for TK-ubuntu14"
	  echo "With xbeemav"
	  rosrun robot_upstart install --logdir /media/key/ROS_WS/log/ dji_sdk_mistlab/launch_robot/m100TKAll.launch
	elif [ "$1" = 3 ] && [ "$2" = "X" ]
	then
	  echo "Installing launch file for Solo"
	  echo "With xbeemav"
	  rosrun robot_upstart install --logdir /media/key/ROS_WS/log/ dji_sdk_mistlab/launch_robot/solo.launch
	elif [ "$1" = 4 ] && [ "$2" = "X" ]
	then
	  echo "Installing launch file for Spiris"
	  echo "With xbeemav"
	  rosrun robot_upstart install --logdir /media/key/ROS_WS/log/ dji_sdk_mistlab/launch_robot/spiri.launch
	elif [ "$1" = 0 ] && [ "$2" = "H" ]
	then
	  echo "Installing launch file for TX-ubuntu16 with usb2serial"
	  echo "With heavenmav"
	  rosrun robot_upstart install --logdir /media/key/ROS_WS/log/ dji_sdk_mistlab/launch_robot/m100TX16USHeaven.launch
	elif [ "$1" = 1 ] && [ "$2" = "H" ]
	then
	  echo "Installing launch file for TX-ubuntu16"
	  echo "With heavenmav"
	  rosrun robot_upstart install --logdir /media/key/ROS_WS/log/ dji_sdk_mistlab/launch_robot/m100TX16Heaven.launch
	elif [ "$1" = 2 ] && [ "$2" = "H" ]
	then
	  echo "Installing launch file for TK-ubuntu14"
	  echo "With heavenmav"
	  rosrun robot_upstart install --logdir /media/key/ROS_WS/log/ dji_sdk_mistlab/launch_robot/m100TKAllHeaven.launch
	elif [ "$1" = 3 ] && [ "$2" = "H" ]
	then
	  echo "Installing launch file for Solo"
	  echo "With heavenmav"
	  rosrun robot_upstart install --logdir /media/key/ROS_WS/log/ dji_sdk_mistlab/launch_robot/soloHeaven.launch
	else
	  echo "Wrong arguments!"
	fi
}
function uavstate {
	let "a = $1 + 900"
	rosservice call robot0/buzzcmd 0 $a 0 0 0 0 0 0 0 0
}

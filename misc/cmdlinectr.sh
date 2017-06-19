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
function arm {
	rosservice call /buzzcmd 0 400 0 1 0 0 0 0 0 0
}
function disarm {
	rosservice call /buzzcmd 0 400 0 0 0 0 0 0 0 0
}
function record {
<<<<<<< HEAD
	rosbag record /flight_status /global_position /users_pos /dji_sdk/local_position /neighbours_pos /power_status /guidance/obstacle_distance /guidance/front/depth/image_rect/compressedDepth /guidance/right/depth/image_rect/compressedDepth /guidance/front/depth/points /guidance/right/depth/points /guidance/front/right/image_rect/compressed /guidance/front/left/image_rect/compressed /guidance/right/right/image_rect/compressed /guidance/right/left/image_rect/compressed /guidance/front/left/camera_info /guidance/front/right/camera_info /guidance/right/right/camera_info /guidance/right/left/camera_info
=======
	rosbag record $1/flight_status $1/global_position $1/users_pos $1/local_position $1/neighbours_pos /power_status /guidance/obstacle_distance /guidance/front/depth/image_rect/compressedDepth /guidance/right/depth/image_rect/compressedDepth /guidance/front/depth/points /guidance/right/depth/points /guidance/front/right/image_rect/compressed /guidance/front/left/image_rect/compressed /guidance/right/right/image_rect/compressed /guidance/right/left/image_rect/compressed /guidance/front/left/camera_info /guidance/front/right/camera_info /guidance/right/right/camera_info /guidance/right/left/camera_info
>>>>>>> 78edc3c9f069e1c186f3b9a5cb3853a233f3dde3
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
#	rosrun robot_upstart install --logdir ~/ROS_WS/log/ robot_upstart/launch/m100buzzynocam.launch
	rosrun robot_upstart install --logdir ~/ROS_WS/log/ dji_sdk_mistlab/launch/m100buzzy.launch
}
function updaterobot {
#	rosrun robot_upstart install --logdir ~/ROS_WS/log/ robot_upstart/launch/m100buzzynocam.launch
	rosrun robot_upstart install --logdir ~/ROS_WS/log/ dji_sdk_mistlab/launch/m100buzzy.launch
}

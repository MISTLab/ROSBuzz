#! /bin/bash
function takeoff {
	rosservice call /buzzcmd 0 22 0 0 0 0 0 0 0 0
}
function land {
	rosservice call /buzzcmd 0 21 0 0 0 0 0 0 0 0
}
function arm {
	rosservice call /buzzcmd 0 400 0 1 0 0 0 0 0 0
}
function disarm {
	rosservice call /buzzcmd 0 400 0 0 0 0 0 0 0 0
}
function record {
	rosbag record /flight_status /global_position /users_pos /dji_sdk/local_position /neighbours_pos /power_status /guidance/obstacle_distance /guidance/front/depth/image_rect/compressedDepth /guidance/right/depth/image_rect/compressedDepth /guidance/front/depth/points /guidance/right/depth/points /guidance/front/right/image_rect/compressed /guidance/front/left/image_rect/compressed /guidance/right/right/image_rect/compressed /guidance/right/left/image_rect/compressed /guidance/front/left/camera_info /guidance/front/right/camera_info /guidance/right/right/camera_info /guidance/right/left/camera_info
}
function clean {
	sudo rm /var/log/upstart/robot*
	sudo rm /var/log/upstart/dji*
	sudo rm /var/log/upstart/x3s*
}
function startrobot {
	sudo service robot start
}
function stoprobot {
	sudo service robot stop
}
function updaterobot {
#	rosrun robot_upstart install --logdir ~/ROS_WS/log/ robot_upstart/launch/m100buzzynocam.launch
	rosrun robot_upstart install --logdir ~/ROS_WS/log/ dji_sdk_mistlab/launch/m100buzzy.launch
}

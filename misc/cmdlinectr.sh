#! /bin/bash
function takeoff {
	rosservice call /buzzcmd 0 22 0 0 0 0 0 0 0 0
}
function land {
	rosservice call /buzzcmd 0 21 0 0 0 0 0 0 0 0
}
function record {
	rosbag record /flight_status /global_position /dji_sdk/local_position /neighbours_pos /power_status /guidance/obstacle_distance /guidance/front_depth_image/compressedDepth /guidance/right_depth_image/compressedDepth /guidance/left_depth_image/compressedDepth /guidance/bottom_depth_image/compressedDepth /guidance/back_depth_image/compressedDepth
}
function clean {
	sudo rm /var/log/upstart/robot*
	sudo rm /var/log/upstart/x3s*
}
function startrobot {
	sudo service robot start
}
function stoprobot {
	sudo service robot stop
}

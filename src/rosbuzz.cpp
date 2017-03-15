/** @file      rosbuzz.cpp
 *  @version   1.0 
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS for Dji M100 Drone. 
 *  @author    Vivek Shankar Varadharajan
 *  @copyright 2016 MistLab. All rights reserved.
 */

#include "roscontroller.h"

/**
 * This program implements Buzz node in ros using mavros_msgs.
 */
	
int main(int argc, char **argv)
{
  /*Initialize rosBuzz node*/
  ros::init(argc, argv, "rosBuzz"); 
  ros::NodeHandle n_c("~");
  rosbzz_node::roscontroller RosController(n_c);
  /*Register ros controller object inside buzz*/
  //buzzuav_closures::set_ros_controller_ptr(&RosController);
  RosController.RosControllerRun(); 
  return 0;
}



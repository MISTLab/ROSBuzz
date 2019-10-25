/** @file      rosbuzz.cpp
 *  @version   1.0
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS.
 *  @author    Vivek Shankar Varadharajan and David St-Onge
 *  @copyright 2016 MistLab. All rights reserved.
 */

#include <rosbuzz/roscontroller.h>

int main(int argc, char** argv)
/*
 / This program implements Buzz in a ROS node using mavros_msgs.
 -----------------------------------------------------------------*/
{
  //  Initialize rosBuzz node
  ros::init(argc, argv, "rosBuzz");
  ros::NodeHandle nh_priv("~");
  ros::NodeHandle nh;
  rosbuzz_node::roscontroller RosController(nh, nh_priv);

  RosController.RosControllerRun();
  return 0;
}

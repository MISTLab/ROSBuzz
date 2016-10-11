#pragma once
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/CommandInt.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/BatteryStatus.h"
#include "mavros_msgs/Mavlink.h"
#include "sensor_msgs/NavSatStatus.h"
#include <sstream>
#include <buzz/buzzasm.h>
#include "buzzuav_closures.h"
#include "buzz_utility.h"
#include "uav_utility.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <ostream>
#include <map>

using namespace std;

namespace rosbzz_node{


class roscontroller{

public:
	roscontroller();
	~roscontroller();
	void RosControllerRun();

private:
        
 	double cur_pos[3];
	uint64_t payload;
	std::map< int,  buzz_utility::Pos_struct> neighbours_pos_map;
	int timer_step=0;
	int robot_id=0;
        int oldcmdID=0;
	std::string bzzfile_name, fcclient_name, rcservice_name; //, rcclient;
	bool rcclient;
	ros::ServiceClient mav_client;
	ros::Publisher payload_pub;
	ros::ServiceServer service;
	ros::Subscriber current_position_sub;
	ros::Subscriber battery_sub;
	ros::Subscriber payload_sub;
	/*Create node Handler*/
		ros::NodeHandle n_c;
	/*Commands for flight controller*/
  		mavros_msgs::CommandInt cmd_srv;	
  	/* The bytecode filename */
   		char* bcfname = (char*)"../catkin_ws/src/rosbuzz/src/out.bo"; //argv[1];
   		/* The debugging information file name */
   		char* dbgfname = (char*)"../catkin_ws/src/rosbuzz/src/out.bdbg"; //argv[2];

	void Initialize_pub_sub();

	/*Obtain data from ros parameter server*/	
	void Rosparameters_get();

	void Compile_bzz();

	/*Prepare messages and publish*/
	inline void prepare_msg_and_publish();

	  	
	/*Refresh neighbours Position for every ten step*/
	inline void maintain_pos(int tim_step);

	/*Maintain neighbours position*/
	inline void neighbours_pos_maintain(int id, buzz_utility::Pos_struct pos_arr );

        /*Set the current position of the robot callback*/
	inline void set_cur_pos(double latitude,
			 double longitude,
			 double altitude);
	/*convert from catresian to spherical coordinate system callback */
	inline double* cvt_spherical_coordinates(double neighbours_pos_payload []);
	
	/*battery status callback*/ 
	inline void battery(const mavros_msgs::BatteryStatus::ConstPtr& msg);

	/*current position callback*/
	inline void current_pos(const sensor_msgs::NavSatFix::ConstPtr& msg);

	/*payload callback callback*/
	inline void payload_obt(const mavros_msgs::Mavlink::ConstPtr& msg);

	/* RC commands service */
	inline bool rc_callback(mavros_msgs::CommandInt::Request  &req,
		         mavros_msgs::CommandInt::Response &res);



};

}

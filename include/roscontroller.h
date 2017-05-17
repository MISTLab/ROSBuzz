#pragma once
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/BatteryStatus.h"
#include "mavros_msgs/Mavlink.h"
#include "mavros_msgs/PositionTarget.h"
#include "sensor_msgs/NavSatStatus.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/StreamRate.h"
#include "mavros_msgs/ParamGet.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/LaserScan.h>
#include <rosbuzz/neigh_pos.h>
#include <sstream>
#include <buzz/buzzasm.h>
#include "buzz_utility.h"
#include "uav_utility.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <ostream>
#include <map>
#include "buzzuav_closures.h"

#define UPDATER_MESSAGE_CONSTANT 987654321
#define XBEE_MESSAGE_CONSTANT 586782343
#define XBEE_STOP_TRANSMISSION 4355356352
#define TIMEOUT	60

using namespace std;

namespace rosbzz_node{
	
class roscontroller{

public:
	roscontroller(ros::NodeHandle& n_c, ros::NodeHandle& n_c_priv);
	~roscontroller();
	//void RosControllerInit();
	void RosControllerRun();
	
private:
    struct num_robot_count
	{
		uint8_t history[10];
		uint8_t index=0;
	  	uint8_t current=0;
  		num_robot_count(){}
	  
	}; typedef struct num_robot_count Num_robot_count ;

	struct gps
	{
		double longitude=0.0;
		double latitude=0.0;
		float  altitude=0.0;
	}; typedef struct gps GPS ;
	
	GPS target, home, cur_pos;
 	double cur_rel_altitude;

	uint64_t payload;
	std::map< int,  buzz_utility::Pos_struct> neighbours_pos_map;
	std::map< int,  buzz_utility::Pos_struct> raw_neighbours_pos_map;
	//std::map< int, buzz_utility::Pos_struct> pub_neigh_pos;
	int timer_step=0;
	int robot_id=0;
	std::string robot_name = "";
        //int oldcmdID=0;
	int rc_cmd;
	float fcu_timeout;
	int armstate;
	int barrier;
	int message_number=0;
	uint8_t no_of_robots=0;
	/*tmp to be corrected*/
	uint8_t no_cnt=0;
	uint8_t old_val=0;	
	std::string bzzfile_name, fcclient_name, armclient, modeclient, rcservice_name,bcfname,dbgfname,out_payload,in_payload,stand_by,xbeesrv_name, setpoint_name;
	std::string stream_client_name;
	std::string relative_altitude_sub_name;
	std::string setpoint_nonraw;
	bool rcclient;
	bool xbeeplugged = false;
	bool multi_msg;
	Num_robot_count count_robots;
	ros::ServiceClient mav_client;
	ros::ServiceClient xbeestatus_srv;
	ros::Publisher payload_pub;
	ros::Publisher neigh_pos_pub;
	ros::Publisher localsetpoint_nonraw_pub;
	ros::ServiceServer service;
	ros::Subscriber current_position_sub;
	ros::Subscriber users_sub;
	ros::Subscriber battery_sub;
	ros::Subscriber payload_sub;
	ros::Subscriber flight_status_sub;
	ros::Subscriber obstacle_sub;
	ros::Subscriber Robot_id_sub;
	ros::Subscriber relative_altitude_sub;

	std::string local_pos_sub_name;
	ros::Subscriber local_pos_sub;
	double local_pos_new[3];


	ros::ServiceClient stream_client;

	int setpoint_counter;
	double my_x = 0, my_y = 0;
	
	std::ofstream log;

	/*Commands for flight controller*/
  	//mavros_msgs::CommandInt cmd_srv;
  	mavros_msgs::CommandLong cmd_srv;
  	std::vector<std::string> m_sMySubscriptions;
  	std::map<std::string, std::string> m_smTopic_infos;

  	mavros_msgs::CommandBool m_cmdBool;
  	ros::ServiceClient arm_client;

  	mavros_msgs::SetMode m_cmdSetMode;
  	ros::ServiceClient mode_client;
	
	/*Initialize publisher and subscriber, done in the constructor*/
	void Initialize_pub_sub(ros::NodeHandle& n_c);

  	std::string current_mode; // SOLO SPECIFIC: just so you don't call the switch to same mode

	/*Obtain data from ros parameter server*/	
	void Rosparameters_get(ros::NodeHandle& n_c_priv);

	/*compiles buzz script from the specified .bzz file*/
	std::string Compile_bzz(std::string bzzfile_name);

	/*Flight controller service call*/
	void flight_controller_service_call();
	
	/*Neighbours pos publisher*/
	void neighbours_pos_publisher();

	/*Prepare messages and publish*/
	void prepare_msg_and_publish();

	  	
	/*Refresh neighbours Position for every ten step*/
	void maintain_pos(int tim_step);

	/*Puts neighbours position inside neigbours_pos_map*/
	void neighbours_pos_put(int id, buzz_utility::Pos_struct pos_arr );

	/*Puts raw neighbours position in lat.,long.,alt. inside raw_neigbours_pos_map*/
	void raw_neighbours_pos_put(int id, buzz_utility::Pos_struct pos_arr );

    /*Set the current position of the robot callback*/
	void set_cur_pos(double latitude,
			 double longitude,
			 double altitude);
	/*convert from spherical to cartesian coordinate system callback */
	void gps_rb(GPS nei_pos, double out[]);
	void gps_ned_cur(float &ned_x, float &ned_y, GPS t);
	void gps_ned_home(float &ned_x, float &ned_y);
	void gps_convert_ned(float &ned_x, float &ned_y,
			double gps_t_lon, double gps_t_lat,
			double gps_r_lon, double gps_r_lat);

	/*battery status callback*/ 
	void battery(const mavros_msgs::BatteryStatus::ConstPtr& msg);
	
	/*flight extended status callback*/
	void flight_extended_status_update(const mavros_msgs::ExtendedState::ConstPtr& msg);

	/*flight status callback*/
	void flight_status_update(const mavros_msgs::State::ConstPtr& msg);
	
	/*current position callback*/
	void current_pos(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void users_pos(const rosbuzz::neigh_pos msg);


	/*current relative altitude callback*/
	void current_rel_alt(const std_msgs::Float64::ConstPtr& msg);

	/*payload callback callback*/
	void payload_obt(const mavros_msgs::Mavlink::ConstPtr& msg);

	/* RC commands service */
	bool rc_callback(mavros_msgs::CommandLong::Request  &req, mavros_msgs::CommandLong::Response &res);

	/*robot id sub callback*/
	void set_robot_id(const std_msgs::UInt8::ConstPtr& msg);

	/*Obstacle distance table callback*/
	void obstacle_dist(const sensor_msgs::LaserScan::ConstPtr& msg);

	/*Get publisher and subscriber from YML file*/
	void GetSubscriptionParameters(ros::NodeHandle& node_handle);

	/*Arm/disarm method that can be called from buzz*/
	void Arm();

	/*set mode like guided for solo*/
	void SetMode(std::string mode, int delay_miliseconds);

	/*Robot independent subscribers*/
	void Subscribe(ros::NodeHandle& n_c);

	void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);

	//void WaypointMissionSetup(float lat, float lng, float alt);

	void fc_command_setup();

	void SetLocalPosition(float x, float y, float z, float yaw);

	void SetLocalPositionNonRaw(float x, float y, float z, float yaw);

	void SetStreamRate(int id, int rate, int on_off);
	
	void get_number_of_robots();
	void GetRobotId();
};

}

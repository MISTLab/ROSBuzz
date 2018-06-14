#pragma once
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/UInt8.h>
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/BatteryStatus.h"
#include "sensor_msgs/BatteryState.h"
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
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <rosbuzz/neigh_pos.h>
#include <sstream>
#include <buzz/buzzasm.h>
#include "buzz_utility.h"
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
#define TIMEOUT 60
#define BUZZRATE 10
#define CMD_REQUEST_UPDATE 666

using namespace std;

namespace rosbzz_node
{
class roscontroller
{
public:
  roscontroller(ros::NodeHandle& n_c, ros::NodeHandle& n_c_priv);
  ~roscontroller();
  void RosControllerRun();

  static const string CAPTURE_SRV_DEFAULT_NAME;

private:
  struct num_robot_count
  {
    uint8_t history[10];
    uint8_t index = 0;
    uint8_t current = 0;
    num_robot_count()
    {
    }
  };
  typedef struct num_robot_count Num_robot_count;

  Num_robot_count count_robots;

  struct POSE
  {
    double longitude = 0.0;
    double latitude = 0.0;
    float altitude = 0.0;
    // NED coordinates
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float yaw = 0.0;
  };
  typedef struct POSE ros_pose;

  ros_pose target, home, cur_pos;

  uint64_t payload;
  std::map<int, buzz_utility::Pos_struct> neighbours_pos_map;
  std::map<int, buzz_utility::Pos_struct> raw_neighbours_pos_map;
  int timer_step = 0;
  int robot_id = 0;
  std::string robot_name = "";

  int rc_cmd;
  float fcu_timeout;
  int armstate;
  int barrier;
  int update;
  int message_number = 0;
  uint8_t no_of_robots = 0;
  bool rcclient;
  bool xbeeplugged = false;
  bool multi_msg;
  uint8_t no_cnt = 0;
  uint8_t old_val = 0;
  std::string bzzfile_name;
  std::string fcclient_name;
  std::string armclient;
  std::string modeclient;
  std::string rcservice_name;
  std::string bcfname, dbgfname;
  std::string out_payload;
  std::string in_payload;
  std::string stand_by;
  std::string xbeesrv_name;
  std::string capture_srv_name;
  std::string setpoint_name;
  std::string stream_client_name;
  std::string setpoint_nonraw;

  // ROS service, publishers and subscribers
  ros::ServiceClient mav_client;
  ros::ServiceClient xbeestatus_srv;
  ros::ServiceClient capture_srv;
  ros::ServiceClient stream_client;
  ros::Publisher payload_pub;
  ros::Publisher MPpayload_pub;
  ros::Publisher neigh_pos_pub;
  ros::Publisher uavstate_pub;
  ros::Publisher grid_pub;
  ros::Publisher localsetpoint_nonraw_pub;
  ros::ServiceServer service;
  ros::Subscriber current_position_sub;
  ros::Subscriber users_sub;
  ros::Subscriber battery_sub;
  ros::Subscriber payload_sub;
  ros::Subscriber flight_estatus_sub;
  ros::Subscriber flight_status_sub;
  ros::Subscriber obstacle_sub;
  ros::Subscriber Robot_id_sub;
  ros::Subscriber relative_altitude_sub;
  ros::Subscriber local_pos_sub;

  std::map<std::string, std::string> m_smTopic_infos;

  int setpoint_counter;

  std::ofstream log;

  //  Commands for flight controller
  mavros_msgs::CommandLong cmd_srv;

  mavros_msgs::CommandBool m_cmdBool;
  ros::ServiceClient arm_client;

  mavros_msgs::SetMode m_cmdSetMode;
  ros::ServiceClient mode_client;

  //  Initialize publisher and subscriber, done in the constructor
  void Initialize_pub_sub(ros::NodeHandle& n_c);

  std::string current_mode;

  /*Obtain data from ros parameter server*/
  void Rosparameters_get(ros::NodeHandle& n_c_priv);

  /*compiles buzz script from the specified .bzz file*/
  std::string Compile_bzz(std::string bzzfile_name);

  /*Flight controller service call*/
  void flight_controller_service_call();

  /*Neighbours pos publisher*/
  void neighbours_pos_publisher();

  /*UAVState publisher*/
  void uavstate_publisher();

  /*Grid publisher*/
  void grid_publisher();

  /*BVM message payload publisher*/
  void send_MPpayload();

  /*Prepare messages and publish*/
  void prepare_msg_and_publish();

  /*Refresh neighbours Position for every ten step*/
  void maintain_pos(int tim_step);

  /*Puts neighbours position inside neigbours_pos_map*/
  void neighbours_pos_put(int id, buzz_utility::Pos_struct pos_arr);

  /*Puts raw neighbours position in lat.,long.,alt. inside raw_neigbours_pos_map*/
  void raw_neighbours_pos_put(int id, buzz_utility::Pos_struct pos_arr);

  /*Set the current position of the robot callback*/
  void set_cur_pos(double latitude, double longitude, double altitude);

  /*convert from spherical to cartesian coordinate system callback */
  float constrainAngle(float x);
  void gps_rb(POSE nei_pos, double out[]);
  void gps_ned_cur(float& ned_x, float& ned_y, POSE t);
  void gps_convert_ned(float& ned_x, float& ned_y, double gps_t_lon, double gps_t_lat, double gps_r_lon,
                       double gps_r_lat);

  /*battery status callback */
  void battery(const sensor_msgs::BatteryState::ConstPtr& msg);

  /*flight extended status callback*/
  void flight_extended_status_update(const mavros_msgs::ExtendedState::ConstPtr& msg);

  /*flight status callback*/
  void flight_status_update(const mavros_msgs::State::ConstPtr& msg);

  /*current position callback*/
  void global_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

  /*current relative altitude callback*/
  void rel_alt_callback(const std_msgs::Float64::ConstPtr& msg);

  /*payload callback callback*/
  void payload_obt(const mavros_msgs::Mavlink::ConstPtr& msg);

  /* RC commands service */
  bool rc_callback(mavros_msgs::CommandLong::Request& req, mavros_msgs::CommandLong::Response& res);

  /*robot id sub callback*/
  void set_robot_id(const std_msgs::UInt8::ConstPtr& msg);

  /*Obstacle distance table callback*/
  void obstacle_dist_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /*Get publisher and subscriber from YML file*/
  void GetSubscriptionParameters(ros::NodeHandle& node_handle);

  /*Arm/disarm method that can be called from buzz*/
  void Arm();

  /*set mode like guided for solo*/
  void SetMode(std::string mode, int delay_miliseconds);

  /*Robot independent subscribers*/
  void Subscribe(ros::NodeHandle& n_c);

  void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);

  void fc_command_setup();

  void SetLocalPosition(float x, float y, float z, float yaw);

  void SetLocalPositionNonRaw(float x, float y, float z, float yaw);

  void SetStreamRate(int id, int rate, int on_off);

  void get_number_of_robots();

  // functions related to Xbee modules information
  void GetRobotId();
  bool GetDequeFull(bool& result);
  bool GetRssi(float& result);
  bool TriggerAPIRssi(const uint8_t short_id);
  bool GetAPIRssi(const uint8_t short_id, float& result);
  bool GetRawPacketLoss(const uint8_t short_id, float& result);
  bool GetFilteredPacketLoss(const uint8_t short_id, float& result);
  void get_xbee_status();

};
}

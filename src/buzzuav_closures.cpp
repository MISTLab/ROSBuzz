/** @file      buzzuav_closures.cpp
 *  @version   1.0
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS.
 *  @author    Vivek Shankar Varadharajan and David St-Onge
 *  @copyright 2016 MistLab. All rights reserved.
 */

#include "buzzuav_closures.h"
#include "math.h"

namespace buzzuav_closures
{
// TODO: Minimize the required global variables and put them in the header
// static const rosbzz_node::roscontroller* roscontroller_ptr;
static double goto_pos[4];
static double goto_gpsgoal[3];
static double cur_pos[4];
static double cur_NEDpos[2];

static int rc_id = -1;
static int rc_cmd = 0;
static double rc_gpsgoal[3];
static float rc_gimbal[4];

static float batt[3];
static float obst[5] = { 0, 0, 0, 0, 0 };
static uint8_t status;

static int cur_cmd = 0;
static int buzz_cmd = 0;
static float height = 0;

static bool deque_full = false;
static int rssi = 0;
static float raw_packet_loss = 0.0;
static int filtered_packet_loss = 0;
static float api_rssi = 0.0;

string WPlistname = "";

std::map<int, buzz_utility::RB_struct> targets_map;
std::map<int, buzz_utility::RB_struct> wplist_map;
std::map<int, buzz_utility::Pos_struct> neighbors_map;
std::map<int, buzz_utility::neighbors_status> neighbors_status_map;
std::map<int, std::map<int,int>> grid;

/****************************************/
/****************************************/

int buzzros_print(buzzvm_t vm)
/*
/ Buzz closure to print out
----------------------------------------------------------- */
{
  std::ostringstream buffer(std::ostringstream::ate);
  buffer << std::fixed << std::setprecision(6) << "[" << buzz_utility::get_robotid() << "] ";
  for (uint32_t index = 1; index < buzzdarray_size(vm->lsyms->syms); ++index)
  {
    buzzvm_lload(vm, index);
    buzzobj_t o = buzzvm_stack_at(vm, 1);
    buzzvm_pop(vm);
    switch (o->o.type)
    {
      case BUZZTYPE_NIL:
        buffer << " BUZZ - [nil]";
        break;
      case BUZZTYPE_INT:
        buffer << " " << o->i.value;
        break;
      case BUZZTYPE_FLOAT:
        buffer << " " << o->f.value;
        break;
      case BUZZTYPE_TABLE:
        buffer << " [table with " << buzzdict_size(o->t.value) << " elems]";
        break;
      case BUZZTYPE_CLOSURE:
        if (o->c.value.isnative)
        {
          buffer << " [n-closure @" << o->c.value.ref << "]";
        }
        else
        {
          buffer << " [c-closure @" << o->c.value.ref << "]";
        }
        break;
      case BUZZTYPE_STRING:
        buffer << "  " << o->s.value.str;
        break;
      case BUZZTYPE_USERDATA:
        buffer << " [userdata @" << o->u.value << "]";
        break;
      default:
        break;
    }
  }
  ROS_INFO("%s", buffer.str().c_str());
  return buzzvm_ret0(vm);
}

void setWPlist(string file)
/*
/ set the absolute path for a csv list of waypoints
----------------------------------------------------------- */
{
  WPlistname = file;//path + "include/taskallocate/waypointlist.csv";
  parse_gpslist();
}

float constrainAngle(float x)
/*
/ Wrap the angle between -pi, pi
----------------------------------------------------------- */
{
  x = fmod(x + M_PI, 2 * M_PI);
  if (x < 0.0)
    x += 2 * M_PI;
  return x - M_PI;
}

void rb_from_gps(double nei[], double out[], double cur[])
/*
/ Compute Range and Bearing from 2 GPS set of coordinates
/----------------------------------------*/
{
  double d_lon = nei[1] - cur[1];
  double d_lat = nei[0] - cur[0];
  double ned_x = DEG2RAD(d_lat) * EARTH_RADIUS;
  double ned_y = DEG2RAD(d_lon) * EARTH_RADIUS * cos(DEG2RAD(nei[0]));
  out[0] = sqrt(ned_x * ned_x + ned_y * ned_y);
  out[1] = constrainAngle(atan2(ned_y, ned_x));
  out[2] = 0.0;
}

void parse_gpslist()
/*
/ parse a csv of GPS targets
/----------------------------------------*/
{
  // Open file:
  ROS_INFO("WP list file: %s", WPlistname.c_str());
  std::ifstream fin(WPlistname.c_str());  // Open in text-mode.

  // Opening may fail, always check.
  if (!fin)
  {
    ROS_ERROR("GPS list parser, could not open file.");
    return;
  }

  // Prepare a C-string buffer to be used when reading lines.
  const int MAX_LINE_LENGTH = 1024;  // Choose this large enough for your need.
  char buffer[MAX_LINE_LENGTH] = {};
  const char* DELIMS = "\t ,";  // Tab, space or comma.

  // Read the file and load the data:
  buzz_utility::RB_struct RB_arr;
  // Read one line at a time.
  while (fin.getline(buffer, MAX_LINE_LENGTH))
  {
    // Extract the tokens:
    int tid = atoi(strtok(buffer, DELIMS));
    double lon = atof(strtok(NULL, DELIMS));
    double lat = atof(strtok(NULL, DELIMS));
    int alt = atoi(strtok(NULL, DELIMS));
    // int tilt = atoi(strtok(NULL, DELIMS));
    //  DEBUG
    // ROS_INFO("%.6f, %.6f, %i %i %i",lat, lon, alt, tilt, tid);
    RB_arr.latitude = lat;
    RB_arr.longitude = lon;
    RB_arr.altitude = alt;
    // Insert elements.
    map<int, buzz_utility::RB_struct>::iterator it = wplist_map.find(tid);
    if (it != wplist_map.end())
      wplist_map.erase(it);
    wplist_map.insert(make_pair(tid, RB_arr));
  }

  ROS_INFO("----->Saved %i waypoints.", wplist_map.size());

  // Close the file:
  fin.close();
}

void check_targets_sim(double lat, double lon, double *res)
/*
/ check if a listed target is close
----------------------------------------------------------- */
{
  map<int, buzz_utility::RB_struct>::iterator it;
  for (it = wplist_map.begin(); it != wplist_map.end(); ++it)
  {
    double rb[3];
    double ref[2]={lat, lon};
    double tar[2]={it->second.latitude, it->second.longitude};
    rb_from_gps(tar, rb, ref);
    if(rb[0]<3.0){
      ROS_WARN("FOUND A TARGET!!! [%i]", it->first);
      res[0] = it->first;
      res[1] = it->second.latitude;
      res[2] = it->second.longitude;
      res[3] = it->second.altitude;
    }
  }
}

int buzz_exportmap(buzzvm_t vm)
/*
/ Buzz closure to export a 2D map
/----------------------------------------*/
{
  grid.clear();
  buzzvm_lnum_assert(vm, 1);
  // Get the parameter
  buzzvm_lload(vm, 1);
  buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);    // dictionary
  buzzobj_t t = buzzvm_stack_at(vm, 1);
  for(int32_t i = 1; i <= buzzdict_size(t->t.value); ++i) {
    buzzvm_dup(vm);
    buzzvm_pushi(vm, i);
    buzzvm_tget(vm);
    std::map<int, int> row;
    for(int32_t j = 1; j <= buzzdict_size(buzzvm_stack_at(vm, 1)->t.value); ++j) {
      buzzvm_dup(vm);
      buzzvm_pushi(vm, j);
      buzzvm_tget(vm);
      row.insert(std::pair<int,int>(j, 100.0 - round(buzzvm_stack_at(vm, 1)->f.value*100.0)));
      buzzvm_pop(vm);
    }
    grid.insert(std::pair<int,std::map<int, int>>(i,row));
    buzzvm_pop(vm);
  }
  // DEBUG
  // ROS_INFO("----- Recorded a grid of %i(%i)", grid.size(), buzzdict_size(t->t.value));
  return buzzvm_ret0(vm);
}

int buzzuav_moveto(buzzvm_t vm)
/*
/ Buzz closure to move following a 3D vector + Yaw
/----------------------------------------*/
{
  buzzvm_lnum_assert(vm, 4);
  buzzvm_lload(vm, 1);  // dx
  buzzvm_lload(vm, 2);  // dy
  buzzvm_lload(vm, 3);  // dheight
  buzzvm_lload(vm, 4);  // dyaw
  buzzvm_type_assert(vm, 4, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  float dyaw = buzzvm_stack_at(vm, 1)->f.value;
  float dh = buzzvm_stack_at(vm, 2)->f.value;
  float dy = buzzvm_stack_at(vm, 3)->f.value;
  float dx = buzzvm_stack_at(vm, 4)->f.value;
  goto_pos[0] = dx;
  goto_pos[1] = dy;
  goto_pos[2] = height + dh;
  goto_pos[3] = dyaw;
  //  DEBUG
  // ROS_WARN("[%i] Buzz requested Move To: x: %.7f , y: %.7f, z: %.7f", (int)buzz_utility::get_robotid(), goto_pos[0],
  // goto_pos[1], goto_pos[2]);
  buzz_cmd = NAV_SPLINE_WAYPOINT;
  return buzzvm_ret0(vm);
}

int buzzuav_addtargetRB(buzzvm_t vm)
/*
/ Buzz closure to add a target (goal) GPS
/----------------------------------------*/
{
  buzzvm_lnum_assert(vm, 3);
  buzzvm_lload(vm, 1);  // longitude
  buzzvm_lload(vm, 2);  // latitude
  buzzvm_lload(vm, 3);  // id
  buzzvm_type_assert(vm, 3, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  double tmp[3];
  tmp[0] = buzzvm_stack_at(vm, 2)->f.value;
  tmp[1] = buzzvm_stack_at(vm, 1)->f.value;
  tmp[2] = 0.0;
  int uid = buzzvm_stack_at(vm, 3)->i.value;
  double rb[3];

  rb_from_gps(tmp, rb, cur_pos);
  if (fabs(rb[0]) < 100.0)
  {
    buzz_utility::RB_struct RB_arr;
    RB_arr.latitude = tmp[0];
    RB_arr.longitude = tmp[1];
    RB_arr.altitude = tmp[2];
    RB_arr.r = rb[0];
    RB_arr.b = rb[1];
    map<int, buzz_utility::RB_struct>::iterator it = targets_map.find(uid);
    if (it != targets_map.end())
      targets_map.erase(it);
    targets_map.insert(make_pair(uid, RB_arr));
    //  DEBUG
    // ROS_INFO("Buzz_utility got updated/new user: %i (%f,%f,%f)", id, latitude, longitude, altitude);
    return vm->state;
  }
  else
    ROS_WARN(" ---------- Target too far %f", rb[0]);

  return 0;
}

int buzzuav_addNeiStatus(buzzvm_t vm)
/*
/ closure to add neighbors status to the BVM
/----------------------------------------*/
{
  buzzvm_lnum_assert(vm, 1);
  buzzvm_lload(vm, 1);  // state table
  buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);
  buzzobj_t t = buzzvm_stack_at(vm, 1);
  if(buzzdict_size(t->t.value) != 5) {
    ROS_WARN("Wrong neighbor status size.");
    return vm->state;
  }

  buzz_utility::neighbors_status newRS;
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "id", 1));
  buzzvm_tget(vm);
  uint8_t id = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "ba", 1));
  buzzvm_tget(vm);
  newRS.batt_lvl = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "gp", 1));
  buzzvm_tget(vm);
  newRS.gps_strenght = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "xb", 1));
  buzzvm_tget(vm);
  newRS.xbee = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "st", 1));
  buzzvm_tget(vm);
  newRS.flight_status = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);
  
  map<int, buzz_utility::neighbors_status>::iterator it = neighbors_status_map.find(id);
  if (it != neighbors_status_map.end())
    neighbors_status_map.erase(it);
  neighbors_status_map.insert(make_pair(id, newRS));
  return vm->state;
}

mavros_msgs::Mavlink get_status()
/*
/ return neighbors status from BVM
/----------------------------------------*/
{
  mavros_msgs::Mavlink payload_out;
  map<int, buzz_utility::neighbors_status>::iterator it;
  for (it = neighbors_status_map.begin(); it != neighbors_status_map.end(); ++it)
  {
    payload_out.payload64.push_back(it->first);
    payload_out.payload64.push_back(it->second.gps_strenght);
    payload_out.payload64.push_back(it->second.batt_lvl);
    payload_out.payload64.push_back(it->second.xbee);
    payload_out.payload64.push_back(it->second.flight_status);
  }
  //  Add Robot id and message number to the published message
  payload_out.sysid = (uint8_t)neighbors_status_map.size();

  return payload_out;
}

int buzzuav_resetrc(buzzvm_t vm)
/*
/ Buzz closure to reset the RC input.
/----------------------------------------*/
{
  rc_id = -1;
  return buzzvm_ret0(vm);
}

int buzzuav_takepicture(buzzvm_t vm)
/*
/ Buzz closure to take a picture here.
/----------------------------------------*/
{
  buzz_cmd = IMAGE_START_CAPTURE;
  return buzzvm_ret0(vm);
}

int buzzuav_setgimbal(buzzvm_t vm)
/*
/ Buzz closure to change locally the gimbal orientation
/----------------------------------------*/
{
  buzzvm_lnum_assert(vm, 4);
  buzzvm_lload(vm, 1);  // time
  buzzvm_lload(vm, 2);  // pitch
  buzzvm_lload(vm, 3);  // roll
  buzzvm_lload(vm, 4);  // yaw
  buzzvm_type_assert(vm, 4, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  rc_gimbal[0] = buzzvm_stack_at(vm, 4)->f.value;
  rc_gimbal[1] = buzzvm_stack_at(vm, 3)->f.value;
  rc_gimbal[2] = buzzvm_stack_at(vm, 2)->f.value;
  rc_gimbal[3] = buzzvm_stack_at(vm, 1)->f.value;

  ROS_INFO("Set RC_GIMBAL ---- %f %f %f (%f)", rc_gimbal[0], rc_gimbal[1], rc_gimbal[2], rc_gimbal[3]);
  buzz_cmd = DO_MOUNT_CONTROL;
  return buzzvm_ret0(vm);
}

int buzzuav_storegoal(buzzvm_t vm)
/*
/ Buzz closure to store locally a GPS destination from the fleet
/----------------------------------------*/
{
  buzzvm_lnum_assert(vm, 3);
  buzzvm_lload(vm, 1);  // altitude
  buzzvm_lload(vm, 2);  // longitude
  buzzvm_lload(vm, 3);  // latitude
  buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  double goal[3];
  goal[0] = buzzvm_stack_at(vm, 3)->f.value;
  goal[1] = buzzvm_stack_at(vm, 2)->f.value;
  goal[2] = buzzvm_stack_at(vm, 1)->f.value;
  if (goal[0] == -1 && goal[1] == -1 && goal[2] == -1)
  {
    if (wplist_map.size() <= 0)
      parse_gpslist();
    goal[0] = wplist_map.begin()->second.latitude;
    goal[1] = wplist_map.begin()->second.longitude;
    goal[2] = wplist_map.begin()->second.altitude;
    wplist_map.erase(wplist_map.begin()->first);
  }

  set_gpsgoal(goal);
  // prevent an overwrite
  rc_id = -1;

  return buzzvm_ret0(vm);
}

void set_gpsgoal(double goal[3])
/*
/ update GPS goal value
-----------------------------------*/
{
  double rb[3];
  rb_from_gps(goal, rb, cur_pos);
  if (fabs(rb[0]) < 150.0) {
    goto_gpsgoal[0] = goal[0];goto_gpsgoal[1] = goal[1];goto_gpsgoal[2] = goal[2];
    ROS_INFO("[%i] Set GPS GOAL TO ---- %f %f %f (%f %f, %f %f)", buzz_utility::get_robotid(), goal[0], goal[1], goal[2], cur_pos[0], cur_pos[1], rb[0], rb[1]);
  } else
    ROS_WARN("[%i] GPS GOAL TOO FAR !!-- %f %f %f (%f %f, %f %f)", buzz_utility::get_robotid(), goal[0], goal[1], goal[2], cur_pos[0], cur_pos[1], rb[0], rb[1]);

}

int buzzuav_arm(buzzvm_t vm)
/*
/ Buzz closure to arm
/---------------------------------------*/
{
  cur_cmd = COMPONENT_ARM_DISARM;
  printf(" Buzz requested Arm \n");
  buzz_cmd = cur_cmd;
  return buzzvm_ret0(vm);
}

int buzzuav_disarm(buzzvm_t vm)
/*
/ Buzz closure to disarm
/---------------------------------------*/
{
  cur_cmd = COMPONENT_ARM_DISARM + 1;
  printf(" Buzz requested Disarm  \n");
  buzz_cmd = cur_cmd;
  return buzzvm_ret0(vm);
}

int buzzuav_takeoff(buzzvm_t vm)
/*
/ Buzz closure to takeoff
/---------------------------------------*/
{
  buzzvm_lnum_assert(vm, 1);
  buzzvm_lload(vm, 1); /* Altitude */
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  goto_pos[2] = buzzvm_stack_at(vm, 1)->f.value;
  height = goto_pos[2];
  cur_cmd = NAV_TAKEOFF;
  printf(" Buzz requested Take off !!! \n");
  buzz_cmd = cur_cmd;
  return buzzvm_ret0(vm);
}

int buzzuav_land(buzzvm_t vm)
/*
/ Buzz closure to land
/-------------------------------------------------------------*/
{
  cur_cmd = NAV_LAND;
  printf(" Buzz requested Land !!! \n");
  buzz_cmd = cur_cmd;
  return buzzvm_ret0(vm);
}

int buzzuav_gohome(buzzvm_t vm)
/*
/ Buzz closure to return Home
/-------------------------------------------------------------*/
{
  cur_cmd = NAV_RETURN_TO_LAUNCH;
  printf(" Buzz requested gohome !!! \n");
  buzz_cmd = cur_cmd;
  return buzzvm_ret0(vm);
}

double* getgoto()
/*
/ return the GPS goal
/-------------------------------------------------------------*/
{
  return goto_pos;
}

std::map<int, std::map<int,int>> getgrid()
/*
/ return the grid
/-------------------------------------------------------------*/
{
  return grid;
}

float* getgimbal()
/*
/ return current gimbal commands
---------------------------------------*/
{
  return rc_gimbal;
}

int getcmd()
/*
/ return current mavros command to the FC
---------------------------------------*/
{
  return cur_cmd;
}

int bzz_cmd()
/*
/ return and clean the custom command from Buzz to the FC
----------------------------------------------------------*/
{
  int cmd = buzz_cmd;
  buzz_cmd = 0;
  return cmd;
}

void rc_set_goto(int id, double latitude, double longitude, double altitude)
/*
/ update interface RC GPS goal input
-----------------------------------*/
{
  rc_id = id;
  rc_gpsgoal[0] = latitude;
  rc_gpsgoal[1] = longitude;
  rc_gpsgoal[2] = altitude;
}

void rc_set_gimbal(int id, float yaw, float roll, float pitch, float t)
/*
/ update interface RC gimbal control input
-----------------------------------*/
{
  rc_id = id;
  rc_gimbal[0] = yaw;
  rc_gimbal[1] = roll;
  rc_gimbal[2] = pitch;
  rc_gimbal[3] = t;
}

void rc_call(int rc_cmd_in)
/*
/ update interface RC command input
-----------------------------------*/
{
  rc_cmd = rc_cmd_in;
}

void set_obstacle_dist(float dist[])
/*
/ update interface proximity value array
-----------------------------------*/
{
  for (int i = 0; i < 5; i++)
    obst[i] = dist[i];
}

void set_battery(float voltage, float current, float remaining)
/*
/ update interface battery value array
-----------------------------------*/
{
  batt[0] = voltage;
  batt[1] = current;
  batt[2] = remaining;
}

int buzzuav_update_battery(buzzvm_t vm)
/*
/ update BVM battery table
-----------------------------------*/
{
  buzzvm_pushs(vm, buzzvm_string_register(vm, "battery", 1));
  buzzvm_pusht(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "voltage", 1));
  buzzvm_pushf(vm, batt[0]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "current", 1));
  buzzvm_pushf(vm, batt[1]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "capacity", 1));
  buzzvm_pushi(vm, (int)batt[2]);
  buzzvm_tput(vm);
  buzzvm_gstore(vm);
  return vm->state;
}

/*
/ Set of function to update interface variable of xbee network status
----------------------------------------------------------------------*/
void set_deque_full(bool state)
{
  deque_full = state;
}

void set_rssi(float value)
{
  rssi = round(value);
}

void set_raw_packet_loss(float value)
{
  raw_packet_loss = value;
}

void set_filtered_packet_loss(float value)
{
  filtered_packet_loss = round(100 * value);
}

/*void set_api_rssi(float value)
{
  api_rssi = value;
}*/

int buzzuav_update_xbee_status(buzzvm_t vm)
/*
/ update BVM xbee_status table
-----------------------------------*/
{
  buzzvm_pushs(vm, buzzvm_string_register(vm, "xbee_status", 1));
  buzzvm_pusht(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "deque_full", 1));
  buzzvm_pushi(vm, static_cast<uint8_t>(deque_full));
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "rssi", 1));
  buzzvm_pushi(vm, rssi);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "raw_packet_loss", 1));
  buzzvm_pushf(vm, raw_packet_loss);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "filtered_packet_loss", 1));
  buzzvm_pushi(vm, filtered_packet_loss);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "api_rssi", 1));
  buzzvm_pushf(vm, api_rssi);
  buzzvm_tput(vm);
  buzzvm_gstore(vm);
  return vm->state;
}

void set_currentNEDpos(double x, double y)
/*
/ update interface position array
-----------------------------------*/
{
  cur_NEDpos[0] = x;
  cur_NEDpos[1] = y;
}

void set_currentpos(double latitude, double longitude, float altitude, float yaw)
/*
/ update interface position array
-----------------------------------*/
{
  cur_pos[0] = latitude;
  cur_pos[1] = longitude;
  cur_pos[2] = altitude;
  cur_pos[3] = yaw;
}
//  adds neighbours position
void neighbour_pos_callback(int id, float range, float bearing, float elevation)
{
  buzz_utility::Pos_struct pos_arr;
  pos_arr.x = range;
  pos_arr.y = bearing;
  pos_arr.z = elevation;
  map<int, buzz_utility::Pos_struct>::iterator it = neighbors_map.find(id);
  if (it != neighbors_map.end())
    neighbors_map.erase(it);
  neighbors_map.insert(make_pair(id, pos_arr));
}

//  update at each step the VM table
void update_neighbors(buzzvm_t vm)
{
  //   Reset neighbor information
  buzzneighbors_reset(vm);
  //  Get robot id and update neighbor information
  map<int, buzz_utility::Pos_struct>::iterator it;
  for (it = neighbors_map.begin(); it != neighbors_map.end(); ++it)
  {
    buzzneighbors_add(vm, it->first, (it->second).x, (it->second).y, (it->second).z);
  }
}

// Clear neighbours pos
void clear_neighbours_pos(){
  neighbors_map.clear();
}

int buzzuav_update_currentpos(buzzvm_t vm)
/*
/ Update the BVM position table
/------------------------------------------------------*/
{
  buzzvm_pushs(vm, buzzvm_string_register(vm, "pose", 1));
  buzzvm_pusht(vm);
  buzzobj_t tPoseTable = buzzvm_stack_at(vm, 1);
  buzzvm_gstore(vm);

  //  Create table for i-th read
  buzzvm_pusht(vm);
  buzzobj_t tPosition = buzzvm_stack_at(vm, 1);
  buzzvm_pop(vm);
  //  Fill in the read
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "latitude", 0));
  buzzvm_pushf(vm, cur_pos[0]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "longitude", 0));
  buzzvm_pushf(vm, cur_pos[1]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "altitude", 0));
  buzzvm_pushf(vm, cur_pos[2]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "x", 0));
  buzzvm_pushf(vm, cur_NEDpos[0]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "y", 0));
  buzzvm_pushf(vm, cur_NEDpos[1]);
  buzzvm_tput(vm);
  //  Store read table in the proximity table
  buzzvm_push(vm, tPoseTable);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "position", 0));
  buzzvm_push(vm, tPosition);
  buzzvm_tput(vm);

  //  Create table for i-th read
  buzzvm_pusht(vm);
  buzzobj_t tOrientation = buzzvm_stack_at(vm, 1);
  buzzvm_pop(vm);
  //  Fill in the read
  buzzvm_push(vm, tOrientation);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "yaw", 0));
  buzzvm_pushf(vm, cur_pos[3]);
  buzzvm_tput(vm);
  //  Store read table in the proximity table
  buzzvm_push(vm, tPoseTable);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "orientation", 0));
  buzzvm_push(vm, tOrientation);
  buzzvm_tput(vm);

  return vm->state;
}

void flight_status_update(uint8_t state)
/*
/ Update the interface status variable
/------------------------------------------------------*/
{
  status = state;
}

int buzzuav_update_flight_status(buzzvm_t vm)
/*
/ Create the generic robot table with status, remote controller current comand and destination
/ and current position of the robot
/------------------------------------------------------*/
{
  buzzvm_pushs(vm, buzzvm_string_register(vm, "flight", 1));
  buzzvm_pusht(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "rc_cmd", 1));
  buzzvm_pushi(vm, rc_cmd);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  rc_cmd = 0;
  buzzvm_pushs(vm, buzzvm_string_register(vm, "status", 1));
  buzzvm_pushi(vm, status);
  buzzvm_tput(vm);
  buzzvm_gstore(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "rc_goto", 1));
  buzzvm_pusht(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "id", 1));
  buzzvm_pushi(vm, rc_id);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "latitude", 1));
  buzzvm_pushf(vm, rc_gpsgoal[0]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "longitude", 1));
  buzzvm_pushf(vm, rc_gpsgoal[1]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "altitude", 1));
  buzzvm_pushf(vm, rc_gpsgoal[2]);
  buzzvm_tput(vm);
  buzzvm_gstore(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "cur_goal", 1));
  buzzvm_pusht(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "latitude", 1));
  buzzvm_pushf(vm, goto_gpsgoal[0]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "longitude", 1));
  buzzvm_pushf(vm, goto_gpsgoal[1]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "altitude", 1));
  buzzvm_pushf(vm, goto_gpsgoal[2]);
  buzzvm_tput(vm);
  buzzvm_gstore(vm);
  return vm->state;
}

int buzzuav_update_prox(buzzvm_t vm)
/*
/ Create an obstacle Buzz table from proximity sensors
/  Acessing proximity in buzz script
/    proximity[0].angle and proximity[0].value - front
/  ""          ""          "" 	      - right and back
/    proximity[3].angle and proximity[3].value - left
/    proximity[4].angle = -1 and proximity[4].value -bottom
-------------------------------------------------------------*/
{
  buzzvm_pushs(vm, buzzvm_string_register(vm, "proximity", 1));
  buzzvm_pusht(vm);
  buzzobj_t tProxTable = buzzvm_stack_at(vm, 1);
  buzzvm_gstore(vm);

  //  Fill into the proximity table
  buzzobj_t tProxRead;
  float angle = 0;
  for (size_t i = 0; i < 4; ++i)
  {
    //  Create table for i-th read
    buzzvm_pusht(vm);
    tProxRead = buzzvm_stack_at(vm, 1);
    buzzvm_pop(vm);
    //  Fill in the read
    buzzvm_push(vm, tProxRead);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "value", 0));
    buzzvm_pushf(vm, obst[i + 1]);
    buzzvm_tput(vm);
    buzzvm_push(vm, tProxRead);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "angle", 0));
    buzzvm_pushf(vm, angle);
    buzzvm_tput(vm);
    //  Store read table in the proximity table
    buzzvm_push(vm, tProxTable);
    buzzvm_pushi(vm, i);
    buzzvm_push(vm, tProxRead);
    buzzvm_tput(vm);
    angle += 1.5708;
  }
  //  Create table for bottom read
  angle = -1;
  buzzvm_pusht(vm);
  tProxRead = buzzvm_stack_at(vm, 1);
  buzzvm_pop(vm);
  //  Fill in the read
  buzzvm_push(vm, tProxRead);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "value", 0));
  buzzvm_pushf(vm, obst[0]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tProxRead);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "angle", 0));
  buzzvm_pushf(vm, angle);
  buzzvm_tput(vm);
  //  Store read table in the proximity table
  buzzvm_push(vm, tProxTable);
  buzzvm_pushi(vm, 4);
  buzzvm_push(vm, tProxRead);
  buzzvm_tput(vm);
  return vm->state;
}

int dummy_closure(buzzvm_t vm)
/*
/ Dummy closure for use during update testing
----------------------------------------------------*/
{
  return buzzvm_ret0(vm);
}
}

#pragma once

#include <buzz/buzzvm.h>
#include <stdio.h>
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/Mavlink.h"
#include "ros/ros.h"
#include <rosbuzz/buzz_utility.h>
#include <rosbuzz/mavrosCC.h>

#define EARTH_RADIUS (double)6371000.0
#define DEG2RAD(DEG) (double)((DEG) * ((M_PI) / (180.0)))
#define RAD2DEG(RAD) (double)((RAD) * ((180.0) / (M_PI)))

namespace buzzuav_closures
{
struct bounding_box_struct
{
  std::string obj_class;
  double probability;
  int64_t xmin;
  int64_t ymin;
  int64_t xmax;
  int64_t ymax;
  bounding_box_struct(std::string c, double prob, int64_t xmi, int64_t ymi, int64_t xma, int64_t yma)
    : obj_class(c), probability(prob), xmin(xmi), ymin(ymi), xmax(xma), ymax(yma){};
  bounding_box_struct()
  {
  }
};
typedef struct bounding_box_struct bounding_box;

/*
 * prextern int() function in Buzz
 * This function is used to print data from buzz
 * The command to use in Buzz is buzzros_print takes any available datatype in Buzz
 */
int buzzros_print(buzzvm_t vm);
void setWPlist(std::string file);
void setVorlog(std::string path);
void check_targets_sim(double lat, double lon, double* res);

/*
 * closure to move following a vector
 */
int buzzuav_moveto(buzzvm_t vm);
/*
 * closure to store a new GPS goal
 */
int buzzuav_storegoal(buzzvm_t vm);
/*
 * closure to control the gimbal
 */
int buzzuav_setgimbal(buzzvm_t vm);
/*
 * parse a csv list of waypoints
 */
void parse_gpslist();
/*
 * closure to export a 2D map
 */
int buzz_exportmap(buzzvm_t vm);
/*
 * closure to take a picture
 */
int buzzuav_takepicture(buzzvm_t vm);
/*
 * closure to reset RC input
 */
int buzzuav_resetrc(buzzvm_t vm);
/*
 * Returns the current command from local variable
 */
int getcmd();
/*
 * update GPS goal value
 */
void set_gpsgoal(double goal[3]);
/*
 * Sets goto position from rc client
 */
void rc_set_goto(int id, double latitude, double longitude, double altitude);
/*
 *Sets gimbal orientation from rc client
 */
void rc_set_gimbal(int id, float yaw, float roll, float pitch, float t);
/*
 * sets rc requested command
 */
void rc_call(int rc_cmd);
/*
 * sets the battery state
 */
void set_battery(float voltage, float current, float remaining);
/*
 * Update yolo boxes into buzz
 */
int buzzuav_update_yolo_boxes(buzzvm_t vm);
/*
 *Stores the received bounding boxes
 */
void store_bounding_boxes(std::vector<bounding_box> bbox);
/*
 * sets the xbee network status
 */
void set_deque_full(bool state);
void set_rssi(float value);
void set_raw_packet_loss(float value);
void set_filtered_packet_loss(float value);
// void set_api_rssi(float value);
/*
 * sets current position
 */

void set_currentNEDpos(double x, double y);

void set_currentpos(double latitude, double longitude, float altitude, float yaw);
/*
 * returns the current go to position
 */
double* getgoto();
/*
 * returns the current grid
 */
std::map<int, std::map<int, int>> getgrid();
int voronoi_center(buzzvm_t vm);

/*
 * returns the gimbal commands
 */
float* getgimbal();
/*
 *updates flight status
 */
void flight_status_update(uint8_t state);
/*
 *Update neighbors table
 */
void neighbour_pos_callback(int id, float range, float bearing, float elevation);
/*
 * update neighbors from in msgs
 */
void update_neighbors(buzzvm_t vm);
/*
 *Clear neighbours struct
 */
void clear_neighbours_pos();
/*
 * closure to add a neighbor status
 */
int buzzuav_addNeiStatus(buzzvm_t vm);
/*
 * returns the current array of neighbors status
 */
mavros_msgs::Mavlink get_status();

/*
 *Flight status
 */
void set_obstacle_dist(float dist[]);

/*
 * Commands the UAV to takeoff
 */
int buzzuav_takeoff(buzzvm_t vm);
/*
 * Arm command from Buzz
 */
int buzzuav_arm(buzzvm_t vm);
/*
 * Disarm from buzz
 */
int buzzuav_disarm(buzzvm_t vm);
/* Commands the UAV to land
 */
int buzzuav_land(buzzvm_t vm);

/*
 * Command the UAV to go to home location
 */
int buzzuav_gohome(buzzvm_t vm);
int buzzuav_geofence(buzzvm_t vm);

/*
 * Updates battery information in Buzz
 */
int buzzuav_update_battery(buzzvm_t vm);
/*
 * Updates xbee_status information in Buzz
 */
int buzzuav_update_xbee_status(buzzvm_t vm);
/*
 * Updates current position in Buzz
 */
int buzzuav_update_currentpos(buzzvm_t vm);
/*
 * add new target in the BVM
 */
int buzzuav_addtargetRB(buzzvm_t vm);
/*
 * Updates flight status and rc command in Buzz, put it in a tabel to acess it
 * use flight.status for flight status
 * use flight.rc_cmd for current rc cmd
 */
int buzzuav_update_flight_status(buzzvm_t vm);

/*
 * Updates IR information in Buzz
 * Proximity and ground sensors to do !!!!
 */
int buzzuav_update_prox(buzzvm_t vm);
/*
 * returns the current FC command
 */
int bzz_cmd();

int dummy_closure(buzzvm_t vm);
}

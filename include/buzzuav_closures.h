#pragma once

#include <buzz/buzzvm.h>
#include <stdio.h>
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/Mavlink.h"
#include "ros/ros.h"
#include "buzz_utility.h"

#define EARTH_RADIUS (double)6371000.0
#define DEG2RAD(DEG) (double)((DEG) * ((M_PI) / (180.0)))
#define RAD2DEG(RAD) (double)((RAD) * ((180.0) / (M_PI)))

namespace buzzuav_closures
{
typedef enum {
  COMMAND_NIL = 0,  // Dummy command
  COMMAND_TAKEOFF,  // Take off
  COMMAND_LAND,
  COMMAND_GOHOME,
  COMMAND_ARM,
  COMMAND_DISARM,
  COMMAND_GOTO,
  COMMAND_MOVETO,
  COMMAND_PICTURE,
  COMMAND_GIMBAL,
} Custom_CommandCode;

/*
 * prextern int() function in Buzz
 * This function is used to print data from buzz
 * The command to use in Buzz is buzzros_print takes any available datatype in Buzz
 */
int buzzros_print(buzzvm_t vm);
void setWPlist(std::string path);
/*
 * buzzuav_goto(latitude,longitude,altitude) function in Buzz
 * commands the UAV to go to a position supplied
 */
int buzz_floor(buzzvm_t vm);
int buzzuav_moveto(buzzvm_t vm);
int buzzuav_storegoal(buzzvm_t vm);
int buzzuav_setgimbal(buzzvm_t vm);
void parse_gpslist();
int buzzuav_takepicture(buzzvm_t vm);
/* Returns the current command from local variable*/
int getcmd();
/*Sets goto position from rc client*/
void rc_set_goto(int id, double latitude, double longitude, double altitude);
/*Sets gimbal orientation from rc client*/
void rc_set_gimbal(int id, float yaw, float roll, float pitch, float t);
/*sets rc requested command */
void rc_call(int rc_cmd);
/* sets the battery state */
void set_battery(float voltage, float current, float remaining);
void set_deque_full(bool state);
void set_rssi(float value);
void set_raw_packet_loss(float value);
void set_filtered_packet_loss(float value);
void set_api_rssi(float value);
/* sets current position */
void set_currentpos(double latitude, double longitude, double altitude);
/*retuns the current go to position */
double* getgoto();
std::string getuavstate();
float* getgimbal();
/* updates flight status*/
void flight_status_update(uint8_t state);
/* Update neighbors table */
void neighbour_pos_callback(int id, float range, float bearing, float elevation);
void update_neighbors(buzzvm_t vm);
int buzzuav_addNeiStatus(buzzvm_t vm);
mavros_msgs::Mavlink get_status();

/*Flight status*/
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

/* Command the UAV to go to home location
 */
int buzzuav_gohome(buzzvm_t vm);

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
int buzzuav_update_targets(buzzvm_t vm);
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

int bzz_cmd();

int dummy_closure(buzzvm_t vm);

//#endif
}

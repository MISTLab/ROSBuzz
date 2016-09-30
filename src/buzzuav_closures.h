#ifndef BUZZUAV_CLOSURES_H
#define BUZZUAV_CLOSURES_H

#include <buzz/buzzvm.h>

/*
 * prextern int() function in Buzz
 * This function is used to print data from buzz
 * The command to use in Buzz is buzzros_print takes any available datatype in Buzz
 */
int buzzros_print(buzzvm_t vm);

/*
 * buzzuav_goto(latitude,longitude,altitude) function in Buzz
 * commands the UAV to go to a position supplied
 */
int buzzuav_goto(buzzvm_t vm);
/* Returns the current command from local variable*/
int getcmd();

void set_goto(double pos[]);

void rc_call(int rc_cmd);

/* sets the battery state to the local variable
 */
void set_battery(float voltage,float current,float remaining);
/*retuns the current go to position from local variable*/
double* getgoto();
/*
 * Commands the UAV to takeoff
 */
int buzzuav_takeoff(buzzvm_t vm);

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
 * Updates IR information in Buzz
 * Proximity and ground sensors
 */
int buzzuav_update_prox(buzzvm_t vm);

#endif

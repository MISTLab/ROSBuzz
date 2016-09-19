#ifndef BUZZUAV_CLOSURES_H
#define BUZZUAV_CLOSURES_H

#include <buzz/buzzvm.h>

/*
 * prextern int() function in Buzz
 */
int buzzros_print(buzzvm_t vm);

/*
 * set_wheels(ls,rs) function in Buzz
 * Sets the wheel speeds to ls (left) and rs (right)
 * speeds are expressed in cm/s
 */
int buzzuav_goto(buzzvm_t vm);

int getcmd();
void set_battery(float voltage,float current,float remaining);
double* getgoto();
/*
 * set_leds(r,g,b) function in Buzz
 * Sets the color of the 3 leds to (r,g,b)
 * speeds are expressed in cm/s
 */
int buzzuav_takeoff(buzzvm_t vm);

/*
 * set_leds(r,g,b) function in Buzz
 * Sets the color of the 3 leds to (r,g,b)
 * speeds are expressed in cm/s
 */
int buzzuav_land(buzzvm_t vm);

/*
 * set_leds(r,g,b) function in Buzz
 * Sets the color of the 3 leds to (r,g,b)
 * speeds are expressed in cm/s
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

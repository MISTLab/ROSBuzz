#include "uav_utility.h"
#include "stdio.h"

/****************************************/
/****************************************/

//knet_dev_t* DSPIC;
static const int ACC_INC       = 3;
static const int ACC_DIV       = 0;
static const int MIN_SPEED_ACC = 20;
static const int MIN_SPEED_DEC = 1;
static const int MAX_SPEED     = 400; /* mm/sec */

/****************************************/
/****************************************/

void uav_setup() {
   /* Set the libkhepera debug level */
   /*kb_set_debug_level(2);
   // initiate libkhepera and robot access
   if(kh4_init(0, NULL) !=0)
   {
      printf("\nERROR: could not initialize the khepera!\n\n");
      return;
   }
   /* open robot socket and store the handle in their respective pointers 
   DSPIC = knet_open("Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);
   /* Set speed profile 
   kh4_SetSpeedProfile(ACC_INC,       /* Acceleration increment 
                       ACC_DIV,       /* Acceleration divider
                       MIN_SPEED_ACC, /* Minimum speed acc
                       MIN_SPEED_DEC, /* Minimum speed dec 
                       MAX_SPEED,     /* Maximum speed 
                       DSPIC
      );
   kh4_SetMode(kh4RegSpeedProfile, DSPIC);
   /* Mute ultrasonic sensor 
   kh4_activate_us(0, DSPIC);*/
}

/****************************************/
/****************************************/

void uav_done() {
  /* /* Stop wheels 
   kh4_set_speed(0, 0, DSPIC);
   /* Set motors to idle 
   kh4_SetMode(kh4RegIdle, DSPIC);
   /* Clear rgb leds because they consume energy 
   kh4_SetRGBLeds(0, 0, 0, 0, 0, 0, 0, 0, 0, DSPIC); */
   fprintf(stdout, "Robot stopped.\n");
}

/****************************************/
/****************************************/

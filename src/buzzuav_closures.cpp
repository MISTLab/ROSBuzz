/** @file      buzzuav_closures.cpp
 *  @version   1.0 
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS for Dji M100 Drone. 
 *  @author    Vivek Shankar Varadharajan
 *  @copyright 2016 MistLab. All rights reserved.
 */
//#define _GNU_SOURCE
#include "buzzuav_closures.h"
namespace buzzuav_closures{
static double goto_pos[3];
static double rc_goto_pos[3];
static float batt[3];
static double cur_pos[3];
static uint8_t status;
static int cur_cmd = 0;
static int rc_cmd=0;
static int buzz_cmd=0;
/****************************************/
/****************************************/

int buzzros_print(buzzvm_t vm) {
   int i;
   for(i = 1; i < buzzdarray_size(vm->lsyms->syms); ++i) {
      buzzvm_lload(vm, i);
      buzzobj_t o = buzzvm_stack_at(vm, 1);
      buzzvm_pop(vm);
      switch(o->o.type) {
         case BUZZTYPE_NIL:
	    ROS_INFO("BUZZ - [nil]");
            break;
         case BUZZTYPE_INT:
	    ROS_INFO("%d", o->i.value);
            //fprintf(stdout, "%d", o->i.value);
            break;
         case BUZZTYPE_FLOAT:
	    ROS_INFO("%f", o->f.value);
            break;
         case BUZZTYPE_TABLE:
            ROS_INFO("[table with %d elems]", (buzzdict_size(o->t.value)));
            break;
         case BUZZTYPE_CLOSURE:
            if(o->c.value.isnative)
		ROS_INFO("[n-closure @%d]", o->c.value.ref);
            else
		ROS_INFO("[c-closure @%d]", o->c.value.ref);
            break;
         case BUZZTYPE_STRING:
	    ROS_INFO("%s", o->s.value.str);
            break;
         case BUZZTYPE_USERDATA:
            ROS_INFO("[userdata @%p]", o->u.value);
            break;
         default:
            break;
      }
   }
   //fprintf(stdout, "\n");
   return buzzvm_ret0(vm);
}

/****************************************/
/****************************************/

int buzzuav_goto(buzzvm_t vm) {
   buzzvm_lnum_assert(vm, 3);
   buzzvm_lload(vm, 1); /* Altitude */
   buzzvm_lload(vm, 2); /* Longitude */
   buzzvm_lload(vm, 3); /* Latitude */
   buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
   buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
   buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
   goto_pos[2]=buzzvm_stack_at(vm, 1)->f.value; 
   goto_pos[1]=buzzvm_stack_at(vm, 2)->f.value; 
   goto_pos[0]=buzzvm_stack_at(vm, 3)->f.value; 
   cur_cmd=mavros_msgs::CommandCode::NAV_WAYPOINT;
   printf(" Buzz requested Go To, to Latitude: %15f , Longitude: %15f, Altitude: %15f  \n",goto_pos[0],goto_pos[1],goto_pos[2]);
   buzz_cmd=2;
   return buzzvm_ret0(vm);
}

/******************************/

double* getgoto(){
return goto_pos;
}
/******************************/
int getcmd(){
 return cur_cmd;
}

void set_goto(double pos[]){
goto_pos[0]=pos[0];
goto_pos[1]=pos[1];
goto_pos[2]=pos[2];
    
}

int bzz_cmd(){
int cmd = buzz_cmd;
buzz_cmd=0;
return cmd;
}

void rc_set_goto(double pos[]){
rc_goto_pos[0]=pos[0];
rc_goto_pos[1]=pos[1];
rc_goto_pos[2]=pos[2];
    
}
void rc_call(int rc_cmd_in){
rc_cmd=rc_cmd_in;
}

/****************************************/
/****************************************/

int buzzuav_takeoff(buzzvm_t vm) {
   cur_cmd=mavros_msgs::CommandCode::NAV_TAKEOFF;
   printf(" Buzz requested Take off !!! \n");
   buzz_cmd=1;
   return buzzvm_ret0(vm);
}

int buzzuav_land(buzzvm_t vm) {
   cur_cmd=mavros_msgs::CommandCode::NAV_LAND;
   printf(" Buzz requested Land !!! \n");
   buzz_cmd=1;
   return buzzvm_ret0(vm);
}

int buzzuav_gohome(buzzvm_t vm) {
   cur_cmd=mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
   printf(" Buzz requested gohome !!! \n");
   buzz_cmd=1;
   return buzzvm_ret0(vm);
}

/****************************************/
void set_battery(float voltage,float current,float remaining){
 batt[0]=voltage;
 batt[1]=current;
 batt[2]=remaining;
}
/****************************************/

int buzzuav_update_battery(buzzvm_t vm) {
   //static char BATTERY_BUF[256];
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
   buzzvm_pushf(vm, batt[2]);
   buzzvm_tput(vm);
   buzzvm_gstore(vm);
   return vm->state;
}
/****************************************/
/*current pos update*/
/***************************************/
void set_currentpos(double latitude, double longitude, double altitude){
   cur_pos[0]=latitude;
   cur_pos[1]=longitude;
   cur_pos[2]=altitude;
}
/****************************************/
int buzzuav_update_currentpos(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "position", 1));
   buzzvm_pusht(vm);
   buzzvm_dup(vm);
   buzzvm_pushs(vm, buzzvm_string_register(vm, "latitude", 1));
   buzzvm_pushf(vm, cur_pos[0]);
   buzzvm_tput(vm);
   buzzvm_dup(vm);
   buzzvm_pushs(vm, buzzvm_string_register(vm, "longitude", 1));
   buzzvm_pushf(vm, cur_pos[1]);
   buzzvm_tput(vm);
   buzzvm_dup(vm);
   buzzvm_pushs(vm, buzzvm_string_register(vm, "altitude", 1));
   buzzvm_pushf(vm, cur_pos[2]);
   buzzvm_tput(vm);
   buzzvm_gstore(vm);
   return vm->state;
}
/****************************************/
/*flight status update*/
/***************************************/
void flight_status_update(uint8_t state){
   status=state;
}
/****************************************/
int buzzuav_update_flight_status(buzzvm_t vm) {
   buzzvm_pushs(vm, buzzvm_string_register(vm, "flight", 1));
   buzzvm_pusht(vm);
   buzzvm_dup(vm);
   buzzvm_pushs(vm, buzzvm_string_register(vm, "rc_cmd", 1));
   buzzvm_pushi(vm, rc_cmd);
   buzzvm_tput(vm);
   buzzvm_dup(vm);
   rc_cmd=0;
   buzzvm_pushs(vm, buzzvm_string_register(vm, "status", 1));
   buzzvm_pushi(vm, status);
   buzzvm_tput(vm); 
   buzzvm_gstore(vm);
   //also set rc_controllers goto
   buzzvm_pushs(vm, buzzvm_string_register(vm, "rc_goto", 1));
   buzzvm_pusht(vm);
   buzzvm_dup(vm);
   buzzvm_pushs(vm, buzzvm_string_register(vm, "latitude", 1));
   buzzvm_pushf(vm, rc_goto_pos[0]);
   buzzvm_tput(vm);
   buzzvm_dup(vm);
   buzzvm_pushs(vm, buzzvm_string_register(vm, "longitude", 1));
   buzzvm_pushf(vm, rc_goto_pos[1]);
   buzzvm_tput(vm);
   buzzvm_dup(vm);
   buzzvm_pushs(vm, buzzvm_string_register(vm, "altitude", 1));
   buzzvm_pushf(vm, rc_goto_pos[2]);
   buzzvm_tput(vm);
   buzzvm_gstore(vm);
   return vm->state;
}



/****************************************/
/*To do !!!!!*/
/****************************************/

int buzzuav_update_prox(buzzvm_t vm) {
 /*  static char PROXIMITY_BUF[256];
   int i;
   //kh4_proximity_ir(PROXIMITY_BUF, DSPIC);
   buzzvm_pushs(vm, buzzvm_string_register(vm, "proximity_ir", 1));
   buzzvm_pusht(vm);
   for(i = 0; i < 8; i++) {
      buzzvm_dup(vm);
      buzzvm_pushi(vm, i+1);
      buzzvm_pushi(vm, (PROXIMITY_BUF[i*2] | PROXIMITY_BUF[i*2+1] << 8));
      buzzvm_tput(vm);
   }
   buzzvm_gstore(vm);
   buzzvm_pushs(vm, buzzvm_string_register(vm, "ground_ir", 1));
   buzzvm_pusht(vm);
   for(i = 8; i < 12; i++) {
      buzzvm_dup(vm);
      buzzvm_pushi(vm, i-7);
      buzzvm_pushi(vm, (PROXIMITY_BUF[i*2] | PROXIMITY_BUF[i*2+1] << 8));
      buzzvm_tput(vm);
   }
   buzzvm_gstore(vm);*/
   return vm->state;
}

/****************************************/
/****************************************/

int dummy_closure(buzzvm_t vm){ return buzzvm_ret0(vm);}

}


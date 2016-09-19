#define _GNU_SOURCE
#include <stdio.h>
#include "buzzuav_closures.h"
#include "uav_utility.h"
#include "mavros_msgs/CommandCode.h"

// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%

double goto_pos[3];
int cur_cmd;
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
            //fprintf(stdout, "[nil]");
	    ROS_INFO("BUZZ - [nil]");
            break;
         case BUZZTYPE_INT:
            //fprintf(stdout, "%d", o->i.value);
            break;
         case BUZZTYPE_FLOAT:
            //fprintf(stdout, "%f", o->f.value);
            break;
         case BUZZTYPE_TABLE:
            //fprintf(stdout, "[table with %d elems]", (buzzdict_size(o->t.value)));
            break;
         case BUZZTYPE_CLOSURE:
            if(o->c.value.isnative)
		ROS_INFO("BUZZ - [nil]");
               //fprintf(stdout, "[n-closure @%d]", o->c.value.ref);
            else
		ROS_INFO("BUZZ - [nil]");
               //fprintf(stdout, "[c-closure @%d]", o->c.value.ref);
            break;
         case BUZZTYPE_STRING:
            //fprintf(stdout, "%s", o->s.value.str);
            break;
         case BUZZTYPE_USERDATA:
            //fprintf(stdout, "[userdata @%p]", o->u.value);
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
   buzzvm_lload(vm, 1); /* Lattitude */
   buzzvm_lload(vm, 2); /* Longitude */
   buzzvm_lload(vm, 3); /* Altitude */
   buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
   buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
   buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
goto_pos[0]=buzzvm_stack_at(vm, 1)->f.value * 10.0f;
goto_pos[1]=buzzvm_stack_at(vm, 2)->f.value * 10.0f;
goto_pos[2]=buzzvm_stack_at(vm, 3)->f.value * 10.0f;
   return buzzvm_ret0(vm);
}

double* getgoto(){
return goto_pos;
}

int getcmd(){
return cur_cmd;
}

/****************************************/
/****************************************/

int buzzuav_takeoff(buzzvm_t vm) {
   cur_cmd=mavros_msgs::CommandCode::NAV_TAKEOFF;
   return buzzvm_ret0(vm);
}

int buzzuav_land(buzzvm_t vm) {
   cur_cmd=mavros_msgs::CommandCode::NAV_LAND;
   return buzzvm_ret0(vm);
}

int buzzuav_gohome(buzzvm_t vm) {
   cur_cmd=mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
   return buzzvm_ret0(vm);
}

/****************************************/
float batt[3];
void set_battery(float voltage,float current,float remaining){
batt[0]=voltage;
batt[1]=current;
batt[2]=remaining;
}
/****************************************/

int buzzuav_update_battery(buzzvm_t vm) {
   static char BATTERY_BUF[256];
 //  kh4_battery_status(BATTERY_BUF, DSPIC);
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
/****************************************/

int buzzuav_update_prox(buzzvm_t vm) {
   static char PROXIMITY_BUF[256];
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
   buzzvm_gstore(vm);
   return vm->state;
}

/****************************************/
/****************************************/

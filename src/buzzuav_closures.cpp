/** @file      buzzuav_closures.cpp
 *  @version   1.0 
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS for Dji M100 Drone. 
 *  @author    Vivek Shankar Varadharajan
 *  @copyright 2016 MistLab. All rights reserved.
 */
//#define _GNU_SOURCE
#include "buzzuav_closures.h"
#include "buzz_utility.h"
namespace buzzuav_closures{
static double goto_pos[3];
static double rc_goto_pos[3];
static float batt[3];
static double cur_pos[3];
static uint8_t status;
static int cur_cmd = 0;
static int rc_cmd=0;
static int buzz_cmd=0;
static float height=0;
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
#define EARTH_RADIUS 6371000.0
/*convert from spherical to cartesian coordinate system callback */
        void cartesian_coordinates(double spherical_pos_payload [], double out[]){
                double latitude, longitude, rho;
                latitude = spherical_pos_payload[0] / 180.0 * M_PI;
                longitude = spherical_pos_payload[1] / 180.0 * M_PI;
                rho = spherical_pos_payload[2]+EARTH_RADIUS; //centered on Earth
                try {
                        out[0] = cos(latitude) * cos(longitude) * rho;
                        out[1] = cos(latitude) * sin(longitude) * rho;
                        out[2] = sin(latitude) * rho; // z is 'up'
                } catch (std::overflow_error e) {
//                std::cout << e.what() << " Error in convertion to cartesian coordinate system "<<endl;
                }
        }
/*convert from cartesian to spherical coordinate system callback */
        void spherical_coordinates(double cartesian_pos_payload [], double out[]){
                double x, y, z;
                x = cartesian_pos_payload[0];
                y = cartesian_pos_payload[1];
                z = cartesian_pos_payload[2];
                try {
                out[2]=sqrt(pow(x,2.0)+pow(y,2.0)+pow(z,2.0))-EARTH_RADIUS;
                out[1]=atan2(y,x)*180.0/M_PI;
                out[0]=atan2(z,(sqrt(pow(x,2.0)+pow(y,2.0))))*180.0/M_PI;
                } catch (std::overflow_error e) {
//                std::cout << e.what() << " Error in convertion to spherical coordinate system "<<endl;
                }
        }

void gps_from_rb(double  range, double bearing, double out[3]) {
	double lat = cur_pos[0]*M_PI/180.0;
	double lon = cur_pos[1]*M_PI/180.0;
//	bearing = bearing*M_PI/180.0;
 	out[0] = asin(sin(lat) * cos(range/EARTH_RADIUS) + cos(lat) * sin(range/EARTH_RADIUS) * cos(bearing));
	out[0] = out[0]*180.0/M_PI;
	out[1] = lon + atan2(sin(bearing) * sin(range/EARTH_RADIUS) * cos(lat), cos(range/EARTH_RADIUS) - sin(lat)*sin(out[0]));
	out[1] = out[1]*180.0/M_PI;
	out[2] = height; //constant height.
}

double hcpos1[4] = {45.564489, -73.562537, 45.564140, -73.562336};
double hcpos2[4] = {45.564729, -73.562060, 45.564362, -73.562958};
double hcpos3[4] = {45.564969, -73.562838, 45.564636, -73.563677};

int buzzuav_moveto(buzzvm_t vm) {
   buzzvm_lnum_assert(vm, 2);
   buzzvm_lload(vm, 1); /* dx */
   buzzvm_lload(vm, 2); /* dy */
   //buzzvm_lload(vm, 3); /* Latitude */
   //buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
   buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
   buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
   float dy = buzzvm_stack_at(vm, 1)->f.value;
   float dx = buzzvm_stack_at(vm, 2)->f.value;
   double d = sqrt(dx*dx+dy*dy);	//range
   double b = atan2(dy,dx);		//bearing
   printf(" Vector for Goto: %.7f,%.7f\n",dx,dy);
   //goto_pos[0]=buzzvm_stack_at(vm, 3)->f.value;
   //double cur_pos_cartesian[3];
   //cartesian_coordinates(cur_pos,cur_pos_cartesian);
   //double goto_cartesian[3];
   //goto_cartesian[0] = dx + cur_pos_cartesian[0];
   //goto_cartesian[1] = dy + cur_pos_cartesian[1];
   //goto_cartesian[2] = cur_pos_cartesian[2];
   //spherical_coordinates(goto_cartesian, goto_pos);
//   goto_pos[0]=dx;
//   goto_pos[1]=dy;
   //goto_pos[2]=height;	// force a constant altitude to avoid loop increases
   gps_from_rb(d, b, goto_pos);
/*   if(dx == 1.0){
	if((int)buzz_utility::get_robotid()==1){
		goto_pos[0]=hcpos1[0];goto_pos[1]=hcpos1[1];goto_pos[2]=height;
	}
	if((int)buzz_utility::get_robotid()==2){
		goto_pos[0]=hcpos2[0];goto_pos[1]=hcpos2[1];goto_pos[2]=height;
	}
	if((int)buzz_utility::get_robotid()==3){
		goto_pos[0]=hcpos3[0];goto_pos[1]=hcpos3[1];goto_pos[2]=height;
	}
   }
   if(dx == 2.0){
	if((int)buzz_utility::get_robotid()==1){
		goto_pos[0]=hcpos1[2];goto_pos[1]=hcpos1[3];goto_pos[2]=height;
	}
	if((int)buzz_utility::get_robotid()==2){
		goto_pos[0]=hcpos2[2];goto_pos[1]=hcpos2[3];goto_pos[2]=height;
	}
	if((int)buzz_utility::get_robotid()==3){
		goto_pos[0]=hcpos3[2];goto_pos[1]=hcpos3[3];goto_pos[2]=height;
	}
   }*/
   cur_cmd=mavros_msgs::CommandCode::NAV_WAYPOINT;
   printf(" Buzz requested Go To, to Latitude: %.7f , Longitude: %.7f, Altitude: %.7f  \n",goto_pos[0],goto_pos[1],goto_pos[2]);
   buzz_cmd=2;
   return buzzvm_ret0(vm);
}

int buzzuav_goto(buzzvm_t vm) {
   set_goto(rc_goto_pos);
   cur_cmd=mavros_msgs::CommandCode::NAV_WAYPOINT;
   printf(" Buzz requested Go To, to Latitude: %.7f , Longitude: %.7f, Altitude: %.7f  \n",goto_pos[0],goto_pos[1],goto_pos[2]);
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
   buzzvm_lnum_assert(vm, 1);
   buzzvm_lload(vm, 1); /* Altitude */
   buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
   goto_pos[2]=buzzvm_stack_at(vm, 1)->f.value;
   height = goto_pos[2];
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


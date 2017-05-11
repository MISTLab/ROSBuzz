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

	// TODO: Minimize the required global variables and put them in the header
	//static const rosbzz_node::roscontroller* roscontroller_ptr;
	/*forward declaration of ros controller ptr storing function*/
	//void set_ros_controller_ptr(const rosbzz_node::roscontroller* roscontroller_ptrin);
	static double goto_pos[3];
	static double rc_goto_pos[3];
	static float batt[3];
	static float obst[5]={0,0,0,0,0};
	static double cur_pos[3];
	static uint8_t status;
	static int cur_cmd = 0;
	static int rc_cmd=0;
	static int buzz_cmd=0;
	static float height=0;


	std::map< int,  buzz_utility::Pos_struct> neighbors_map;

	/****************************************/
	/****************************************/

	int buzzros_print(buzzvm_t vm) {
	int i;
	char buffer [50] = "";
        sprintf(buffer,"%s [%i]", buffer, (int)buzz_utility::get_robotid());
	   for(i = 1; i < buzzdarray_size(vm->lsyms->syms); ++i) {
	      buzzvm_lload(vm, i);
	      buzzobj_t o = buzzvm_stack_at(vm, 1);
	      buzzvm_pop(vm);
	      switch(o->o.type) {
		 case BUZZTYPE_NIL:
		    sprintf(buffer,"%s BUZZ - [nil]", buffer);
		    break;
		 case BUZZTYPE_INT:
		    sprintf(buffer,"%s %d", buffer, o->i.value);
		    //fprintf(stdout, "%d", o->i.value);
		    break;
		 case BUZZTYPE_FLOAT:
		    sprintf(buffer,"%s %f", buffer, o->f.value);
		    break;
		 case BUZZTYPE_TABLE:
		    sprintf(buffer,"%s [table with %d elems]", buffer, (buzzdict_size(o->t.value)));
		    break;
		 case BUZZTYPE_CLOSURE:
		    if(o->c.value.isnative)
		    	sprintf(buffer,"%s [n-closure @%d]", buffer, o->c.value.ref);
		    else
		    	sprintf(buffer,"%s [c-closure @%d]", buffer, o->c.value.ref);
		    break;
		 case BUZZTYPE_STRING:
		    sprintf(buffer,"%s %s", buffer, o->s.value.str);
		    break;
		 case BUZZTYPE_USERDATA:
		    sprintf(buffer,"%s [userdata @%p]", buffer, o->u.value);
		    break;
		 default:
		    break;
	      }
	   }
	   ROS_INFO(buffer);
	   //fprintf(stdout, "\n");
	   return buzzvm_ret0(vm);
	}

	/*----------------------------------------/
	/ Compute GPS destination from current position and desired Range and Bearing
	/----------------------------------------*/

	void gps_from_rb(double  range, double bearing, double out[3]) {
		double lat = cur_pos[0]*M_PI/180.0;
		double lon = cur_pos[1]*M_PI/180.0;
	 	out[0] = asin(sin(lat) * cos(range/EARTH_RADIUS) + cos(lat) * sin(range/EARTH_RADIUS) * cos(bearing));
		out[1] = lon + atan2(sin(bearing) * sin(range/EARTH_RADIUS) * cos(lat), cos(range/EARTH_RADIUS) - sin(lat)*sin(out[0]));
		out[0] = out[0]*180.0/M_PI;
		out[1] = out[1]*180.0/M_PI;
		out[2] = height; //constant height.
	}

	void rb_from_gps(double nei[], double out[], double cur[]){   
       	double d_lon = nei[1] - cur[1];
       	double d_lat = nei[0] - cur[0];
        double ned_x = DEG2RAD(d_lat) * EARTH_RADIUS;
        double ned_y = DEG2RAD(d_lon) * EARTH_RADIUS * cos(DEG2RAD(nei[0]));
		out[0] = sqrt(ned_x*ned_x+ned_y*ned_y);
		out[1] = atan2(ned_y,ned_x);
		out[2] = 0.0;
	}

	// Hard coded GPS position in Park Maisonneuve, Montreal, Canada for simulation tests
	double hcpos1[4] = {45.564489, -73.562537, 45.564140, -73.562336};
	double hcpos2[4] = {45.564729, -73.562060, 45.564362, -73.562958};
	double hcpos3[4] = {45.564969, -73.562838, 45.564636, -73.563677};

	/*----------------------------------------/
	/ Buzz closure to move following a 2D vector
	/----------------------------------------*/
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
	   goto_pos[0]=dx;
       goto_pos[1]=dy;
       goto_pos[2]=height;
	   /*double b = atan2(dy,dx);		//bearing
	   printf(" Vector for Goto: %.7f,%.7f\n",dx,dy);
	   gps_from_rb(d, b, goto_pos);
	   cur_cmd=mavros_msgs::CommandCode::NAV_WAYPOINT;*/
	   printf(" Buzz requested Move To: x: %.7f , y: %.7f, z: %.7f  \n",goto_pos[0], goto_pos[1], goto_pos[2]);
	   buzz_cmd= COMMAND_MOVETO; // TO DO what should we use
	   return buzzvm_ret0(vm);
	}

	int users_add2localtable(buzzvm_t vm, int id, float range, float bearing) {
		if(vm->state != BUZZVM_STATE_READY) return vm->state;
		buzzvm_pushs(vm, buzzvm_string_register(vm, "users", 1));
		buzzvm_gload(vm);
		buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);
		buzzobj_t nbr = buzzvm_stack_at(vm, 1);
		/* Get "data" field */
		buzzvm_pushs(vm, buzzvm_string_register(vm, "dataL", 1));
		buzzvm_tget(vm);
		if(buzzvm_stack_at(vm, 1)->o.type == BUZZTYPE_NIL) {
			//ROS_INFO("Empty data, create a new table");
			buzzvm_pop(vm);
			buzzvm_push(vm, nbr);
			buzzvm_pushs(vm, buzzvm_string_register(vm, "dataL", 1));
			buzzvm_pusht(vm);
			buzzobj_t data = buzzvm_stack_at(vm, 1);
			buzzvm_tput(vm);
			buzzvm_push(vm, data);
		}
		/* When we get here, the "data" table is on top of the stack */
		/* Push user id */
		buzzvm_pushi(vm, id);
		/* Create entry table */
		buzzobj_t entry = buzzheap_newobj(vm->heap, BUZZTYPE_TABLE);
		/* Insert range */
		buzzvm_push(vm, entry);
		buzzvm_pushs(vm, buzzvm_string_register(vm, "r", 1));
		buzzvm_pushf(vm, range);
		buzzvm_tput(vm);
		/* Insert longitude */
		buzzvm_push(vm, entry);
		buzzvm_pushs(vm, buzzvm_string_register(vm, "b", 1));
		buzzvm_pushf(vm, bearing);
		buzzvm_tput(vm);
		/* Save entry into data table */
		buzzvm_push(vm, entry);
		buzzvm_tput(vm);
		//printf("\tBuzz_closure saved new user: %i (%f,%f)\n", id, range, bearing);
		return vm->state;
	}

	int buzzuav_adduserRB(buzzvm_t vm) {
	   buzzvm_lnum_assert(vm, 3);
	   buzzvm_lload(vm, 1); /* longitude */
	   buzzvm_lload(vm, 2); /* latitude */
	   buzzvm_lload(vm, 3); /* id */
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

	   //printf("\tGot new user from bzz stig: %i - %f, %f\n", uid, rb[0], rb[1]);

	   return users_add2localtable(vm, uid, rb[0], rb[1]);
	}

	/*----------------------------------------/
	/ Buzz closure to go directly to a GPS destination from the Mission Planner
	/----------------------------------------*/
	int buzzuav_goto(buzzvm_t vm) {
	   rc_goto_pos[2]=height;
	   set_goto(rc_goto_pos);
	   cur_cmd=mavros_msgs::CommandCode::NAV_WAYPOINT;
	   printf(" Buzz requested Go To, to Latitude: %.7f , Longitude: %.7f, Altitude: %.7f  \n",goto_pos[0],goto_pos[1],goto_pos[2]);
	   buzz_cmd=COMMAND_GOTO;
	   return buzzvm_ret0(vm);
	}

	/*----------------------------------------/
	/ Buzz closure to arm/disarm the drone, useful for field tests to ensure all systems are up and running
	/----------------------------------------*/
	int buzzuav_arm(buzzvm_t vm) {
	   cur_cmd=mavros_msgs::CommandCode::CMD_COMPONENT_ARM_DISARM;
	   printf(" Buzz requested Arm \n");
	   buzz_cmd=COMMAND_ARM;
	   return buzzvm_ret0(vm);
	}
	int buzzuav_disarm(buzzvm_t vm) {
	   cur_cmd=mavros_msgs::CommandCode::CMD_COMPONENT_ARM_DISARM + 1;
	   printf(" Buzz requested Disarm  \n");
	   buzz_cmd=COMMAND_DISARM;
	   return buzzvm_ret0(vm);
	}

	/*---------------------------------------/
	/ Buss closure for basic UAV commands
	/---------------------------------------*/
	int buzzuav_takeoff(buzzvm_t vm) {
	   buzzvm_lnum_assert(vm, 1);
	   buzzvm_lload(vm, 1); /* Altitude */
	   buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
	   goto_pos[2] = buzzvm_stack_at(vm, 1) -> f.value;
	   height = goto_pos[2];
	   cur_cmd=mavros_msgs::CommandCode::NAV_TAKEOFF;
	   printf(" Buzz requested Take off !!! \n");
	   buzz_cmd = COMMAND_TAKEOFF;
	   return buzzvm_ret0(vm);
	}

	int buzzuav_land(buzzvm_t vm) {
	   cur_cmd=mavros_msgs::CommandCode::NAV_LAND;
	   printf(" Buzz requested Land !!! \n");
	   buzz_cmd = COMMAND_LAND;
	   return buzzvm_ret0(vm);
	}

	int buzzuav_gohome(buzzvm_t vm) {
	   cur_cmd=mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
	   printf(" Buzz requested gohome !!! \n");
	   buzz_cmd = COMMAND_GOHOME;
	   return buzzvm_ret0(vm);
	}


	/*-------------------------------
	/ Get/Set to transfer variable from Roscontroller to buzzuav
	/------------------------------------*/
	double* getgoto() {
		return goto_pos;
	}

	int getcmd() {
		return cur_cmd;
	}

	void set_goto(double pos[]) {
		goto_pos[0] = pos[0];
		goto_pos[1] = pos[1];
		goto_pos[2] = pos[2];

	}

	int bzz_cmd() {
		int cmd = buzz_cmd;
		buzz_cmd = 0;
		return cmd;
	}

	void rc_set_goto(double pos[]) {
		rc_goto_pos[0] = pos[0];
		rc_goto_pos[1] = pos[1];
		rc_goto_pos[2] = pos[2];

	}
	void rc_call(int rc_cmd_in) {
		rc_cmd = rc_cmd_in;
	}

	void set_obstacle_dist(float dist[]) {
		for (int i = 0; i < 5; i++)
			obst[i] = dist[i];
	}

	void set_battery(float voltage,float current,float remaining){
	 batt[0]=voltage;
	 batt[1]=current;
	 batt[2]=remaining;
	}

	int buzzuav_update_battery(buzzvm_t vm) {
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
	/*adds neighbours position*/
	void neighbour_pos_callback(int id, float range, float bearing, float elevation){
		buzz_utility::Pos_struct pos_arr;
		pos_arr.x=range;
		pos_arr.y=bearing;
		pos_arr.z=elevation;
		map< int, buzz_utility::Pos_struct >::iterator it = neighbors_map.find(id);
		if(it!=neighbors_map.end())
			neighbors_map.erase(it);
		neighbors_map.insert(make_pair(id, pos_arr));
	}

	/* update at each step the VM table */
	void update_neighbors(buzzvm_t vm){
		/* Reset neighbor information */
    	buzzneighbors_reset(vm);
  		/* Get robot id and update neighbor information */
  	  	map< int, buzz_utility::Pos_struct >::iterator it;
		for (it=neighbors_map.begin(); it!=neighbors_map.end(); ++it){
			buzzneighbors_add(vm,
								it->first,
								(it->second).x,
								(it->second).y,
								(it->second).z);
		}
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

	void flight_status_update(uint8_t state){
	   status=state;
	}

	/*----------------------------------------------------
	/ Create the generic robot table with status, remote controller current comand and destination
	/ and current position of the robot
	/ TODO: change global name for robot
	/------------------------------------------------------*/
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



	/******************************************************/
	/*Create an obstacle Buzz table from proximity sensors*/
	/*  Acessing proximity in buzz script 
	    proximity[0].angle and proximity[0].value - front
		""          ""          "" 	      - right and back
	    proximity[3].angle and proximity[3].value - left
	    proximity[4].angle = -1 and proximity[4].value -bottom */
	/******************************************************/

	int buzzuav_update_prox(buzzvm_t vm) {

	buzzvm_pushs(vm, buzzvm_string_register(vm, "proximity", 1));
        buzzvm_pusht(vm);
        buzzobj_t tProxTable = buzzvm_stack_at(vm, 1);
        buzzvm_gstore(vm);
      
        /* Fill into the proximity table */
        buzzobj_t tProxRead;
        float angle =0;
        for(size_t i = 0; i < 4; ++i) {
           /* Create table for i-th read */
           buzzvm_pusht(vm);
           tProxRead = buzzvm_stack_at(vm, 1);
           buzzvm_pop(vm);
           /* Fill in the read */
	   buzzvm_push(vm, tProxRead);
   	   buzzvm_pushs(vm, buzzvm_string_register(vm, "value", 0));
   	   buzzvm_pushf(vm, obst[i+1]);
   	   buzzvm_tput(vm);
	   buzzvm_push(vm, tProxRead);
   	   buzzvm_pushs(vm, buzzvm_string_register(vm, "angle", 0));
   	   buzzvm_pushf(vm, angle);
   	   buzzvm_tput(vm);
           /* Store read table in the proximity table */
          buzzvm_push(vm, tProxTable);
   	  buzzvm_pushi(vm, i);
  	  buzzvm_push(vm, tProxRead);
 	  buzzvm_tput(vm);
	   angle+=1.5708;
        }
	/* Create table for bottom read */
	 angle =-1;
         buzzvm_pusht(vm);
         tProxRead = buzzvm_stack_at(vm, 1);
         buzzvm_pop(vm);
         /* Fill in the read */
	 buzzvm_push(vm, tProxRead);
   	 buzzvm_pushs(vm, buzzvm_string_register(vm, "value", 0));
   	 buzzvm_pushf(vm, obst[0]);
   	 buzzvm_tput(vm);
	 buzzvm_push(vm, tProxRead);
   	 buzzvm_pushs(vm, buzzvm_string_register(vm, "angle", 0));
   	 buzzvm_pushf(vm, angle);
   	 buzzvm_tput(vm);
	 /*Store read table in the proximity table*/
	 buzzvm_push(vm, tProxTable);
   	 buzzvm_pushi(vm, 4);
  	 buzzvm_push(vm, tProxRead);
 	 buzzvm_tput(vm);

	 /*
	  buzzvm_pushs(vm, buzzvm_string_register(vm, "proximity", 1));
	   buzzvm_pusht(vm);
	   buzzvm_dup(vm);
	   buzzvm_pushs(vm, buzzvm_string_register(vm, "bottom", 1));
	   buzzvm_pushf(vm, obst[0]);
	   buzzvm_tput(vm);
	   buzzvm_dup(vm);
	   buzzvm_pushs(vm, buzzvm_string_register(vm, "front", 1));
	   buzzvm_pushf(vm, obst[1]);
	   buzzvm_tput(vm);
	   buzzvm_dup(vm);
	   buzzvm_pushs(vm, buzzvm_string_register(vm, "right", 1));
	   buzzvm_pushf(vm, obst[2]);
	   buzzvm_tput(vm);
	   buzzvm_dup(vm);
	   buzzvm_pushs(vm, buzzvm_string_register(vm, "back", 1));
	   buzzvm_pushf(vm, obst[3]);
	   buzzvm_tput(vm);
	   buzzvm_dup(vm);
	   buzzvm_pushs(vm, buzzvm_string_register(vm, "left", 1));
	   buzzvm_pushf(vm, obst[4]);
	   buzzvm_tput(vm);
	   buzzvm_gstore(vm);*/
	   return vm->state;
	}

	/**********************************************/
	/*Dummy closure for use during update testing */
	/**********************************************/

	int dummy_closure(buzzvm_t vm){ return buzzvm_ret0(vm);}

	/***********************************************/
	/* Store Ros controller object pointer           */
	/***********************************************/
	
	//void set_ros_controller_ptr(const rosbzz_node::roscontroller* roscontroller_ptrin){
	//roscontroller_ptr = roscontroller_ptrin;
	//} 

}

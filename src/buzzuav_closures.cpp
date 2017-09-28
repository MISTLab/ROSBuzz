/** @file      buzzuav_closures.cpp
 *  @version   1.0
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS for Dji M100 Drone.
 *  @author    Vivek Shankar Varadharajan
 *  @copyright 2016 MistLab. All rights reserved.
 */
//#define _GNU_SOURCE
#include "buzzuav_closures.h"
#include "math.h"

namespace buzzuav_closures{
	// TODO: Minimize the required global variables and put them in the header
	//static const rosbzz_node::roscontroller* roscontroller_ptr;
	/*forward declaration of ros controller ptr storing function*/
	//void set_ros_controller_ptr(const rosbzz_node::roscontroller* roscontroller_ptrin);
	static double goto_pos[3];
	static double rc_goto_pos[3];
	static float rc_gimbal[4];
	static float batt[3];
	static float obst[5]={0,0,0,0,0};
	static double cur_pos[3];
	static uint8_t status;
	static int cur_cmd = 0;
	static int rc_cmd=0;
	static int rc_id=-1;
	static int buzz_cmd=0;
	static float height=0;
	static bool deque_full = false;
	static int rssi = 0;
	static float raw_packet_loss = 0.0;
	static int filtered_packet_loss = 0;
	static float api_rssi = 0.0;
  	string WPlistname = "";

	std::map< int,  buzz_utility::RB_struct> targets_map;
	std::map< int,  buzz_utility::RB_struct> wplist_map;
	std::map< int,  buzz_utility::Pos_struct> neighbors_map;
	std::map< int, buzz_utility::neighbors_status> neighbors_status_map;

	/****************************************/
	/****************************************/

	int buzzros_print(buzzvm_t vm) {
	int i;
	char buffer [100] = "";
        sprintf(buffer,"%s [%i]", buffer, (int)buzz_utility::get_robotid());
	   for(uint32_t i = 1; i < buzzdarray_size(vm->lsyms->syms); ++i) {
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

  void setWPlist(string path){
    WPlistname = path + "include/graphs/waypointlist.csv";
  }

	/*----------------------------------------/
	/ Compute GPS destination from current position and desired Range and Bearing
	/----------------------------------------*/

	void gps_from_rb(double  range, double bearing, double out[3]) {
		double lat = RAD2DEG(cur_pos[0]);
		double lon = RAD2DEG(cur_pos[1]);
	 	out[0] = asin(sin(lat) * cos(range/EARTH_RADIUS) + cos(lat) * sin(range/EARTH_RADIUS) * cos(bearing));
		out[1] = lon + atan2(sin(bearing) * sin(range/EARTH_RADIUS) * cos(lat), cos(range/EARTH_RADIUS) - sin(lat)*sin(out[0]));
		out[0] = RAD2DEG(out[0]);
		out[1] = RAD2DEG(out[1]);
		out[2] = height; //constant height.
	}

	float constrainAngle(float x){
			x = fmod(x,2*M_PI);
			if (x < 0.0)
				x += 2*M_PI;
			return x;
		}

	void rb_from_gps(double nei[], double out[], double cur[]){
    double d_lon = nei[1] - cur[1];
    double d_lat = nei[0] - cur[0];
    double ned_x = DEG2RAD(d_lat) * EARTH_RADIUS;
    double ned_y = DEG2RAD(d_lon) * EARTH_RADIUS * cos(DEG2RAD(nei[0]));
		out[0] = sqrt(ned_x*ned_x+ned_y*ned_y);
		out[1] = constrainAngle(atan2(ned_y,ned_x));
		out[2] = 0.0;
	}

	// Hard coded GPS position in Park Maisonneuve, Montreal, Canada for simulation tests
	double hcpos1[4] = {45.564489, -73.562537, 45.564140, -73.562336};
	double hcpos2[4] = {45.564729, -73.562060, 45.564362, -73.562958};
	double hcpos3[4] = {45.564969, -73.562838, 45.564636, -73.563677};

	void parse_gpslist()
	{
		// Open file:
    ROS_INFO("WP list file: %s", WPlistname.c_str());
		std::ifstream fin(WPlistname.c_str()); // Open in text-mode.

		// Opening may fail, always check.
		if (!fin) { 
			ROS_ERROR("GPS list parser, could not open file."); 
			return;
		}

    // Prepare a C-string buffer to be used when reading lines.
    const int MAX_LINE_LENGTH = 1024; // Choose this large enough for your need.
    char buffer[MAX_LINE_LENGTH] = {}; 
    const char * DELIMS = "\t ,"; // Tab, space or comma.

		// Read the file and load the data:
		double lat, lon;
    int alt, tilt, tid;
		buzz_utility::RB_struct RB_arr;
    // Read one line at a time.
    while ( fin.getline(buffer, MAX_LINE_LENGTH) ) {
      // Extract the tokens:
      tid = atoi(strtok( buffer, DELIMS ));
      lon = atof(strtok( NULL,   DELIMS ));
      lat = atof(strtok( NULL,   DELIMS ));
      alt = atoi(strtok( NULL,   DELIMS ));
      tilt = atoi(strtok( NULL,   DELIMS ));
      //ROS_INFO("%.6f, %.6f, %i %i %i",lat, lon, alt, tilt, tid);
			RB_arr.latitude=lat;
			RB_arr.longitude=lon;
			RB_arr.altitude=alt;
      // Insert elements.
      map< int, buzz_utility::RB_struct >::iterator it = wplist_map.find(tid);
      if(it!=wplist_map.end())
        wplist_map.erase(it);
      wplist_map.insert(make_pair(tid, RB_arr));
    }

    ROS_INFO("----->Saved %i waypoints.", wplist_map.size());

		// Close the file:
		fin.close();
	}

	/*----------------------------------------/
	/ Buzz closure to move following a 2D vector
	/----------------------------------------*/
	int buzzuav_moveto(buzzvm_t vm) {
    buzzvm_lnum_assert(vm, 3);
    buzzvm_lload(vm, 1); /* dx */
    buzzvm_lload(vm, 2); /* dy */
    buzzvm_lload(vm, 3); /* dheight */
    buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
    buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
    buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
    float dh = buzzvm_stack_at(vm, 1)->f.value;
    float dy = buzzvm_stack_at(vm, 2)->f.value;
    float dx = buzzvm_stack_at(vm, 3)->f.value;
    goto_pos[0]=dx;
    goto_pos[1]=dy;
    goto_pos[2]=height+dh;
    /*double b = atan2(dy,dx);		//bearing
    printf(" Vector for Goto: %.7f,%.7f\n",dx,dy);
    gps_from_rb(d, b, goto_pos);
    cur_cmd=mavros_msgs::CommandCode::NAV_WAYPOINT;*/
      //printf(" Vector for Goto: %.7f,%.7f\n",dx,dy);
    //ROS_WARN("[%i] Buzz requested Move To: x: %.7f , y: %.7f, z: %.7f", (int)buzz_utility::get_robotid(), goto_pos[0], goto_pos[1], goto_pos[2]);
    buzz_cmd = COMMAND_MOVETO; // TO DO what should we use
    return buzzvm_ret0(vm);
	}

	int buzzuav_update_targets(buzzvm_t vm) {
		if(vm->state != BUZZVM_STATE_READY) return vm->state;
		buzzvm_pushs(vm, buzzvm_string_register(vm, "targets", 1));
		//buzzobj_t t = buzzheap_newobj(vm->heap, BUZZTYPE_TABLE);
		//buzzvm_push(vm, t);
		buzzvm_pusht(vm);
		buzzobj_t targettbl = buzzvm_stack_at(vm, 1);
		//buzzvm_tput(vm);
		//buzzvm_dup(vm);
		double rb[3], tmp[3];
		map< int, buzz_utility::RB_struct >::iterator it;
		for (it=targets_map.begin(); it!=targets_map.end(); ++it){
			tmp[0]=(it->second).latitude; tmp[1]=(it->second).longitude; tmp[2]=height;
	   	rb_from_gps(tmp, rb, cur_pos);
			//ROS_WARN("----------Pushing target id %i (%f,%f)", it->first, rb[0], rb[1]);
			buzzvm_push(vm, targettbl);
			// When we get here, the "targets" table is on top of the stack
			//ROS_INFO("Buzz_utility will save user %i.", it->first);
			// Push user id
			buzzvm_pushi(vm, it->first);
			// Create entry table
			buzzobj_t entry = buzzheap_newobj(vm->heap, BUZZTYPE_TABLE);
			// Insert range
			buzzvm_push(vm, entry);
			buzzvm_pushs(vm, buzzvm_string_register(vm, "range", 1));
			buzzvm_pushf(vm, rb[0]);
			buzzvm_tput(vm);
			// Insert longitude
			buzzvm_push(vm, entry);
			buzzvm_pushs(vm, buzzvm_string_register(vm, "bearing", 1));
			buzzvm_pushf(vm, rb[1]);
			buzzvm_tput(vm);
			// Save entry into data table
			buzzvm_push(vm, entry);
			buzzvm_tput(vm);
		}
		buzzvm_gstore(vm);

		return vm->state;
	}

	int buzzuav_addtargetRB(buzzvm_t vm) {
	   buzzvm_lnum_assert(vm, 3);
	   buzzvm_lload(vm, 1); // longitude
	   buzzvm_lload(vm, 2); // latitude
	   buzzvm_lload(vm, 3); // id
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
	   if(fabs(rb[0])<100.0) {
			//printf("\tGot new user from bzz stig: %i - %f, %f\n", uid, rb[0], rb[1]);
			buzz_utility::RB_struct RB_arr;
			RB_arr.latitude=tmp[0];
			RB_arr.longitude=tmp[1];
			RB_arr.altitude=tmp[2];
			RB_arr.r=rb[0];
			RB_arr.b=rb[1];
			map< int, buzz_utility::RB_struct >::iterator it = targets_map.find(uid);
			if(it!=targets_map.end())
				targets_map.erase(it);
			targets_map.insert(make_pair(uid, RB_arr));
			//ROS_INFO("Buzz_utility got updated/new user: %i (%f,%f,%f)", id, latitude, longitude, altitude);
			return vm->state;
	   } else
		ROS_WARN(" ---------- Target too far %f",rb[0]);

		return 0;
	}

	int buzzuav_addNeiStatus(buzzvm_t vm){
		buzzvm_lnum_assert(vm, 5);
	  buzzvm_lload(vm, 1); // fc
	  buzzvm_lload(vm, 2); // xbee
	  buzzvm_lload(vm, 3); // batt
	  buzzvm_lload(vm, 4); // gps
	  buzzvm_lload(vm, 5); // id
	  buzzvm_type_assert(vm, 5, BUZZTYPE_INT);
	  buzzvm_type_assert(vm, 4, BUZZTYPE_INT);
	  buzzvm_type_assert(vm, 3, BUZZTYPE_INT);
	  buzzvm_type_assert(vm, 2, BUZZTYPE_INT);
	  buzzvm_type_assert(vm, 1, BUZZTYPE_INT);
		buzz_utility::neighbors_status newRS;
		uint8_t id = buzzvm_stack_at(vm, 5)->i.value;
    newRS.gps_strenght= buzzvm_stack_at(vm, 4)->i.value;
    newRS.batt_lvl= buzzvm_stack_at(vm, 3)->i.value;
    newRS.xbee= buzzvm_stack_at(vm, 2)->i.value;
    newRS.flight_status= buzzvm_stack_at(vm, 1)->i.value;
    map< int, buzz_utility::neighbors_status >::iterator it = neighbors_status_map.find(id);
		if(it!=neighbors_status_map.end())
			neighbors_status_map.erase(it);
		neighbors_status_map.insert(make_pair(id, newRS));
		return vm->state;
	}

	mavros_msgs::Mavlink get_status(){
		mavros_msgs::Mavlink payload_out;
    map< int, buzz_utility::neighbors_status >::iterator it;
		for (it= neighbors_status_map.begin(); it!= neighbors_status_map.end(); ++it){
			payload_out.payload64.push_back(it->first);
			payload_out.payload64.push_back(it->second.gps_strenght);
			payload_out.payload64.push_back(it->second.batt_lvl);
			payload_out.payload64.push_back(it->second.xbee);
			payload_out.payload64.push_back(it->second.flight_status);
		}
    	/*Add Robot id and message number to the published message*/
		payload_out.sysid = (uint8_t)neighbors_status_map.size();
		/*payload_out.msgid = (uint32_t)message_number;

		message_number++;*/
		return payload_out;
	}
	/*----------------------------------------/
	/ Buzz closure to take a picture here.
	/----------------------------------------*/
	int buzzuav_takepicture(buzzvm_t vm) {
	  //cur_cmd = mavros_msgs::CommandCode::CMD_DO_SET_CAM_TRIGG_DIST;
	  buzz_cmd = COMMAND_PICTURE;
	  return buzzvm_ret0(vm);
	}

	/*----------------------------------------/
	/ Buzz closure to change locally the gimbal orientation
	/----------------------------------------*/
	int buzzuav_setgimbal(buzzvm_t vm) {
	  buzzvm_lnum_assert(vm, 4);
	  buzzvm_lload(vm, 1); // time
	  buzzvm_lload(vm, 2); // pitch
	  buzzvm_lload(vm, 3); // roll
	  buzzvm_lload(vm, 4); // yaw
	  buzzvm_type_assert(vm, 4, BUZZTYPE_FLOAT);
	  buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
	  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
	  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
	  rc_gimbal[0] = buzzvm_stack_at(vm, 4)->f.value;
	  rc_gimbal[1] = buzzvm_stack_at(vm, 3)->f.value;
	  rc_gimbal[2] = buzzvm_stack_at(vm, 2)->f.value;
	  rc_gimbal[3] = buzzvm_stack_at(vm, 1)->f.value;

	  ROS_WARN("Set RC_GIMBAL ---- %f %f %f (%f)",rc_gimbal[0],rc_gimbal[1],rc_gimbal[2],rc_gimbal[3]);
	  buzz_cmd = COMMAND_GIMBAL;
	  return buzzvm_ret0(vm);
	}

	/*----------------------------------------/
	/ Buzz closure to store locally a GPS destination from the fleet
	/----------------------------------------*/
	int buzzuav_storegoal(buzzvm_t vm) {
	  buzzvm_lnum_assert(vm, 3);
	  buzzvm_lload(vm, 1); // altitude
	  buzzvm_lload(vm, 2); // longitude
	  buzzvm_lload(vm, 3); // latitude
	  buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
	  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
	  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
    double goal[3];
		goal[0] = buzzvm_stack_at(vm, 3)->f.value;
		goal[1] = buzzvm_stack_at(vm, 2)->f.value;
		goal[2] = buzzvm_stack_at(vm, 1)->f.value;
    if(goal[0]==-1 && goal[1]==-1 && goal[2]==-1){
      if(wplist_map.size()<=0)
        parse_gpslist();
      goal[0] = wplist_map.begin()->second.latitude;
      goal[1] = wplist_map.begin()->second.longitude;
      goal[2] = wplist_map.begin()->second.altitude;
      wplist_map.erase(wplist_map.begin()->first);
    }

    double rb[3];

    rb_from_gps(goal, rb, cur_pos);
    if(fabs(rb[0])<150.0)
		  rc_set_goto((int)buzz_utility::get_robotid(), goal[0], goal[1], goal[2]);

	  ROS_WARN("Set RC_GOTO ---- %f %f %f (%f %f, %f %f)",goal[0],goal[1],goal[2],cur_pos[0],cur_pos[1],rb[0],rb[1]);
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
	/ Buzz closure for basic UAV commands
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

	float* getgimbal() {
		return rc_gimbal;
	}

	string getuavstate() {
		static buzzvm_t     VM = buzz_utility::get_vm();
		std::stringstream state_buff;
      	buzzvm_pushs(VM, buzzvm_string_register(VM, "UAVSTATE",1));
        buzzvm_gload(VM);
        buzzobj_t uav_state = buzzvm_stack_at(VM, 1);
        buzzvm_pop(VM);
		return uav_state->s.value.str;
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

	void rc_set_goto(int id, double latitude, double longitude, double altitude) {
		rc_id = id;
		rc_goto_pos[0] = latitude;
		rc_goto_pos[1] = longitude;
		rc_goto_pos[2] = altitude;

	}

	void rc_set_gimbal(int id, float yaw, float roll, float pitch, float t) {

		rc_id = id;
		rc_gimbal[0] = yaw;
		rc_gimbal[1] = roll;
		rc_gimbal[2] = pitch;
		rc_gimbal[3] = t;

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
	   buzzvm_pushi(vm, (int)batt[2]);
	   buzzvm_tput(vm);
	   buzzvm_gstore(vm);
	   return vm->state;
	}

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
   filtered_packet_loss = round(100*value);
  }

  void set_api_rssi(float value)
  {
   api_rssi = value;
  }

  int buzzuav_update_xbee_status(buzzvm_t vm) {
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

	/***************************************/
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
     buzzvm_pushs(vm, buzzvm_string_register(vm, "id", 1));
	   buzzvm_pushi(vm, rc_id);
	   buzzvm_tput(vm);
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
  /* Store Ros controller object pointer         */
  /***********************************************/

  //void set_ros_controller_ptr(const rosbzz_node::roscontroller* roscontroller_ptrin){
  //roscontroller_ptr = roscontroller_ptrin;
  //}

}

/** @file      buzz_utility.cpp
 *  @version   1.0 
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS for Dji M100 Drone. 
 *  @author    Vivek Shankar Varadharajan
 *  @copyright 2016 MistLab. All rights reserved.
 */

#include "buzz_utility.h"

namespace buzz_utility{

	/****************************************/
	/****************************************/

	static buzzvm_t     VM              = 0;
	static char*        BO_FNAME        = 0;
	static uint8_t*     BO_BUF          = 0;
	static buzzdebug_t  DBG_INFO        = 0;
	static uint8_t      MSG_SIZE        = 250;   // Only 100 bytes of Buzz messages every step
	static int          MAX_MSG_SIZE    = 10000; // Maximum Msg size for sending update packets 
	static int 	    Robot_id        = 0;
	static std::vector<uint8_t*> IN_MSG;
	std::map< int,  Pos_struct> users_map;
	
	/****************************************/

	void add_user(int id, double latitude, double longitude, float altitude)
	{
		Pos_struct pos_arr;
		pos_arr.x=latitude;
		pos_arr.y=longitude;
		pos_arr.z=altitude;
		map< int, buzz_utility::Pos_struct >::iterator it = users_map.find(id);
		if(it!=users_map.end())
			users_map.erase(it);
		users_map.insert(make_pair(id, pos_arr));
		//ROS_INFO("Buzz_utility got updated/new user: %i (%f,%f,%f)", id, latitude, longitude, altitude);
	}

	void update_users(){
		if(users_map.size()>0) {
			/* Reset users information */
			buzzusers_reset();
			/* Get user id and update user information */
			map< int, Pos_struct >::iterator it;
			for (it=users_map.begin(); it!=users_map.end(); ++it){
				//ROS_INFO("Buzz_utility will save user %i.", it->first);
				buzzusers_add(it->first,
									(it->second).x,
									(it->second).y,
									(it->second).z);
			}
		}/*else
			ROS_INFO("[%i] No new users",Robot_id);*/
	}

	int buzzusers_reset() {
		if(VM->state != BUZZVM_STATE_READY) return VM->state;
		/* Make new table */
		buzzobj_t t = buzzheap_newobj(VM->heap, BUZZTYPE_TABLE);
		//make_table(&t);
		if(VM->state != BUZZVM_STATE_READY) return VM->state;
		/* Register table as global symbol */
		/*buzzvm_pushs(VM, buzzvm_string_register(VM, "vt", 1));
		buzzvm_gload(VM);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "put", 1));
        buzzvm_tget(VM);*/
        buzzvm_pushs(VM, buzzvm_string_register(VM, "users", 1));
		buzzvm_push(VM, t);
		buzzvm_gstore(VM);
        //buzzvm_pushi(VM, 2);
		//buzzvm_callc(VM);
		return VM->state;
	}

	int buzzusers_add(int id, double latitude, double longitude, double altitude) {
		if(VM->state != BUZZVM_STATE_READY) return VM->state;
		/* Get users "p" table */
		/*buzzvm_pushs(VM, buzzvm_string_register(VM, "vt", 1));
		buzzvm_gload(VM);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "get", 1));
        buzzvm_tget(VM);*/
		buzzvm_pushs(VM, buzzvm_string_register(VM, "users", 1));
		buzzvm_gload(VM);
        //buzzvm_pushi(VM, 1);
		//buzzvm_callc(VM);
		buzzvm_type_assert(VM, 1, BUZZTYPE_TABLE);
		buzzobj_t nbr = buzzvm_stack_at(VM, 1);
		/* Get "data" field */
		buzzvm_pushs(VM, buzzvm_string_register(VM, "dataG", 1));
		buzzvm_tget(VM);
		if(buzzvm_stack_at(VM, 1)->o.type == BUZZTYPE_NIL) {
			//ROS_INFO("Empty data, create a new table");
			buzzvm_pop(VM);
			buzzvm_push(VM, nbr);
			buzzvm_pushs(VM, buzzvm_string_register(VM, "dataG", 1));
			buzzvm_pusht(VM);
			buzzobj_t data = buzzvm_stack_at(VM, 1);
			buzzvm_tput(VM);
			buzzvm_push(VM, data);
		}
		/* When we get here, the "data" table is on top of the stack */
		/* Push user id */
		buzzvm_pushi(VM, id);
		/* Create entry table */
		buzzobj_t entry = buzzheap_newobj(VM->heap, BUZZTYPE_TABLE);
		/* Insert latitude */
		buzzvm_push(VM, entry);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "la", 1));
		buzzvm_pushf(VM, latitude);
		buzzvm_tput(VM);
		/* Insert longitude */
		buzzvm_push(VM, entry);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "lo", 1));
		buzzvm_pushf(VM, longitude);
		buzzvm_tput(VM);
		/* Insert altitude */
		buzzvm_push(VM, entry);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "al", 1));
		buzzvm_pushf(VM, altitude);
		buzzvm_tput(VM);
		/* Save entry into data table */
		buzzvm_push(VM, entry);
		buzzvm_tput(VM);
		ROS_INFO("Buzz_utility saved new user: %i (%f,%f,%f)", id, latitude, longitude, altitude);
		// forcing the new table into the stigmergy....
		/*buzzobj_t newt = buzzvm_stack_at(VM, 0);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "vt", 1));
		buzzvm_gload(VM);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "put", 1));
        buzzvm_tget(VM);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "p", 1));
        buzzvm_push(VM, nbr);
        buzzvm_pushi(VM, 2);
		buzzvm_callc(VM);*/
		//buzzvm_gstore(VM);
		return VM->state;
	}
	/**************************************************************************/
	/*Deserializes uint64_t into 4 uint16_t, freeing out is left to the user  */
	/**************************************************************************/
	uint16_t* u64_cvt_u16(uint64_t u64){
		uint16_t* out = new uint16_t[4];
		uint32_t int32_1 = u64 & 0xFFFFFFFF;
		uint32_t int32_2 = (u64 & 0xFFFFFFFF00000000 ) >> 32;
		out[0] = int32_1 & 0xFFFF;
		out[1] = (int32_1 & (0xFFFF0000) ) >> 16;
		out[2] = int32_2 & 0xFFFF;
		out[3] = (int32_2 & (0xFFFF0000) ) >> 16;
		//cout << " values " <<out[0] <<"  "<<out[1] <<"  "<<out[2] <<"  "<<out[3] <<"  ";
		return out;
	}

	int get_robotid() {
          return Robot_id;
        }
	/***************************************************/
	/*Appends obtained messages to buzz in message Queue*/
	/***************************************************/

	/*******************************************************************************************************************/
	/* Message format of payload (Each slot is uint64_t)						                   */
	/* _______________________________________________________________________________________________________________ */
	/*|					        		             |			                  |*/
	/*|Size in Uint64_t(but size is Uint16_t)|robot_id|Update msg size|Update msg|Update msgs+Buzz_msgs with size.....|*/
	/*|__________________________________________________________________________|____________________________________|*/
	/*******************************************************************************************************************/

	void in_msg_append(uint64_t* payload){

   		/* Go through messages and append them to the vector */
   		uint16_t* data= u64_cvt_u16((uint64_t)payload[0]);
		/*Size is at first 2 bytes*/
   		uint16_t size=data[0]*sizeof(uint64_t);
   		delete[] data;
   		uint8_t* pl =(uint8_t*)malloc(size);
		/* Copy packet into temporary buffer */
	   	memcpy(pl, payload ,size);
   		IN_MSG.push_back(pl);
   		
	}
	
	void in_message_process(){
		while(!IN_MSG.empty()){
			uint8_t* first_INmsg = (uint8_t*)IN_MSG.front(); 
			/* Go through messages and append them to the FIFO */
   			uint16_t* data= u64_cvt_u16((uint64_t)first_INmsg[0]);
			/*Size is at first 2 bytes*/
   			uint16_t size=data[0]*sizeof(uint64_t);
   			delete[] data;
			/*size and robot id read*/
	   		size_t tot = sizeof(uint32_t);
			/* Go through the messages until there's nothing else to read */
	      		uint16_t unMsgSize=0;
				/*Obtain Buzz messages push it into queue*/
		      			do {
			 			/* Get payload size */
			 			unMsgSize = *(uint16_t*)(first_INmsg + tot);
		   	 			tot += sizeof(uint16_t);
			 			/* Append message to the Buzz input message queue */
			 			if(unMsgSize > 0 && unMsgSize <= size - tot ) {
			    			buzzinmsg_queue_append(VM,
				                buzzmsg_payload_frombuffer(first_INmsg +tot, unMsgSize));
			    			tot += unMsgSize;
			 			}
		      			}while(size - tot > sizeof(uint16_t) && unMsgSize > 0);
			IN_MSG.erase(IN_MSG.begin());
			free(first_INmsg);
		}
		/* Process messages VM call*/
		buzzvm_process_inmsgs(VM);
	}
	/***************************************************/
	/*Obtains messages from buzz out message Queue*/
	/***************************************************/

   	uint64_t* obt_out_msg(){
		/* Process out messages */
		buzzvm_process_outmsgs(VM); 
   		uint8_t* buff_send =(uint8_t*)malloc(MAX_MSG_SIZE);
   		memset(buff_send, 0, MAX_MSG_SIZE);
		/*Taking into consideration the sizes included at the end*/
   		ssize_t tot = sizeof(uint16_t);
   		/* Send robot id */
   		*(uint16_t*)(buff_send+tot) = (uint16_t) VM->robot;
   		tot += sizeof(uint16_t);
 			/* Send messages from FIFO */
	   		do {
				/* Are there more messages? */
	      			if(buzzoutmsg_queue_isempty(VM)) break;
	      			/* Get first message */
	      			buzzmsg_payload_t m = buzzoutmsg_queue_first(VM);
	      			/* Make sure the next message makes the data buffer with buzz messages to be less than 100 Bytes */
	      			if(tot + buzzmsg_payload_size(m) + sizeof(uint16_t)
					>
			 		MSG_SIZE) {
			 		buzzmsg_payload_destroy(&m);
			 		break;
	      			}

      			/* Add message length to data buffer */
      			*(uint16_t*)(buff_send + tot) = (uint16_t)buzzmsg_payload_size(m);
      			tot += sizeof(uint16_t);

      			/* Add payload to data buffer */
      			memcpy(buff_send + tot, m->data, buzzmsg_payload_size(m));
			tot += buzzmsg_payload_size(m);

      			/* Get rid of message */
      			buzzoutmsg_queue_next(VM);
      			buzzmsg_payload_destroy(&m);
	   		} while(1);

   		uint16_t total_size =(ceil((float)tot/(float)sizeof(uint64_t)));
   		*(uint16_t*)buff_send = (uint16_t) total_size;

   		uint64_t* payload_64 = new uint64_t[total_size];

   		memcpy((void*)payload_64, (void*)buff_send, total_size*sizeof(uint64_t));
		free(buff_send);
  		/*for(int i=0;i<total_size;i++){
   		cout<<" payload from out msg  "<<*(payload_64+i)<<endl;
   		}*/
   		/* Send message */
	return payload_64;
	}

	/****************************************/
	/*Buzz script not able to load*/
	/****************************************/

	static const char* buzz_error_info() {
		buzzdebug_entry_t dbg = *buzzdebug_info_get_fromoffset(DBG_INFO, &VM->pc);
   		char* msg;
   		if(dbg != NULL) {
      			asprintf(&msg,
               		"%s: execution terminated abnormally at %s:%" PRIu64 ":%" PRIu64 " : %s\n\n",
               		BO_FNAME,
               		dbg->fname,
               		dbg->line,
               		dbg->col,
               		VM->errormsg);
   		}
   		else {
      			asprintf(&msg,
               			"%s: execution terminated abnormally at bytecode offset %d: %s\n\n",
               			 BO_FNAME,
              			 VM->pc,
      			         VM->errormsg);
   		}
 	return msg;
	}

	/****************************************/
	/*Buzz hooks that can be used inside .bzz file*/
	/****************************************/

	static int buzz_register_hooks() {
		buzzvm_pushs(VM,  buzzvm_string_register(VM, "print", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzros_print));
   		buzzvm_gstore(VM);
   		buzzvm_pushs(VM,  buzzvm_string_register(VM, "log", 1));
        buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzros_print));
        buzzvm_gstore(VM);
        buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_moveto", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_moveto));
   		buzzvm_gstore(VM);
        buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_goto", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_goto));
   		buzzvm_gstore(VM);
        buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_arm", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_arm));
   		buzzvm_gstore(VM);
		buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_disarm", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_disarm));
   		buzzvm_gstore(VM);
   		buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_takeoff", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_takeoff));
   		buzzvm_gstore(VM);
   		buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_gohome", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_gohome));
   		buzzvm_gstore(VM);
   		buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_land", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_land));
   		buzzvm_gstore(VM);
   		buzzvm_pushs(VM,  buzzvm_string_register(VM, "add_user_rb", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_adduserRB));
   		buzzvm_gstore(VM);

   	return VM->state;
	}

	/**************************************************/
	/*Register dummy Buzz hooks for test during update*/
	/**************************************************/

	static int testing_buzz_register_hooks() {
		buzzvm_pushs(VM,  buzzvm_string_register(VM, "print", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzros_print));
   		buzzvm_gstore(VM);
   		buzzvm_pushs(VM,  buzzvm_string_register(VM, "log", 1));
        buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzros_print));
        buzzvm_gstore(VM);
        buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_moveto", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
   		buzzvm_gstore(VM);
        buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_goto", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
   		buzzvm_gstore(VM);
        buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_arm", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
   		buzzvm_gstore(VM);
		buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_disarm", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
   		buzzvm_gstore(VM); 
   		buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_takeoff", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
   		buzzvm_gstore(VM);
   		buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_gohome", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
   		buzzvm_gstore(VM);
   		buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_land", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
   		buzzvm_gstore(VM);
   		buzzvm_pushs(VM,  buzzvm_string_register(VM, "add_user_rb", 1));
   		buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
   		buzzvm_gstore(VM);

   	return VM->state;
	}

static int create_stig_tables() {
/*
   		// usersvstig = stigmergy.create(123)
        buzzvm_pushs(VM, buzzvm_string_register(VM, "vt", 1));
        // get the stigmergy table from the global scope
        buzzvm_pushs(VM, buzzvm_string_register(VM, "stigmergy", 1));
        buzzvm_gload(VM);
        // get the create method from the stigmergy table
        buzzvm_pushs(VM, buzzvm_string_register(VM, "create", 1));
        buzzvm_tget(VM);
        // value of the stigmergy id
        buzzvm_pushi(VM, 5);
        // call the stigmergy.create() method
		buzzvm_dump(VM);
//        buzzvm_closure_call(VM, 1);
		buzzvm_pushi(VM, 1);
		buzzvm_callc(VM);
        buzzvm_gstore(VM);
		buzzvm_dump(VM);

		//buzzusers_reset();
		buzzobj_t t = buzzheap_newobj(VM->heap, BUZZTYPE_TABLE);

		buzzvm_pushs(VM, buzzvm_string_register(VM, "vt", 1));
		buzzvm_gload(VM);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "put", 1));
        buzzvm_tget(VM);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "p", 1));
		buzzvm_push(VM, t);
		buzzvm_dump(VM);
//        buzzvm_closure_call(VM, 2);
        buzzvm_pushi(VM, 2);
		buzzvm_callc(VM);
        //buzzvm_gstore(VM);
		//buzzvm_dump(VM);

		/*buzzvm_pushs(VM, buzzvm_string_register(VM, "vt", 1));
        buzzvm_gload(VM);
        buzzvm_pushs(VM, buzzvm_string_register(VM, "put", 1));
        buzzvm_tget(VM);
        buzzvm_pushs(VM, buzzvm_string_register(VM, "u", 1));
        buzzvm_pusht(VM);
        buzzvm_pushi(VM, 2);
		buzzvm_call(VM, 0);
        buzzvm_gstore(VM);*/

		buzzobj_t t = buzzheap_newobj(VM->heap, BUZZTYPE_TABLE);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "users", 1));
		buzzvm_push(VM,t);
		buzzvm_gstore(VM);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "users", 1));
		buzzvm_gload(VM);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "dataG", 1));
		buzzvm_pusht(VM);
		buzzobj_t data = buzzvm_stack_at(VM, 1);
		buzzvm_tput(VM);
		buzzvm_push(VM, data);
		
		buzzvm_pushs(VM, buzzvm_string_register(VM, "users", 1));
		buzzvm_gload(VM);
		buzzvm_pushs(VM, buzzvm_string_register(VM, "dataL", 1));
		buzzvm_pusht(VM);
		data = buzzvm_stack_at(VM, 1);
		buzzvm_tput(VM);
		buzzvm_push(VM, data);

   	return VM->state;
}
	/****************************************/
	/*Sets the .bzz and .bdbg file*/
	/****************************************/
	int buzz_script_set(const char* bo_filename,
	                    const char* bdbg_filename, int robot_id) {
	   	ROS_INFO(" Robot ID: " , robot_id);
	   	/* Reset the Buzz VM */
	   	if(VM) buzzvm_destroy(&VM);
		Robot_id = robot_id;
	   	VM = buzzvm_new((int)robot_id);
	   	/* Get rid of debug info */
	   	if(DBG_INFO) buzzdebug_destroy(&DBG_INFO);
	   	DBG_INFO = buzzdebug_new();
	   	/* Read bytecode and fill in data structure */
	   	FILE* fd = fopen(bo_filename, "rb");
	   	if(!fd) {
	      		perror(bo_filename);
      		return 0;
	   	}
	   	fseek(fd, 0, SEEK_END);
	   	size_t bcode_size = ftell(fd);
	   	rewind(fd);
	   	BO_BUF = (uint8_t*)malloc(bcode_size);
	   	if(fread(BO_BUF, 1, bcode_size, fd) < bcode_size) {
	      		perror(bo_filename);
	      		buzzvm_destroy(&VM);
	      		buzzdebug_destroy(&DBG_INFO);
	      		fclose(fd);
      		return 0;
	   	}
	   	fclose(fd);
	   	/* Read debug information */
	   	if(!buzzdebug_fromfile(DBG_INFO, bdbg_filename)) {
	      		buzzvm_destroy(&VM);
	      		buzzdebug_destroy(&DBG_INFO);
	      		perror(bdbg_filename);
      		return 0;
   	}
   	/* Set byte code */
   	if(buzzvm_set_bcode(VM, BO_BUF, bcode_size) != BUZZVM_STATE_READY) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		ROS_ERROR("%s: Error loading Buzz script", bo_filename);
      		return 0;
   	}
   	/* Register hook functions */
   	if(buzz_register_hooks() != BUZZVM_STATE_READY) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		ROS_ERROR("[%i] Error registering hooks", Robot_id);
      		return 0;
   	}
   	/* Create vstig tables */
	if(create_stig_tables() != BUZZVM_STATE_READY) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		ROS_ERROR("[%i] Error creating stigmergy tables", Robot_id);
			//cout << "ERROR!!!!   ----------  " << VM->errormsg << endl;
			//cout << "ERROR!!!!   ----------  " << buzzvm_strerror(VM) << endl;
      		return 0;
   	}
        
   	/* Save bytecode file name */
   	BO_FNAME = strdup(bo_filename);
   	/* Execute the global part of the script */
   	buzzvm_execute_script(VM);
   	/* Call the Init() function */
   	buzzvm_function_call(VM, "init", 0);
   	/* All OK */

    ROS_INFO("[%i] INIT DONE!!!", Robot_id);

   	return 1;//buzz_update_set(BO_BUF, bdbg_filename, bcode_size);
	}
        
	/****************************************/
	/*Sets a new update                     */
	/****************************************/
	int buzz_update_set(uint8_t* UP_BO_BUF, const char* bdbg_filename,size_t bcode_size){
	// Reset the Buzz VM
   	if(VM) buzzvm_destroy(&VM);
   	VM = buzzvm_new(Robot_id);
   	// Get rid of debug info
   	if(DBG_INFO) buzzdebug_destroy(&DBG_INFO);
  	DBG_INFO = buzzdebug_new();

   	// Read debug information
   	if(!buzzdebug_fromfile(DBG_INFO, bdbg_filename)) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		perror(bdbg_filename);
      		return 0;
   	 }
   	// Set byte code
   	if(buzzvm_set_bcode(VM, UP_BO_BUF, bcode_size) != BUZZVM_STATE_READY) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		fprintf(stdout, "%s: Error loading Buzz script\n\n", BO_FNAME);
      		return 0;
   	 }
   	// Register hook functions
   	if(buzz_register_hooks() != BUZZVM_STATE_READY) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		fprintf(stdout, "%s: Error registering hooks\n\n", BO_FNAME);
        	return 0;
   	}
   	/* Create vstig tables */
	if(create_stig_tables() != BUZZVM_STATE_READY) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		ROS_ERROR("[%i] Error creating stigmergy tables", Robot_id);
			//cout << "ERROR!!!!   ----------  " << VM->errormsg << endl;
			//cout << "ERROR!!!!   ----------  " << buzzvm_strerror(VM) << endl;
      		return 0;
   	}
   	// Execute the global part of the script
   	buzzvm_execute_script(VM);
   	// Call the Init() function
   	buzzvm_function_call(VM, "init", 0);
   	// All OK
   	return 1;
	}

	/****************************************/
	/*Performs a initialization test        */
	/****************************************/
	int buzz_update_init_test(uint8_t* UP_BO_BUF, const char* bdbg_filename,size_t bcode_size){
	// Reset the Buzz VM
   	if(VM) buzzvm_destroy(&VM);
   	VM = buzzvm_new(Robot_id);
   	// Get rid of debug info
   	if(DBG_INFO) buzzdebug_destroy(&DBG_INFO);
  	DBG_INFO = buzzdebug_new();

   	// Read debug information
   	if(!buzzdebug_fromfile(DBG_INFO, bdbg_filename)) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		perror(bdbg_filename);
      		return 0;
   	 }
   	// Set byte code
   	if(buzzvm_set_bcode(VM, UP_BO_BUF, bcode_size) != BUZZVM_STATE_READY) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		fprintf(stdout, "%s: Error loading Buzz script\n\n", BO_FNAME);
      		return 0;
   	 }
   	// Register hook functions
   	if(testing_buzz_register_hooks() != BUZZVM_STATE_READY) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		fprintf(stdout, "%s: Error registering hooks\n\n", BO_FNAME);
        	return 0;
   	}
   	/* Create vstig tables */
	if(create_stig_tables() != BUZZVM_STATE_READY) {
      		buzzvm_destroy(&VM);
      		buzzdebug_destroy(&DBG_INFO);
      		ROS_ERROR("[%i] Error creating stigmergy tables", Robot_id);
			//cout << "ERROR!!!!   ----------  " << VM->errormsg << endl;
			//cout << "ERROR!!!!   ----------  " << buzzvm_strerror(VM) << endl;
      		return 0;
   	}
   	// Execute the global part of the script
   	buzzvm_execute_script(VM);
   	// Call the Init() function
   	buzzvm_function_call(VM, "init", 0);
   	// All OK
   	return 1;
	}

	/****************************************/
	/*Swarm struct*/
	/****************************************/

	struct buzzswarm_elem_s {
	   buzzdarray_t swarms;
	   uint16_t age;
	};
	typedef struct buzzswarm_elem_s* buzzswarm_elem_t;

	void check_swarm_members(const void* key, void* data, void* params) {
		buzzswarm_elem_t e = *(buzzswarm_elem_t*)data;
		int* status = (int*)params;
		if(*status == 3) return;
		fprintf(stderr, "CHECKING SWARM :%i, member: %i, age: %i \n",
			buzzdarray_get(e->swarms, 0, uint16_t), *(uint16_t*)key, e->age);
		if(buzzdarray_size(e->swarms) != 1) {
			fprintf(stderr, "Swarm list size is not 1\n");
			*status = 3;
		}
		else {
			int sid = 1;
			if(!buzzdict_isempty(VM->swarms)) {
				if(*buzzdict_get(VM->swarms, &sid, uint8_t) &&
				    buzzdarray_get(e->swarms, 0, uint16_t) != sid) {
				    fprintf(stderr, "I am in swarm #%d and neighbor is in %d\n",
					    sid,
					    buzzdarray_get(e->swarms, 0, uint16_t));
				    *status = 3;
				return;
				}
			}
			if(buzzdict_size(VM->swarms)>1) {
				sid = 2;
				if(*buzzdict_get(VM->swarms, &sid, uint8_t) &&
					buzzdarray_get(e->swarms, 0, uint16_t) != sid) {
					fprintf(stderr, "I am in swarm #%d and neighbor is in %d\n",
					    sid,
					    buzzdarray_get(e->swarms, 0, uint16_t));
					*status = 3;
				return;
				}
			}
		}
	}

	/*Step through the buzz script*/
   	void update_sensors(){
		/* Update sensors*/
		buzzuav_closures::buzzuav_update_battery(VM);
	   	buzzuav_closures::buzzuav_update_prox(VM);
	   	buzzuav_closures::buzzuav_update_currentpos(VM);
	   	buzzuav_closures::update_neighbors(VM);
	   	update_users();
	   	buzzuav_closures::buzzuav_update_flight_status(VM);
	}

	void buzz_script_step() {
		/*Process available messages*/
		in_message_process();
		/*Update sensors*/
		update_sensors();
		/* Call Buzz step() function */
		if(buzzvm_function_call(VM, "step", 0) != BUZZVM_STATE_READY) {
		fprintf(stderr, "%s: execution terminated abnormally: %s\n\n",
		      BO_FNAME,
		      buzz_error_info());
		buzzvm_dump(VM);
		}
		
		/*Print swarm*/
		//buzzswarm_members_print(stdout, VM->swarmmembers, VM->robot);
		//int SwarmSize = buzzdict_size(VM->swarmmembers)+1;
		//fprintf(stderr, "Real Swarm Size: %i\n",SwarmSize);
		set_robot_var(buzzdict_size(VM->swarmmembers)+1);

		/* Check swarm state -- Not crashing thanks to test added in check_swarm_members */
		//int status = 1;
		//buzzdict_foreach(VM->swarmmembers, check_swarm_members, &status);
	}

	/****************************************/
	/*Destroy the bvm and other resorces*/
	/****************************************/

	void buzz_script_destroy() {
	   if(VM) {
		  if(VM->state != BUZZVM_STATE_READY) {
			 fprintf(stderr, "%s: execution terminated abnormally: %s\n\n",
					 BO_FNAME,
					 buzz_error_info());
			 buzzvm_dump(VM);
		  }
		  buzzvm_function_call(VM, "destroy", 0);
		  buzzvm_destroy(&VM);
		  free(BO_FNAME);
		  buzzdebug_destroy(&DBG_INFO);
	   }
	   fprintf(stdout, "Script execution stopped.\n");
	}


	/****************************************/
	/****************************************/

	/****************************************/
	/*Execution completed*/
	/****************************************/

	int buzz_script_done() {
		return VM->state != BUZZVM_STATE_READY;
	}

	int update_step_test() {

		buzzuav_closures::buzzuav_update_battery(VM);
		buzzuav_closures::buzzuav_update_prox(VM);
		buzzuav_closures::buzzuav_update_currentpos(VM);
	   	buzzuav_closures::update_neighbors(VM);
	   	update_users();
		buzzuav_closures::buzzuav_update_flight_status(VM);

		int a = buzzvm_function_call(VM, "step", 0);
		if(a != BUZZVM_STATE_READY) {
			fprintf(stdout, "step test VM state %i\n",a);
			fprintf(stdout, " execution terminated abnormally\n\n");
		}
		return a == BUZZVM_STATE_READY;
	}

	buzzvm_t get_vm() {
		return VM;
	}

	void set_robot_var(int ROBOTS){
		buzzvm_pushs(VM, buzzvm_string_register(VM, "ROBOTS", 1));
		buzzvm_pushi(VM, ROBOTS);
		buzzvm_gstore(VM);
	}
}



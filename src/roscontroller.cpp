#include "roscontroller.h"

namespace rosbzz_node{

	/*---------------
	/Constructor
	---------------*/
	roscontroller::roscontroller(ros::NodeHandle n_c)	
	{
		ROS_INFO("Buzz_node");
		/*Obtain parameters from ros parameter server*/
	  	Rosparameters_get(n_c);
		/*Initialize publishers, subscribers and client*/
  		Initialize_pub_sub(n_c);
		/*Compile the .bzz file to .basm, .bo and .bdbg*/
 		Compile_bzz();
		set_bzz_file(bzzfile_name.c_str());
		/*Initialize variables*/
		armstate = 0;
		multi_msg = true;
	}

	/*---------------------
	/Destructor
	---------------------*/
	roscontroller::~roscontroller()
	{
		/* All done */  
		/* Cleanup */
		buzz_utility::buzz_script_destroy();
 		/* Stop the robot */
   		uav_done();
	}

	/*-------------------------------------------------
	/rosbuzz_node loop method executed once every step
	/--------------------------------------------------*/
	void roscontroller::RosControllerRun(){
		mavros_msgs::ParamGet::Request robot_id_srv_request; robot_id_srv_request.param_id="id";
        	mavros_msgs::ParamGet::Response robot_id_srv_response;
		while(!xbeestatus_srv.call(robot_id_srv_request,robot_id_srv_response)){
			/*run once*/
    			ros::spinOnce();
			/*loop rate of ros*/
			ros::Rate loop_rate(10);
			loop_rate.sleep();
 			/*sleep for the mentioned loop rate*/
			ROS_ERROR("Waiting for Xbee to respond to get device ID");
		}
		robot_id=robot_id_srv_response.value.integer;
			
		/* Set the Buzz bytecode */
		if(buzz_utility::buzz_script_set(bcfname.c_str(), dbgfname.c_str(),robot_id)) {
			fprintf(stdout, "Bytecode file found and set\n");
			init_update_monitor(bcfname.c_str(),stand_by.c_str());
			while (ros::ok() && !buzz_utility::buzz_script_done())
  			{
      				/*Update neighbors position inside Buzz*/
     				buzz_utility::neighbour_pos_callback(neighbours_pos_map);
				/*Neighbours of the robot published with id in respective topic*/
				neighbours_pos_publisher();
				/*Check updater state and step code*/
  				update_routine(bcfname.c_str(), dbgfname.c_str());
				/*Step buzz script */
      				buzz_utility::buzz_script_step();
				/*Prepare messages and publish them in respective topic*/
		  		prepare_msg_and_publish();
				/*call flight controler service to set command long*/
				flight_controler_service_call();
				/*Set multi message available after update*/
				if(get_update_status()){
					set_read_update_status();
					multi_msg=true;
				}
				/*Set ROBOTS variable for barrier in .bzz from neighbours count*/
				buzz_utility::set_robot_var(neighbours_pos_map.size()+1);
				/*Set no of robots for updates*/
				updates_set_robots(neighbours_pos_map.size()+1);
    				/*run once*/
    				ros::spinOnce();
				/*loop rate of ros*/
				 ros::Rate loop_rate(10);
				 loop_rate.sleep();
 				/*sleep for the mentioned loop rate*/
    				timer_step+=1;
   				maintain_pos(timer_step);
				
				
			}
			/* Destroy updater and Cleanup */
    			//update_routine(bcfname.c_str(), dbgfname.c_str(),1);
  		}
	}

	/*--------------------------------------------------------
	/ Get all required parameters from the ROS launch file
	/-------------------------------------------------------*/
	void roscontroller::Rosparameters_get(ros::NodeHandle n_c){
		
		/*Obtain .bzz file name from parameter server*/
	  	if(n_c.getParam("bzzfile_name", bzzfile_name));
	  	else {ROS_ERROR("Provide a .bzz file to run in Launch file"); system("rosnode kill rosbuzz_node");}  
	  	/*Obtain rc service option from parameter server*/
	  	if(n_c.getParam("rcclient", rcclient)){
		     	if(rcclient==true){
			    	/*Service*/
			    	if(n_c.getParam("rcservice_name", rcservice_name)){
				        service = n_c.advertiseService(rcservice_name, &roscontroller::rc_callback,this);
				    	ROS_INFO("Ready to receive Mav Commands from RC client");
			        }
			        else{ROS_ERROR("Provide a name topic name for rc service in Launch file"); system("rosnode kill rosbuzz_node");}
		        }
     			else if(rcclient==false){ROS_INFO("RC service is disabled");}
  		}
  		else{ROS_ERROR("Provide a rc client option: yes or no in Launch file"); system("rosnode kill rosbuzz_node");} 
  		/*Obtain robot_id from parameter server*/
  		//n_c.getParam("robot_id", robot_id);
		//robot_id=(int)buzz_utility::get_robotid();
		/*Obtain out payload name*/
  		n_c.getParam("out_payload", out_payload);
		/*Obtain in payload name*/
  		n_c.getParam("in_payload", in_payload);
		/*Obtain Number of robots for barrier*/
		n_c.getParam("No_of_Robots", barrier);
		/*Obtain standby script to run during update*/
		n_c.getParam("stand_by", stand_by);
		n_c.getParam("xbee_status_srv", xbeesrv_name);
		
		GetSubscriptionParameters(n_c);
		// initialize topics to null?

	}
	/*-----------------------------------------------------------------------------------
	/Obtains publisher, subscriber and services from yml file based on the robot used
	/-----------------------------------------------------------------------------------*/
	void roscontroller::GetSubscriptionParameters(ros::NodeHandle node_handle){
		//todo: make it as an array in yaml?
		m_sMySubscriptions.clear();
		std::string gps_topic, gps_type;
		node_handle.getParam("topics/gps", gps_topic);
		node_handle.getParam("type/gps", gps_type);
		m_smTopic_infos.insert(pair <std::string, std::string>(gps_topic, gps_type));

		std::string battery_topic, battery_type;
		node_handle.getParam("topics/battery", battery_topic);
		node_handle.getParam("type/battery", battery_type);
		m_smTopic_infos.insert(pair <std::string, std::string>(battery_topic, battery_type));


		std::string status_topic, status_type;
		node_handle.getParam("topics/status", status_topic);
		node_handle.getParam("type/status", status_type);
		m_smTopic_infos.insert(pair <std::string, std::string>(status_topic, status_type));

  		/*Obtain fc client name from parameter server*/
  		if(node_handle.getParam("topics/fcclient", fcclient_name));
  		else {ROS_ERROR("Provide a fc client name in Launch file"); system("rosnode kill rosbuzz_node");}  
  		if(node_handle.getParam("topics/armclient", armclient));
  		else {ROS_ERROR("Provide an arm client name in Launch file"); system("rosnode kill rosbuzz_node");}  
  		if(node_handle.getParam("topics/modeclient", modeclient));
  		else {ROS_ERROR("Provide a mode client name in Launch file"); system("rosnode kill rosbuzz_node");}  
	}

	/*--------------------------------------------------------
	/ Create all required subscribers, publishers and ROS service clients
	/-------------------------------------------------------*/
	void roscontroller::Initialize_pub_sub(ros::NodeHandle n_c){
		/*subscribers*/

		Subscribe(n_c);

  		//current_position_sub = n_c.subscribe("/global_position", 1000, &roscontroller::current_pos,this);
  		//battery_sub = n_c.subscribe("/power_status", 1000, &roscontroller::battery,this);
  		payload_sub = n_c.subscribe(in_payload, 1000, &roscontroller::payload_obt,this);
		//flight_status_sub =n_c.subscribe("/flight_status",100, &roscontroller::flight_extended_status_update,this);
		//Robot_id_sub = n_c.subscribe("/device_id_xbee_", 1000, &roscontroller::set_robot_id,this);
  		obstacle_sub= n_c.subscribe("/guidance/obstacle_distance",100, &roscontroller::obstacle_dist,this);
  		/*publishers*/
		payload_pub = n_c.advertise<mavros_msgs::Mavlink>(out_payload, 1000);
		neigh_pos_pub = n_c.advertise<rosbuzz::neigh_pos>("/neighbours_pos",1000);
		localsetpoint_pub = n_c.advertise<mavros_msgs::PositionTarget>("/setpoint_raw/local",1000);	
		/* Service Clients*/
		arm_client = n_c.serviceClient<mavros_msgs::CommandBool>(armclient);
		mode_client =  n_c.serviceClient<mavros_msgs::SetMode>(modeclient);
		mav_client = n_c.serviceClient<mavros_msgs::CommandLong>(fcclient_name);
		xbeestatus_srv = n_c.serviceClient<mavros_msgs::ParamGet>(xbeesrv_name);
		multi_msg=true;
	}
	/*---------------------------------------
	/Robot independent subscribers
	/--------------------------------------*/
	void roscontroller::Subscribe(ros::NodeHandle n_c){

  		for(std::map<std::string, std::string>::iterator it = m_smTopic_infos.begin(); it != m_smTopic_infos.end(); ++it){
  			if(it->second == "mavros_msgs/ExtendedState"){
  				flight_status_sub = n_c.subscribe(it->first, 100, &roscontroller::flight_extended_status_update, this);
  			}
  			else if(it->second == "mavros_msgs/State"){
  				flight_status_sub = n_c.subscribe(it->first, 100, &roscontroller::flight_status_update, this);
  			}
  			else if(it->second == "mavros_msgs/BatteryStatus"){
  		  		battery_sub = n_c.subscribe(it->first, 1000, &roscontroller::battery, this);
  			}
  			else if(it->second == "sensor_msgs/NavSatFix"){
  		  		current_position_sub = n_c.subscribe(it->first, 1000, &roscontroller::current_pos, this);
 			}

	  		std::cout << "Subscribed to: " << it->first << endl;
  		}
	}

	/*--------------------------------------------------------
	/ Create Buzz bytecode from the bzz script inputed
	/-------------------------------------------------------*/
	void roscontroller::Compile_bzz(){
		/*TODO: change to bzzc instead of bzzparse and also add -I for includes*/
		/*Compile the buzz code .bzz to .bo*/
		stringstream bzzfile_in_compile;
	        std::string  path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/"));
		bzzfile_in_compile<<path<<"/";
		path = bzzfile_in_compile.str();
		bzzfile_in_compile.str("");
		std::string  name = bzzfile_name.substr(bzzfile_name.find_last_of("/\\") + 1);
 		name = name.substr(0,name.find_last_of("."));
		bzzfile_in_compile << "bzzparse "<<bzzfile_name<<" "<<path<< name<<".basm";
   		system(bzzfile_in_compile.str().c_str());
		bzzfile_in_compile.str("");
           	bzzfile_in_compile <<"bzzasm "<<path<<name<<".basm "<<path<<name<<".bo "<<path<<name<<".bdbg";
   		system(bzzfile_in_compile.str().c_str());
		bzzfile_in_compile.str("");
		bzzfile_in_compile <<path<<name<<".bo";
		bcfname = bzzfile_in_compile.str();
		bzzfile_in_compile.str("");
		bzzfile_in_compile <<path<<name<<".bdbg";
		dbgfname = bzzfile_in_compile.str();
   		
	}
	/*----------------------------------------------------
	/ Publish neighbours pos and id in neighbours pos topic
	/----------------------------------------------------*/
	void roscontroller::neighbours_pos_publisher(){
		auto current_time = ros::Time::now();
		map< int, buzz_utility::Pos_struct >::iterator it;
		rosbuzz::neigh_pos neigh_pos_array; //neigh_pos_array.clear(); 
		neigh_pos_array.header.frame_id = "/world";
		neigh_pos_array.header.stamp = current_time;
		for (it=raw_neighbours_pos_map.begin(); it!=raw_neighbours_pos_map.end(); ++it){
			sensor_msgs::NavSatFix neigh_tmp;
			//cout<<"iterator it val: "<< it-> first << " After convertion: " <<(uint8_t) buzz_utility::get_rid_uint8compac(it->first)<<endl;				
			neigh_tmp.header.stamp = current_time;
			neigh_tmp.header.frame_id = "/world";
			neigh_tmp.position_covariance_type=it->first; //custom robot id storage
        		neigh_tmp.latitude=(it->second).x;
        		neigh_tmp.longitude=(it->second).y;
        		neigh_tmp.altitude=(it->second).z;
			neigh_pos_array.pos_neigh.push_back(neigh_tmp); 
			//std::cout<<"long obt"<<neigh_tmp.longitude<<endl;  					
			}
		neigh_pos_pub.publish(neigh_pos_array); 	
	}

	/*--------------------------------------------------------
	/ Functions handling the local MAV ROS flight controller
	/-------------------------------------------------------*/
	void roscontroller::Arm(){
		mavros_msgs::CommandBool arming_message;
		arming_message.request.value = armstate;
		if(arm_client.call(arming_message)) {
			if(arming_message.response.success==1)
				ROS_INFO("FC Arm Service called!");
			else
				ROS_INFO("FC Arm Service call failed!");
		} else {
			ROS_INFO("FC Arm Service call failed!");
		}
	}
	/*---------------------------------------------------------
	/ Set mode for the solos
	/---------------------------------------------------------*/
	void roscontroller::SetMode(){
		mavros_msgs::SetMode set_mode_message;
		set_mode_message.request.base_mode = 0;
		set_mode_message.request.custom_mode = "GUIDED"; // shit!
		if(mode_client.call(set_mode_message)) {
			ROS_INFO("Set Mode Service call successful!");
		} else {
			ROS_INFO("Set Mode Service call failed!");
		}
	}


	/*-----------------------------------------------------------------------------------------------------
	/Prepare Buzz messages payload for each step and publish
	/-----------------------------------------------------------------------------------------------------*/
	/*----------------------------------------------------------------------------------------------------*/
	/* Message format of payload (Each slot is uint64_t)						       /
	/ _________________________________________________________________________________________________    /
	/|	|     |	    |						     |			           |   /
	/|Pos x|Pos y|Pos z|Size in Uint64_t|robot_id|Buzz_msg_size|Buzz_msg|Buzz_msgs with size.........  |   /
	/|_____|_____|_____|________________________________________________|______________________________|  */
	/*----------------------------------------------------------------------------------------------------*/	
	void roscontroller::prepare_msg_and_publish(){
    		/*obtain Pay load to be sent*/  
   		uint64_t* payload_out_ptr= buzz_utility::out_msg_process();
    		uint64_t  position[3];
  		/*Appened current position to message*/
    		memcpy(position, cur_pos, 3*sizeof(uint64_t));
    		mavros_msgs::Mavlink payload_out;
    		payload_out.payload64.push_back(position[0]);
    		payload_out.payload64.push_back(position[1]);
    		payload_out.payload64.push_back(position[2]);
    		/*Append Buzz message*/
    		uint16_t* out = buzz_utility::u64_cvt_u16(payload_out_ptr[0]);  
    		for(int i=0;i<out[0];i++){
    			payload_out.payload64.push_back(payload_out_ptr[i]);
    		}	
		/*Add Robot id and message number to the published message*/
		if (message_number < 0) message_number=0;
		else message_number++;
		payload_out.sysid=(uint8_t)robot_id;
		payload_out.msgid=(uint32_t)message_number;
		
     		/*publish prepared messages in respective topic*/
    		payload_pub.publish(payload_out);
    		delete[] out;
    		delete[] payload_out_ptr;
		/*Check for updater message if present send*/
		if((int)get_update_mode()!=CODE_RUNNING && is_msg_present()==1 && multi_msg){
			uint8_t* buff_send = 0;
	   		uint16_t updater_msgSize=*(uint16_t*) (getupdate_out_msg_size());;
			int tot=0;
			mavros_msgs::Mavlink update_packets;
			fprintf(stdout,"Transfering code \n");
			fprintf(stdout,"Sent Update packet Size: %u \n",updater_msgSize);
			/*allocate mem and clear it*/
			buff_send =(uint8_t*)malloc(sizeof(uint16_t)+updater_msgSize);
			memset(buff_send, 0,sizeof(uint16_t)+updater_msgSize);
		   	/*Append updater msg size*/
		  	 *(uint16_t*)(buff_send + tot)=updater_msgSize;
		   	//fprintf(stdout,"Updater sent msg size : %i \n", (int)updater_msgSize);
      		   	tot += sizeof(uint16_t);
		   	/*Append updater msgs*/   	
    		   	memcpy(buff_send + tot, (uint8_t*)(getupdater_out_msg()), updater_msgSize);
		   	tot += updater_msgSize;
		   	/*Destroy the updater out msg queue*/
		    	destroy_out_msg_queue();
		    	uint16_t total_size =(ceil((float)(float)tot/(float)sizeof(uint64_t))); 
		    	uint64_t* payload_64 = new uint64_t[total_size];
	  	    	memcpy((void*)payload_64, (void*)buff_send, total_size*sizeof(uint64_t));
		    	delete[] buff_send;
		    	/*Send a constant number to differenciate updater msgs*/
		    	update_packets.payload64.push_back((uint64_t)UPDATER_MESSAGE_CONSTANT);
		    	for(int i=0;i<total_size;i++){
				update_packets.payload64.push_back(payload_64[i]);
		    	}
		    	/*Add Robot id and message number to the published message*/
		    	if (message_number < 0) message_number=0;
		    	else message_number++;
		    	update_packets.sysid=(uint8_t)robot_id;
		    	update_packets.msgid=(uint32_t)message_number;
		    	payload_pub.publish(update_packets);
		    	multi_msg=false;
		    	delete[] payload_64;
		}
		
	}
	/*---------------------------------------------------------------------------------
	/Flight controller service call every step if there is a command set from bzz script
	/-------------------------------------------------------------------------------- */
	void roscontroller::flight_controler_service_call(){
		/* flight controller client call if requested from Buzz*/
		/*FC call for takeoff,land, gohome and arm/disarm*/
		int tmp = buzzuav_closures::bzz_cmd();
    		double* goto_pos = buzzuav_closures::getgoto();
		if (tmp == buzzuav_closures::COMMAND_TAKEOFF || tmp== buzzuav_closures::COMMAND_LAND || tmp==buzzuav_closures::COMMAND_GOHOME){
			cmd_srv.request.param7 = goto_pos[2];
			//cmd_srv.request.z = goto_pos[2];
			cmd_srv.request.command =  buzzuav_closures::getcmd();  		
			if (mav_client.call(cmd_srv)){ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success); }
	    		else ROS_ERROR("Failed to call service from flight controller"); 
		} else if (tmp == buzzuav_closures::COMMAND_GOTO) { /*FC call for goto*/ 
			/*prepare the goto publish message */
			cmd_srv.request.param5 = goto_pos[0];
    			cmd_srv.request.param6 = goto_pos[1];
    			cmd_srv.request.param7 = goto_pos[2];
    			cmd_srv.request.command =  buzzuav_closures::getcmd();
			if (mav_client.call(cmd_srv)){ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success); }
	    		else ROS_ERROR("Failed to call service from flight controller"); 
    			cmd_srv.request.command = mavros_msgs::CommandCode::CMD_MISSION_START;
			if (mav_client.call(cmd_srv)){ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success); }
	    		else ROS_ERROR("Failed to call service from flight controller"); 
		} else if (tmp == buzzuav_closures::COMMAND_ARM) { /*FC call for arm*/
			armstate=1;
			Arm(); 
		} else if (tmp == buzzuav_closures::COMMAND_DISARM){
			armstate=0;
			Arm();
		} else if (tmp == buzzuav_closures::COMMAND_MOVETO) { /*Buzz call for moveto*/ 
			/*prepare the goto publish message */
			mavros_msgs::PositionTarget pt;
			pt.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
			pt.coordinate_frame =  mavros_msgs::PositionTarget::FRAME_LOCAL_OFFSET_NED;
			pt.position.x = goto_pos[0];
    			pt.position.y = goto_pos[1];
    			pt.position.z = goto_pos[2];
    			pt.yaw = 0;//goto_pos[3];
			ROS_INFO("Sending local setpoint: %.2f, %.2f, %.2f",pt.position.x,pt.position.y,pt.position.z);
			localsetpoint_pub.publish(pt);
 		}
	}
	/*----------------------------------------------
	/Refresh neighbours Position for every ten step
	/---------------------------------------------*/
	void roscontroller::maintain_pos(int tim_step){
		if(timer_step >=10){
		neighbours_pos_map.clear();
		//raw_neighbours_pos_map.clear(); // TODO: currently not a problem, but have to clear !
		timer_step=0;
		}
	}

	/*---------------------------------------------------------------------------------
	/Puts neighbours position into local struct storage that is cleared every 10 step
	/---------------------------------------------------------------------------------*/
	void roscontroller::neighbours_pos_put(int id, buzz_utility::Pos_struct pos_arr ){
		map< int, buzz_utility::Pos_struct >::iterator it = neighbours_pos_map.find(id);
		if(it!=neighbours_pos_map.end())
		neighbours_pos_map.erase(it);
		neighbours_pos_map.insert(make_pair(id, pos_arr));
		}
	/*-----------------------------------------------------------------------------------
	/Puts raw neighbours position into local storage for neighbours pos publisher
	/-----------------------------------------------------------------------------------*/
	void roscontroller::raw_neighbours_pos_put(int id, buzz_utility::Pos_struct pos_arr ){
		map< int, buzz_utility::Pos_struct >::iterator it = raw_neighbours_pos_map.find(id);
		if(it!=raw_neighbours_pos_map.end())
		raw_neighbours_pos_map.erase(it);
		raw_neighbours_pos_map.insert(make_pair(id, pos_arr));
		}

	/*--------------------------------------------------------
	/Set the current position of the robot callback
	/--------------------------------------------------------*/
	void roscontroller::set_cur_pos(double latitude,
			 double longitude,
			 double altitude){
		cur_pos [0] =latitude;
		cur_pos [1] =longitude;
		cur_pos [2] =altitude;
	}


	/*-----------------------------------------------------------
	/ Compute Range and Bearing of a neighbor in a local reference frame
	/ from GPS coordinates
	----------------------------------------------------------- */
	#define EARTH_RADIUS (double) 6371000.0
	#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))
	void roscontroller::cvt_rangebearing_coordinates(double nei[], double out[], double cur[]){
 		double d_lon = nei[1] - cur[1];
        	double d_lat = nei[0] - cur[0];
        	double ned_x = DEG2RAD(d_lat) * EARTH_RADIUS;
        	double ned_y = DEG2RAD(d_lon) * EARTH_RADIUS * cos(DEG2RAD(nei[0]));
		out[0] = sqrt(ned_x*ned_x+ned_y*ned_y);
		out[1] = atan2(ned_y,ned_x);
		out[2] = 0.0;
/*		double lat1 = cur[0]*M_PI/180.0;
		double lon1 = cur[1]*M_PI/180.0;
		double lat2 = nei[0]*M_PI/180.0;
		double lon2 = nei[1]*M_PI/180.0;
		out[0] = acos(sin(lat1) * sin(lat2) +cos(lat2) * cos(lat1)*cos(lon2-lon1))*EARTH_RADIUS;
		double y = sin(lon1-lon2)*cos(lat1);
		double x =  cos(lat2)*sin(lat1) - sin(lat2)*cos(lat1)*cos(lon1-lon2);
		out[1] = atan2(y,x)+M_PI;
		out[2] = 0.0;*/
	}

	/*------------------------------------------------------
	/ Update battery status into BVM from subscriber
	/------------------------------------------------------*/ 
	void roscontroller::battery(const mavros_msgs::BatteryStatus::ConstPtr& msg){
		buzzuav_closures::set_battery(msg->voltage,msg->current,msg->remaining);
		//ROS_INFO("voltage : %d  current : %d  remaining : %d",msg->voltage, msg->current, msg ->remaining);
	}

	/*----------------------------------------------------------------------
	/Update flight extended status into BVM from subscriber for solos
	/---------------------------------------------------------------------*/
	void roscontroller::flight_status_update(const mavros_msgs::State::ConstPtr& msg){
		// http://wiki.ros.org/mavros/CustomModes
		// TODO: Handle landing
		std::cout << "Message: " << msg->mode << std::endl;
		if(msg->mode == "GUIDED")
			buzzuav_closures::flight_status_update(1);
		else if (msg->mode == "LAND")
			buzzuav_closures::flight_status_update(4);
		else // ground standby = LOITER?
			buzzuav_closures::flight_status_update(1);//?
	}

	/*------------------------------------------------------------
	/Update flight extended status into BVM from subscriber
	------------------------------------------------------------*/
	void roscontroller::flight_extended_status_update(const mavros_msgs::ExtendedState::ConstPtr& msg){
		buzzuav_closures::flight_status_update(msg->landed_state);
	}
	/*-------------------------------------------------------------
	/ Update current position into BVM from subscriber
	/-------------------------------------------------------------*/
	void roscontroller::current_pos(const sensor_msgs::NavSatFix::ConstPtr& msg){
		set_cur_pos(msg->latitude,msg->longitude,msg->altitude);
		buzzuav_closures::set_currentpos(msg->latitude,msg->longitude,msg->altitude);
	}
	/*-------------------------------------------------------------
	/Set obstacle Obstacle distance table into BVM from subscriber
	/-------------------------------------------------------------*/	
	void roscontroller::obstacle_dist(const sensor_msgs::LaserScan::ConstPtr& msg){
		float obst[5];
		for(int i=0;i<5;i++)
			obst[i]=msg->ranges[i];
		buzzuav_closures::set_obstacle_dist(obst);
	}

	/*-------------------------------------------------------------
	/Push payload into BVM FIFO
	/-------------------------------------------------------------*/
	/*******************************************************************************************************/
	/* Message format of payload (Each slot is uint64_t)						       */
	/* _________________________________________________________________________________________________   */
	/*|	|     |	    |						     |			            |  */
	/*|Pos x|Pos y|Pos z|Size in Uint64_t|robot_id|Buzz_msg_size|Buzz_msg|Buzz_msgs with size.........  |  */
	/*|_____|_____|_____|________________________________________________|______________________________|  */
	/*******************************************************************************************************/		
	void roscontroller::payload_obt(const mavros_msgs::Mavlink::ConstPtr& msg){
		/*Check for Updater message, if updater message push it into updater FIFO*/
		if((uint64_t)msg->payload64[0]==(uint64_t)UPDATER_MESSAGE_CONSTANT && msg->payload64.size() > 5){
			uint16_t obt_msg_size=sizeof(uint64_t)*(msg->payload64.size());
			uint64_t message_obt[obt_msg_size];
			/* Go throught the obtained payload*/
			for(int i=0;i < (int)msg->payload64.size();i++){
				message_obt[i] =(uint64_t)msg->payload64[i];
			}

			uint8_t* pl =(uint8_t*)malloc(obt_msg_size);
   			memset(pl, 0,obt_msg_size);
			/* Copy packet into temporary buffer neglecting update constant */
   			memcpy((void*)pl, (void*)(message_obt+1) ,obt_msg_size);
			uint16_t unMsgSize = *(uint16_t*)(pl);
			//uint16_t tot;
			//tot+=sizeof(uint16_t);
     			fprintf(stdout,"Update packet, read msg size : %u \n",unMsgSize);	
			if(unMsgSize>0){
				code_message_inqueue_append((uint8_t*)(pl + sizeof(uint16_t)),unMsgSize);
				//fprintf(stdout,"before in queue process : utils\n");
			      	code_message_inqueue_process();
				//fprintf(stdout,"after in queue process : utils\n");
			}
   			delete[] pl;
		}
		/*BVM FIFO message*/
		else if(msg->payload64.size() > 3){
			uint64_t message_obt[msg->payload64.size()];
			/* Go throught the obtained payload*/
			for(int i=0;i < (int)msg->payload64.size();i++){
				message_obt[i] =(uint64_t)msg->payload64[i];
		        	//cout<<"[Debug:] obtaind message "<<message_obt[i]<<endl;
				//i++;
			}
			/* Extract neighbours position from payload*/
			double neighbours_pos_payload[3];
			memcpy(neighbours_pos_payload, message_obt, 3*sizeof(uint64_t));
			buzz_utility::Pos_struct raw_neigh_pos(neighbours_pos_payload[0],neighbours_pos_payload[1],neighbours_pos_payload[2]);
	//		cout<<"Got" << neighbours_pos_payload[0] <<", " << neighbours_pos_payload[1] << ", " << neighbours_pos_payload[2] << endl;
			double cvt_neighbours_pos_payload[3];
			cvt_rangebearing_coordinates(neighbours_pos_payload, cvt_neighbours_pos_payload, cur_pos);
			/*Extract robot id of the neighbour*/
	 		uint16_t* out = buzz_utility::u64_cvt_u16((uint64_t)*(message_obt+3));
			cout << "Rel Pos of " << (int)out[1] << ": " << cvt_neighbours_pos_payload[0] << ", "<< cvt_neighbours_pos_payload[1] << ", "<< cvt_neighbours_pos_payload[2] << endl;
			/*pass neighbour position to local maintaner*/
			buzz_utility::Pos_struct n_pos(cvt_neighbours_pos_payload[0],cvt_neighbours_pos_payload[1],cvt_neighbours_pos_payload[2]);
			/*Put RID and pos*/
			raw_neighbours_pos_put((int)out[1],raw_neigh_pos);		
			neighbours_pos_put((int)out[1],n_pos);
			delete[] out;
			buzz_utility::in_msg_process((message_obt+3));
		}
   
	}

	/*-----------------------------------------------------------
	/ Catch the ROS service call from a custom remote controller (Mission Planner)
	/ and send the requested commands to Buzz
	----------------------------------------------------------- */
	bool roscontroller::rc_callback(mavros_msgs::CommandLong::Request  &req,
		         mavros_msgs::CommandLong::Response &res){
		int rc_cmd;
   		switch(req.command){
			case mavros_msgs::CommandCode::NAV_TAKEOFF:
   				ROS_INFO("RC_call: TAKE OFF!!!!");
				rc_cmd=mavros_msgs::CommandCode::NAV_TAKEOFF;
				buzzuav_closures::rc_call(rc_cmd);
				res.success = true;
				break;
			case mavros_msgs::CommandCode::NAV_LAND:
   				ROS_INFO("RC_Call: LAND!!!!");
				rc_cmd=mavros_msgs::CommandCode::NAV_LAND;
				buzzuav_closures::rc_call(rc_cmd);
				res.success = true;
				break;
			case mavros_msgs::CommandCode::CMD_COMPONENT_ARM_DISARM:
				rc_cmd=mavros_msgs::CommandCode::CMD_COMPONENT_ARM_DISARM;
				armstate = req.param1;
				if(armstate){
   					ROS_INFO("RC_Call: ARM!!!!");
					buzzuav_closures::rc_call(rc_cmd);
					res.success = true;	
				}			
				else{
   					ROS_INFO("RC_Call: DISARM!!!!");
					buzzuav_closures::rc_call(rc_cmd+1);
					res.success = true;	
				}			
				break;
			case mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH:
   				ROS_INFO("RC_Call: GO HOME!!!!");
				rc_cmd=mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
				buzzuav_closures::rc_call(rc_cmd);
				res.success = true;
				break;
			case mavros_msgs::CommandCode::NAV_WAYPOINT:
   				ROS_INFO("RC_Call: GO TO!!!! ");
				double rc_goto[3];
				rc_goto[0] = req.param5;
				rc_goto[1] = req.param6;
				rc_goto[2] = req.param7;

				buzzuav_closures::rc_set_goto(rc_goto);
				rc_cmd=mavros_msgs::CommandCode::NAV_WAYPOINT;
				buzzuav_closures::rc_call(rc_cmd);
				res.success = true;
				break;
			default:
				res.success = false;
				break;
   		}
   		return true;
	}
	/*-----------------------------------------------------
	/Obtain robot id by subscribing to xbee robot id topic 
	/ TODO: check for integrity of this subscriber call back
	/----------------------------------------------------*/
	/*void roscontroller::set_robot_id(const std_msgs::UInt8::ConstPtr& msg){
	
	
	}*/
	
}


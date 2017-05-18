#include "roscontroller.h"
#include <thread>
namespace rosbzz_node{

	/*---------------
	/Constructor
	---------------*/
	roscontroller::roscontroller(ros::NodeHandle& n_c, ros::NodeHandle& n_c_priv)	
	{
		ROS_INFO("Buzz_node");
		/*Obtain parameters from ros parameter server*/
	  	Rosparameters_get(n_c_priv);
		/*Initialize publishers, subscribers and client*/
  		Initialize_pub_sub(n_c);
		/*Compile the .bzz file to .basm, .bo and .bdbg*/
		std::string fname = Compile_bzz(bzzfile_name);
 		bcfname = fname + ".bo";
 		dbgfname = fname + ".bdb";
		set_bzz_file(bzzfile_name.c_str());
		/*Initialize variables*/
		// Solo things
		SetMode("LOITER", 0);
		armstate = 0;
		multi_msg = true;
		// set stream rate - wait for the FC to be started
		SetStreamRate(0, 10, 1);
		// Get Robot Id - wait for Xbee to be started

		setpoint_counter = 0;
		fcu_timeout = TIMEOUT;

		cur_pos.longitude = 0;
		cur_pos.latitude = 0;
		cur_pos.altitude = 0;

		while(cur_pos.latitude == 0.0f){
			ROS_INFO("Waiting for GPS. ");
			ros::Duration(0.5).sleep();
			ros::spinOnce();
		}


		if(xbeeplugged){
			GetRobotId();
		} else {
			robot_id= strtol(robot_name.c_str() + 5, NULL, 10);
		}
		std::string  path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/")) + "/";
		path+="Update.log";
		log.open(path.c_str(), std::ios_base::trunc | std::ios_base::out);
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
		log.close();
	}

	void roscontroller::GetRobotId()
	{

		
		mavros_msgs::ParamGet::Request robot_id_srv_request; robot_id_srv_request.param_id="id";
        mavros_msgs::ParamGet::Response robot_id_srv_response;
		while(!xbeestatus_srv.call(robot_id_srv_request,robot_id_srv_response)){
			ros::Duration(0.1).sleep();
			ROS_ERROR("Waiting for Xbee to respond to get device ID");
		}

		robot_id=robot_id_srv_response.value.integer;

		//robot_id = 8;
	}

	/*-------------------------------------------------
	/rosbuzz_node loop method executed once every step
	/--------------------------------------------------*/
	void roscontroller::RosControllerRun(){

		/* Set the Buzz bytecode */
		if(buzz_utility::buzz_script_set(bcfname.c_str(), dbgfname.c_str(),robot_id)) {
			fprintf(stdout, "Bytecode file found and set\n");
			std::string standby_bo = Compile_bzz(stand_by) + ".bo";
			init_update_monitor(bcfname.c_str(),standby_bo.c_str());
                        ///////////////////////////////////////////////////////
                        // MAIN LOOP
                        //////////////////////////////////////////////////////
			while (ros::ok() && !buzz_utility::buzz_script_done())
  			{
      			/*Update neighbors position inside Buzz*/
     			//buzz_closure::neighbour_pos_callback(neighbours_pos_map);
				/*Neighbours of the robot published with id in respective topic*/
				neighbours_pos_publisher();
				/*Check updater state and step code*/
  				update_routine(bcfname.c_str(), dbgfname.c_str());
				/*Step buzz script */
      			buzz_utility::buzz_script_step();
				/*Prepare messages and publish them in respective topic*/
		  		prepare_msg_and_publish();
				/*call flight controler service to set command long*/
				flight_controller_service_call();
				/*Set multi message available after update*/
				if(get_update_status()){
					set_read_update_status();
					multi_msg=true;
					log<<cur_pos.latitude<<","<<cur_pos.longitude<<","<<cur_pos.altitude<<","; 
					collect_data(log);
					log<<std::endl;
				}
				/*Set ROBOTS variable for barrier in .bzz from neighbours count*/
				//no_of_robots=get_number_of_robots();
				get_number_of_robots();
				//if(neighbours_pos_map.size() >0) no_of_robots =neighbours_pos_map.size()+1;
				//buzz_utility::set_robot_var(no_of_robots);
				/*Set no of robots for updates TODO only when not updating*/
				if(multi_msg)
				updates_set_robots(no_of_robots);
				ROS_INFO("ROBOTS: %i , acutal : %i",(int)no_of_robots,(int)buzzdict_size(buzz_utility::get_vm()->swarmmembers)+1); 
	    			/*run once*/
	    			ros::spinOnce();
				/*loop rate of ros*/
				 ros::Rate loop_rate(BUZZRATE);
				 loop_rate.sleep();
				 if(fcu_timeout<=0)
					buzzuav_closures::rc_call(mavros_msgs::CommandCode::NAV_LAND);
				 else
					fcu_timeout -= 1/BUZZRATE;
 				/*sleep for the mentioned loop rate*/
    			timer_step+=1;
   				maintain_pos(timer_step);

   				//std::cout<< "HOME: " << home.latitude << ", " << home.longitude;
			}
			/* Destroy updater and Cleanup */
    			//update_routine(bcfname.c_str(), dbgfname.c_str(),1);
  		}
	}

	/*--------------------------------------------------------
	/ Get all required parameters from the ROS launch file
	/-------------------------------------------------------*/
	void roscontroller::Rosparameters_get(ros::NodeHandle& n_c){
		
		/*Obtain .bzz file name from parameter server*/
	  	if(n_c.getParam("bzzfile_name", bzzfile_name));
	  	else {ROS_ERROR("Provide a .bzz file to run in Launch file"); system("rosnode kill rosbuzz_node");}  
	  	/*Obtain rc service option from parameter server*/
	  	if(n_c.getParam("rcclient", rcclient)){
		     	if(rcclient==true){
			    	/*Service*/
			    	if(!n_c.getParam("rcservice_name", rcservice_name))
                                    {ROS_ERROR("Provide a name topic name for rc service in Launch file"); system("rosnode kill rosbuzz_node");}
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
		/*Obtain standby script to run during update*/
		n_c.getParam("stand_by", stand_by);
                
  		if(n_c.getParam("xbee_plugged", xbeeplugged));
  		else {ROS_ERROR("Provide the xbee plugged boolean in Launch file"); system("rosnode kill rosbuzz_node");} 
  		if(!xbeeplugged){
                    if(n_c.getParam("name", robot_name));
                    else {ROS_ERROR("Provide the xbee plugged boolean in Launch file"); system("rosnode kill rosbuzz_node");}
                }else
			n_c.getParam("xbee_status_srv", xbeesrv_name);

		std::cout<< "////////////////// " << xbeesrv_name;

		GetSubscriptionParameters(n_c);
		// initialize topics to null?

	}
	/*-----------------------------------------------------------------------------------
	/Obtains publisher, subscriber and services from yml file based on the robot used
	/-----------------------------------------------------------------------------------*/
	void roscontroller::GetSubscriptionParameters(ros::NodeHandle& node_handle){
		//todo: make it as an array in yaml?
		m_sMySubscriptions.clear();
		std::string gps_topic, gps_type;
  		if(node_handle.getParam("topics/gps", gps_topic));
  		else {ROS_ERROR("Provide a gps topic in Launch file"); system("rosnode kill rosbuzz_node");}  
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

		std::string altitude_topic, altitude_type;
		node_handle.getParam("topics/altitude", altitude_topic);
		node_handle.getParam("type/altitude", altitude_type);
		m_smTopic_infos.insert(pair <std::string, std::string>(altitude_topic, altitude_type));

  		/*Obtain fc client name from parameter server*/
  		if(node_handle.getParam("topics/fcclient", fcclient_name));
  		else {ROS_ERROR("Provide a fc client name in Launch file"); system("rosnode kill rosbuzz_node");}  
  		if(node_handle.getParam("topics/setpoint", setpoint_name));
  		else {ROS_ERROR("Provide a Set Point name in Launch file"); system("rosnode kill rosbuzz_node");}  
  		if(node_handle.getParam("topics/armclient", armclient));
  		else {ROS_ERROR("Provide an arm client name in Launch file"); system("rosnode kill rosbuzz_node");}  
  		if(node_handle.getParam("topics/modeclient", modeclient));
  		else {ROS_ERROR("Provide a mode client name in Launch file"); system("rosnode kill rosbuzz_node");}  
  		if(node_handle.getParam("topics/stream", stream_client_name));
  		else {ROS_ERROR("Provide a mode client name in Launch file"); system("rosnode kill rosbuzz_node");}
  		if(node_handle.getParam("topics/localpos", local_pos_sub_name));
  		else {ROS_ERROR("Provide a localpos name in YAML file"); system("rosnode kill rosbuzz_node");}



	}

	/*--------------------------------------------------------
	/ Create all required subscribers, publishers and ROS service clients
	/-------------------------------------------------------*/
	void roscontroller::Initialize_pub_sub(ros::NodeHandle& n_c){
		/*subscribers*/

		Subscribe(n_c);

  		//current_position_sub = n_c.subscribe("/global_position", 1000, &roscontroller::current_pos,this);
  		//battery_sub = n_c.subscribe("/power_status", 1000, &roscontroller::battery,this);
  		payload_sub = n_c.subscribe(in_payload, 1000, &roscontroller::payload_obt,this);
		//flight_status_sub =n_c.subscribe("/flight_status",100, &roscontroller::flight_extended_status_update,this);
		//Robot_id_sub = n_c.subscribe("/device_id_xbee_", 1000, &roscontroller::set_robot_id,this);
  		obstacle_sub= n_c.subscribe("guidance/obstacle_distance",100, &roscontroller::obstacle_dist,this);
  		/*publishers*/
		payload_pub = n_c.advertise<mavros_msgs::Mavlink>(out_payload, 1000);
		neigh_pos_pub = n_c.advertise<rosbuzz::neigh_pos>("neighbours_pos",1000);
		localsetpoint_nonraw_pub = n_c.advertise<geometry_msgs::PoseStamped>(setpoint_name,1000);
		/* Service Clients*/
		arm_client = n_c.serviceClient<mavros_msgs::CommandBool>(armclient);
		mode_client =  n_c.serviceClient<mavros_msgs::SetMode>(modeclient);
		mav_client = n_c.serviceClient<mavros_msgs::CommandLong>(fcclient_name);
        if(rcclient==true)
        	service = n_c.advertiseService(rcservice_name, &roscontroller::rc_callback,this);
		ROS_INFO("Ready to receive Mav Commands from RC client");
		xbeestatus_srv = n_c.serviceClient<mavros_msgs::ParamGet>(xbeesrv_name);
		stream_client = n_c.serviceClient<mavros_msgs::StreamRate>(stream_client_name);
                
  		users_sub = n_c.subscribe("users_pos", 1000, &roscontroller::users_pos,this);
  		local_pos_sub = n_c.subscribe(local_pos_sub_name, 1000, &roscontroller::local_pos_callback, this);


		multi_msg=true;
	}
	/*---------------------------------------
	/Robot independent subscribers
	/--------------------------------------*/
	void roscontroller::Subscribe(ros::NodeHandle& n_c){

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
  			else if(it->second == "std_msgs/Float64"){
  				relative_altitude_sub = n_c.subscribe(it->first, 1000, &roscontroller::current_rel_alt, this);
  			}

	  		std::cout << "Subscribed to: " << it->first << endl;
  		}
	}

	/*--------------------------------------------------------
	/ Create Buzz bytecode from the bzz script inputed
	/-------------------------------------------------------*/
	std::string roscontroller::Compile_bzz(std::string bzzfile_name){
		/*TODO: change to bzzc instead of bzzparse and also add -I for includes*/
		/*Compile the buzz code .bzz to .bo*/
		stringstream bzzfile_in_compile;
	    std::string  path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/")) + "/";
		//bzzfile_in_compile << path << "/";
		//path = bzzfile_in_compile.str();
		//bzzfile_in_compile.str("");
		std::string  name = bzzfile_name.substr(bzzfile_name.find_last_of("/\\") + 1);
 		name = name.substr(0,name.find_last_of("."));
		bzzfile_in_compile << "bzzc -I " << path << "include/"; //<<" "<<path<< name<<".basm";
		//bzzfile_in_compile.str("");
        //bzzfile_in_compile <<"bzzasm "<<path<<name<<".basm "<<path<<name<<".bo "<<path<<name<<".bdbg";
   		//system(bzzfile_in_compile.str().c_str());
		//bzzfile_in_compile.str("");
		bzzfile_in_compile << " -b " << path << name << ".bo";
		//bcfname = bzzfile_in_compile.str();
		//std::string tmp_bcfname = path + name + ".bo";
		//bzzfile_in_compile.str("");
		bzzfile_in_compile << " -d " << path << name << ".bdb ";
		//bzzfile_in_compile << " -a " << path << name << ".asm ";
		bzzfile_in_compile << bzzfile_name;
		//std::string tmp_dbgfname = path + name + ".bdb";

		ROS_WARN("Launching buzz compilation: %s", bzzfile_in_compile.str().c_str());

   		system(bzzfile_in_compile.str().c_str());

		return path + name;
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
				ROS_WARN("FC Arm Service called!");
			else
				ROS_WARN("FC Arm Service call failed!");
		} else {
			ROS_WARN("FC Arm Service call failed!");
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
   		uint64_t* payload_out_ptr= buzz_utility::obt_out_msg();
    		uint64_t  position[3];
  		/*Appened current position to message*/
		  	double tmp[3];tmp[0]=cur_pos.latitude;tmp[1]=cur_pos.longitude;tmp[2]=cur_pos.altitude;
    		memcpy(position, tmp, 3*sizeof(uint64_t));
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
			// allocate mem and clear it
			buff_send =(uint8_t*)malloc(sizeof(uint16_t)+updater_msgSize);
			memset(buff_send, 0,sizeof(uint16_t)+updater_msgSize);
		   	// Append updater msg size
		  	 *(uint16_t*)(buff_send + tot)=updater_msgSize;
		   	//fprintf(stdout,"Updater sent msg size : %i \n", (int)updater_msgSize);
      		   	tot += sizeof(uint16_t);
		   	// Append updater msgs
    		   	memcpy(buff_send + tot, (uint8_t*)(getupdater_out_msg()), updater_msgSize);
		   	tot += updater_msgSize;
		   	// Destroy the updater out msg queue
		    	destroy_out_msg_queue();
		    	uint16_t total_size =(ceil((float)(float)tot/(float)sizeof(uint64_t))); 
		    	uint64_t* payload_64 = new uint64_t[total_size];
	  	    	memcpy((void*)payload_64, (void*)buff_send, total_size*sizeof(uint64_t));
		    	free(buff_send);
		    	// Send a constant number to differenciate updater msgs
		    	update_packets.payload64.push_back((uint64_t)UPDATER_MESSAGE_CONSTANT);
		    	for(int i=0;i<total_size;i++){
				update_packets.payload64.push_back(payload_64[i]);
		    	}
		    	// Add Robot id and message number to the published message
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
	void roscontroller::flight_controller_service_call() {
		/* flight controller client call if requested from Buzz*/
		/*FC call for takeoff,land, gohome and arm/disarm*/
		int tmp = buzzuav_closures::bzz_cmd();
		double* goto_pos = buzzuav_closures::getgoto();
		if (tmp == buzzuav_closures::COMMAND_TAKEOFF || tmp== buzzuav_closures::COMMAND_LAND || tmp==buzzuav_closures::COMMAND_GOHOME) {
			cmd_srv.request.param7 = goto_pos[2];
			cmd_srv.request.command =  buzzuav_closures::getcmd();
			//std::cout << " CMD: " << buzzuav_closures::getcmd() << std::endl;
			// SOLO SPECIFIC: SITL DOES NOT USE 21 TO LAND -- workaround: set to LAND mode
			switch(buzzuav_closures::getcmd()){
			case mavros_msgs::CommandCode::NAV_LAND:
				if(current_mode != "LAND") {
                                    SetMode("LAND", 0);
                                    armstate = 0;
                                    Arm();
                                }
				if (mav_client.call(cmd_srv)){
					ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success);
				} else
					{ROS_ERROR("Failed to call service from flight controller");}
				armstate = 0;
				break;
			case mavros_msgs::CommandCode::NAV_TAKEOFF:
				if(!armstate){

					SetMode("LOITER", 0);
					armstate = 1;
					Arm();
					ros::Duration(0.5).sleep();
					// Registering HOME POINT.
					home = cur_pos;
				}
				if(current_mode != "GUIDED")
					SetMode("GUIDED", 2000); // for real solo, just add 2000ms delay (it should always be in loiter after arm/disarm)
				if (mav_client.call(cmd_srv))
					{ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success);}
				else
					ROS_ERROR("Failed to call service from flight controller");
				break;
			}

		} else if (tmp == buzzuav_closures::COMMAND_GOTO) { /*FC call for goto*/
			/*prepare the goto publish message */
			cmd_srv.request.param5 = goto_pos[0];
			cmd_srv.request.param6 = goto_pos[1];
			cmd_srv.request.param7 = goto_pos[2];
			cmd_srv.request.command = buzzuav_closures::getcmd();
			if (mav_client.call(cmd_srv)) {
				ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success);
			} else
				ROS_ERROR("Failed to call service from flight controller");
			cmd_srv.request.command = mavros_msgs::CommandCode::CMD_MISSION_START;
			if (mav_client.call(cmd_srv)) {
				ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success);
			} else
				ROS_ERROR("Failed to call service from flight controller");
		} else if (tmp == buzzuav_closures::COMMAND_ARM) { /*FC call for arm*/
			if(!armstate){
				SetMode("LOITER", 0);
				armstate = 1;
				Arm();
			}
		} else if (tmp == buzzuav_closures::COMMAND_DISARM) {
			if(armstate){
				armstate = 0;
				SetMode("LOITER", 0);
				Arm();
			}
		} else if (tmp == buzzuav_closures::COMMAND_MOVETO) { /*Buzz call for moveto*/
			roscontroller::SetLocalPosition(goto_pos[0], goto_pos[1], goto_pos[2], 0);

			//roscontroller::SetLocalPositionNonRaw(goto_pos[1], goto_pos[0], goto_pos[2], 0);
		}
	}
	/*----------------------------------------------
	/Refresh neighbours Position for every ten step
	/---------------------------------------------*/
	void roscontroller::maintain_pos(int tim_step){
		if(timer_step >=BUZZRATE){
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
		cur_pos.latitude =latitude;
		cur_pos.longitude =longitude;
		cur_pos.altitude =altitude;
	}

	/*-----------------------------------------------------------
	/ Compute Range and Bearing of a neighbor in a local reference frame
	/ from GPS coordinates
	----------------------------------------------------------- */
	void ecef2ned_matrix(const double ref_ecef[3], double M[3][3]) {
            double hyp_az, hyp_el;
            double sin_el, cos_el, sin_az, cos_az;

            /* Convert reference point to spherical earth centered coordinates. */
            hyp_az = sqrt(ref_ecef[0]*ref_ecef[0] + ref_ecef[1]*ref_ecef[1]);
            hyp_el = sqrt(hyp_az*hyp_az + ref_ecef[2]*ref_ecef[2]);
            sin_el = ref_ecef[2] / hyp_el;
            cos_el = hyp_az / hyp_el;
            sin_az = ref_ecef[1] / hyp_az;
            cos_az = ref_ecef[0] / hyp_az;

            M[0][0] = -sin_el * cos_az;
            M[0][1] = -sin_el * sin_az;
            M[0][2] = cos_el;
            M[1][0] = -sin_az;
            M[1][1] = cos_az;
            M[1][2] = 0.0;
            M[2][0] = -cos_el * cos_az;
            M[2][1] = -cos_el * sin_az;
            M[2][2] = -sin_el;
            }
            void matrix_multiply(uint32_t n, uint32_t m, uint32_t p, const double *a,
                            const double *b, double *c)
            {
            uint32_t i, j, k;
            for (i = 0; i < n; i++)
                for (j = 0; j < p; j++) {
                c[p*i + j] = 0;
                for (k = 0; k < m; k++)
                    c[p*i + j] += a[m*i+k] * b[p*k + j];
                }
            }

	void roscontroller::gps_rb(GPS nei_pos, double out[])
    {
        float ned_x=0.0, ned_y=0.0;
		gps_ned_cur(ned_x, ned_y, nei_pos);
		out[0] = sqrt(ned_x*ned_x+ned_y*ned_y);
        //out[0] = std::floor(out[0] * 1000000) / 1000000;
		out[1] = atan2(ned_y,ned_x);
        //out[1] = std::floor(out[1] * 1000000) / 1000000;
		out[2] = 0.0;
	}

	void roscontroller::gps_ned_cur(float &ned_x, float &ned_y, GPS t)
	{
		gps_convert_ned(ned_x, ned_y,
			t.longitude, t.latitude,
			cur_pos.longitude, cur_pos.latitude);
	}

	void roscontroller::gps_ned_home(float &ned_x, float &ned_y)
	{
		gps_convert_ned(ned_x, ned_y,
			cur_pos.longitude, cur_pos.latitude,
			home.longitude, home.latitude);
	}

	void roscontroller::gps_convert_ned(float &ned_x, float &ned_y,
			double gps_t_lon, double gps_t_lat,
			double gps_r_lon, double gps_r_lat)
	{
		double d_lon = gps_t_lon - gps_r_lon;
		double d_lat = gps_t_lat - gps_r_lat;
		ned_x = DEG2RAD(d_lat) * EARTH_RADIUS;
		ned_y = DEG2RAD(d_lon) * EARTH_RADIUS * cos(DEG2RAD(gps_t_lat));
	};

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
			buzzuav_closures::flight_status_update(2);
		else if (msg->mode == "LAND")
			buzzuav_closures::flight_status_update(1);
		else // ground standby = LOITER?
			buzzuav_closures::flight_status_update(7);//?
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
		//ROS_INFO("Altitude out: %f", cur_rel_altitude);
		fcu_timeout = TIMEOUT;
                //double lat = std::floor(msg->latitude * 1000000) / 1000000;
                //double lon = std::floor(msg->longitude * 1000000) / 1000000;
		/*if(cur_rel_altitude<1.2){
			home[0]=lat;
			home[1]=lon;
			home[2]=cur_rel_altitude;
		}*/
		set_cur_pos(msg->latitude, msg->longitude, cur_rel_altitude);//msg->altitude);
		buzzuav_closures::set_currentpos(msg->latitude, msg->longitude, cur_rel_altitude);//msg->altitude);
	}

	void roscontroller::local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& pose){
		local_pos_new[0] = pose->pose.position.x;
		local_pos_new[1] = pose->pose.position.y;
		local_pos_new[2] = pose->pose.position.z;
	}

	void roscontroller::users_pos(const rosbuzz::neigh_pos data){

                int n = data.pos_neigh.size();
        //	ROS_INFO("Users array size: %i\n", n);
                if(n>0)
                {
                        for(int it=0; it<n; ++it)
                        {
				buzz_utility::add_user(data.pos_neigh[it].position_covariance_type,data.pos_neigh[it].latitude, data.pos_neigh[it].longitude, data.pos_neigh[it].altitude);
                        }

                }
		
	}
	/*-------------------------------------------------------------
	/ Update altitude into BVM from subscriber
	/-------------------------------------------------------------*/
	void roscontroller::current_rel_alt(const std_msgs::Float64::ConstPtr& msg){
		//ROS_INFO("Altitude in: %f", msg->data);
		cur_rel_altitude = (double)msg->data;

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
	
	void roscontroller::SetLocalPosition(float x, float y, float z, float yaw){
		// http://docs.ros.org/kinetic/api/mavros_msgs/html/msg/PositionTarget.html
		// http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-local-ned

		geometry_msgs::PoseStamped moveMsg;
		moveMsg.header.stamp = ros::Time::now();
		moveMsg.header.seq = setpoint_counter++;
		moveMsg.header.frame_id = 1;
		float ned_x, ned_y;
		gps_ned_home(ned_x, ned_y);
       //         ROS_INFO("[%i] ROSBuzz Home: %.7f, %.7f", robot_id, home[0], home[1]);
       //         ROS_INFO("[%i] ROSBuzz LocalPos: %.7f, %.7f", robot_id, local_pos[0], local_pos[1]);
                    
		/*prepare the goto publish message ATTENTION: ENU FRAME FOR MAVROS STANDARD (then converted to NED)*/
		//target[0]+=y;	target[1]+=x;
		moveMsg.pose.position.x = local_pos_new[0]+y;//ned_y+y;
		moveMsg.pose.position.y = local_pos_new[1]+x;//ned_x+x;
		moveMsg.pose.position.z = z;

		moveMsg.pose.orientation.x = 0;
		moveMsg.pose.orientation.y = 0;
		moveMsg.pose.orientation.z = 0;
		moveMsg.pose.orientation.w = 1;

                // To prevent drifting from stable position.
		//if(fabs(x)>0.005 || fabs(y)>0.005) {
                    localsetpoint_nonraw_pub.publish(moveMsg);
                    //ROS_INFO("Sent local NON RAW position message!");
        //        }
	}

	void roscontroller::SetMode(std::string mode, int delay_miliseconds){
		// wait if necessary
		if (delay_miliseconds != 0){
			std::this_thread::sleep_for( std::chrono::milliseconds ( delay_miliseconds ) );
		}
		// set mode
		mavros_msgs::SetMode set_mode_message;
		set_mode_message.request.base_mode = 0;
		set_mode_message.request.custom_mode = mode;
		current_mode = mode;
		if(mode_client.call(set_mode_message)) {
			ROS_INFO("Set Mode Service call successful!");
		} else {
			ROS_INFO("Set Mode Service call failed!");
		}
	}


	void roscontroller::SetStreamRate(int id, int rate, int on_off){
		mavros_msgs::StreamRate message;
		message.request.stream_id = id;
		message.request.message_rate = rate;
		message.request.on_off = on_off;

		while(!stream_client.call(message)){
			ROS_INFO("Set stream rate call failed!, trying again...");
			ros::Duration(0.1).sleep();
		}
		ROS_INFO("Set stream rate call successful");
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
   			free(pl);
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
			GPS nei_pos;
			nei_pos.latitude=neighbours_pos_payload[0];
			nei_pos.longitude=neighbours_pos_payload[1];
			nei_pos.altitude=neighbours_pos_payload[2];
			double cvt_neighbours_pos_payload[3];
	//		cout<<"Got" << neighbours_pos_payload[0] <<", " << neighbours_pos_payload[1] << ", " << neighbours_pos_payload[2] << endl;
			gps_rb(nei_pos, cvt_neighbours_pos_payload);
			/*Extract robot id of the neighbour*/
	 		uint16_t* out = buzz_utility::u64_cvt_u16((uint64_t)*(message_obt+3));
			//cout << "Rel Pos of " << (int)out[1] << ": " << cvt_neighbours_pos_payload[0] << ", "<< cvt_neighbours_pos_payload[1] << ", "<< cvt_neighbours_pos_payload[2] << endl;
			/*pass neighbour position to local maintaner*/
			buzz_utility::Pos_struct n_pos(cvt_neighbours_pos_payload[0],cvt_neighbours_pos_payload[1],cvt_neighbours_pos_payload[2]);
			/*Put RID and pos*/
			raw_neighbours_pos_put((int)out[1],raw_neigh_pos);
			/* TODO: remove roscontroller local map array for neighbors */	
			neighbours_pos_put((int)out[1],n_pos);
			buzzuav_closures::neighbour_pos_callback((int)out[1],n_pos.x,n_pos.y,n_pos.z);
			delete[] out;
			buzz_utility::in_msg_append((message_obt+3));
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
   				ROS_INFO("RC_Call: LAND!!!! sending land");
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
   				ROS_INFO("RC_Call: GO TO!!!! --- Doing this! ");
				double rc_goto[3];
				// testing PositionTarget
				rc_goto[0] = req.param5;
				rc_goto[1] = req.param6;
				rc_goto[2] = req.param7;
				// for test
				//SetLocalPosition(rc_goto[0], rc_goto[1], rc_goto[2], 0);

				buzzuav_closures::rc_set_goto(rc_goto);
				rc_cmd= mavros_msgs::CommandCode::NAV_WAYPOINT;
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
	void roscontroller::get_number_of_robots(){
		int cur_robots=(int)buzzdict_size(buzz_utility::get_vm()->swarmmembers)+1;
		if(no_of_robots==0){
			no_of_robots=cur_robots;
						
		}
		else{
			if(no_of_robots!=cur_robots && no_cnt==0){
				no_cnt++;
				old_val=cur_robots;
			
			}			
			else if(no_cnt!=0 && old_val==cur_robots){
				no_cnt++;
				if(no_cnt>=150 || cur_robots > no_of_robots){
					no_of_robots=cur_robots;
					no_cnt=0;
				}
			}
			else{
				no_cnt=0;
			}
		}
		/*
		if(count_robots.current !=0){
			std::map< int,  int> count_count;
			uint8_t index=0;
			count_robots.history[count_robots.index]=neighbours_pos_map.size()+1;
			//count_robots.current=neighbours_pos_map.size()+1;
			count_robots.index++;	
			if(count_robots.index >9) count_robots.index =0;	
			for(int i=0;i<10;i++){
				if(count_robots.history[i]==count_robots.current){
					current_count++;	
				}
				else{
					if(count_robots.history[i]!=0){
						odd_count++;
						odd_val=count_robots.history[i];
					}
				}
			}
			if(odd_count>current_count){
				count_robots.current=odd_val;
			}	
		}
		else{
			if(neighbours_pos_map.size()!=0){
				count_robots.history[count_robots.index]=neighbours_pos_map.size()+1;
				//count_robots.current=neighbours_pos_map.size()+1;
				count_robots.index++;
				if(count_robots.index >9){ 
					count_robots.index =0;
					count_robots.current=neighbours_pos_map.size()+1;
				} 
			}
		}
		*/
	}

}

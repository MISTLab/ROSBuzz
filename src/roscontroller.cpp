
#include "roscontroller.h"


namespace rosbzz_node{

	/***Constructor***/
	roscontroller::roscontroller()	
	{
		ROS_INFO("Buzz_node");
		/*Obtain parameters from ros parameter server*/
	  	Rosparameters_get();
		/*Initialize publishers, subscribers and client*/
  		Initialize_pub_sub();
		/*Compile the .bzz file to .basm, .bo and .bdbg*/
 		Compile_bzz();
	}

	/***Destructor***/
	roscontroller::~roscontroller()
	{
		/* All done */  
		/* Cleanup */
		buzz_utility::buzz_script_destroy();
 		/* Stop the robot */
   		uav_done();
	}
	
	/*rosbuzz_node run*/
	void roscontroller::RosControllerRun(){
		/* Set the Buzz bytecode */
		if(buzz_utility::buzz_script_set(bcfname.c_str(), dbgfname.c_str(),robot_id)) {
			fprintf(stdout, "Bytecode file found and set\n");
			while (ros::ok() && !buzz_utility::buzz_script_done())
  			{
      				/*Update neighbors position inside Buzz*/
     				buzz_utility::neighbour_pos_callback(neighbours_pos_map);
      				/*Step buzz script */
      				buzz_utility::buzz_script_step();
				/*Prepare messages and publish them in respective topic*/
		  		prepare_msg_and_publish();
    				/*run once*/
    				ros::spinOnce();
				/*loop rate of ros*/
				ros::Rate loop_rate(1);
    				/*sleep for the mentioned loop rate*/
    				loop_rate.sleep();
    				timer_step+=1;
   				maintain_pos(timer_step);
				
			}
  		}
	}

	void roscontroller::Rosparameters_get(){
		/*Obtain .bzz file name from parameter server*/
	  	if(ros::param::get("/rosbuzz_node/bzzfile_name", bzzfile_name));
	  	else {ROS_ERROR("Provide a .bzz file to run in Launch file"); system("rosnode kill rosbuzz_node");}  
	  	/*Obtain rc service option from parameter server*/
	  	if(ros::param::get("/rosbuzz_node/rcclient", rcclient)){
		     	if(rcclient==true){
			    	/*Service*/
			    	if(ros::param::get("/rosbuzz_node/rcservice_name", rcservice_name)){
				        service = n_c.advertiseService(rcservice_name, &roscontroller::rc_callback,this);
				    	ROS_INFO("Ready to receive Mav Commands from RC client");
			        }
			        else{ROS_ERROR("Provide a name topic name for rc service in Launch file"); system("rosnode kill rosbuzz_node");}
		        }
     			else if(rcclient==false){ROS_INFO("RC service is disabled");}
  		}
  		else{ROS_ERROR("Provide a rc client option: yes or no in Launch file"); system("rosnode kill rosbuzz_node");} 
  		/*Obtain fc client name from parameter server*/
  		if(ros::param::get("/rosbuzz_node/fcclient_name", fcclient_name));
  		else {ROS_ERROR("Provide a fc client name in Launch file"); system("rosnode kill rosbuzz_node");}  
  		/*Obtain robot_id from parameter server*/
  		ros::param::get("/rosbuzz_node/robot_id", robot_id);	

	}

	void roscontroller::Initialize_pub_sub(){
		/*subscribers*/
  		current_position_sub = n_c.subscribe("/dji_sdk/global_position", 1000, &roscontroller::current_pos,this);
  		battery_sub = n_c.subscribe("/dji_sdk/power_status", 1000, &roscontroller::battery,this);
  		payload_sub = n_c.subscribe("inMavlink", 1000, &roscontroller::payload_obt,this);
  		/*publishers*/
		payload_pub = n_c.advertise<mavros_msgs::Mavlink>("outMavlink", 1000);
		/* Clients*/
  		mav_client = n_c.serviceClient<mavros_msgs::CommandInt>(fcclient_name);

	}
	
	void roscontroller::Compile_bzz(){
		/*Compile the buzz code .bzz to .bo*/
		stringstream bzzfile_in_compile;
	        std::string  path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/"));
   		bzzfile_in_compile << "bzzparse "<<bzzfile_name<<" "<<path<< "/out.basm";
   		//system("rm ../catkin_ws/src/rosbuzz/src/out.basm ../catkin_ws/src/rosbuzz/src/out.bo ../catkin_ws/src/rosbuzz/src/out.bdbg");
   		system(bzzfile_in_compile.str().c_str());
		bzzfile_in_compile.str("");
           	bzzfile_in_compile <<"bzzasm "<<path<<"/out.basm "<<path<<"/out.bo "<<path<<"/out.bdbg";
   		system(bzzfile_in_compile.str().c_str());
		bzzfile_in_compile.str("");
		bzzfile_in_compile <<path<<"/out.bo";
		bcfname = bzzfile_in_compile.str();
		bzzfile_in_compile.str("");
		bzzfile_in_compile <<path<<"/out.bdbg";
		dbgfname = bzzfile_in_compile.str();

	}
	
	void roscontroller::prepare_msg_and_publish(){
		/*prepare the goto publish message */
    		double* goto_pos = buzzuav_closures::getgoto();
		cmd_srv.request.param1 = goto_pos[0];
    		cmd_srv.request.param2 = goto_pos[1];
    		cmd_srv.request.param3 = goto_pos[2];
    		cmd_srv.request.command =  buzzuav_closures::getcmd();
 		/* flight controller client call*/	
    		if (mav_client.call(cmd_srv)){ ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success); }
    		else{ ROS_ERROR("Failed to call service 'djicmd'"); }
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
    		/*int i=0;
		uint64_t message_obt[payload_out.payload64.size()];
    		for(std::vector<long unsigned int>::const_iterator it = payload_out.payload64.begin();
			it != payload_out.payload64.end(); ++it){
			message_obt[i] =(uint64_t) *it;
         		cout<<" [Debug:] sent message "<<*it<<endl;
			i++;
        	}*/
    		/*publish prepared messages in respective topic*/
    		payload_pub.publish(payload_out);
    		delete[] out;
    		delete[] payload_out_ptr;
		}


	/*Refresh neighbours Position for every ten step*/
	void roscontroller::maintain_pos(int tim_step){
		if(timer_step >=10){
		neighbours_pos_map.clear();
		timer_step=0;
		}
	}

	/*Maintain neighbours position*/
	void roscontroller::neighbours_pos_maintain(int id, buzz_utility::Pos_struct pos_arr ){
		map< int, buzz_utility::Pos_struct >::iterator it = neighbours_pos_map.find(id);
		if(it!=neighbours_pos_map.end())
		neighbours_pos_map.erase(it);
		neighbours_pos_map.insert(make_pair(id, pos_arr));
		}


	/*Set the current position of the robot callback*/
	void roscontroller::set_cur_pos(double latitude,
			 double longitude,
			 double altitude){
		cur_pos [0] =latitude;
		cur_pos [1] =longitude;
		cur_pos [2] =altitude;

	}

	/*convert from catresian to spherical coordinate system callback */
	double* roscontroller::cvt_spherical_coordinates(double neighbours_pos_payload []){
		double latitude,longitude,altitude;
		latitude=neighbours_pos_payload[0];
		longitude = neighbours_pos_payload[1];
		altitude=neighbours_pos_payload[2];
		neighbours_pos_payload[0]=sqrt(pow(latitude,2.0)+pow(longitude,2.0)+pow(altitude,2.0));
		neighbours_pos_payload[1]=atan(longitude/latitude);
		neighbours_pos_payload[2]=atan((sqrt(pow(latitude,2.0)+pow(longitude,2.0)))/altitude);
		return neighbours_pos_payload;
	}

	/*battery status callback*/ 
	void roscontroller::battery(const mavros_msgs::BatteryStatus::ConstPtr& msg){
		buzzuav_closures::set_battery(msg->voltage,msg->current,msg->remaining);
	}

	/*current position callback*/
	void roscontroller::current_pos(const sensor_msgs::NavSatFix::ConstPtr& msg){
		set_cur_pos(msg->latitude,msg->longitude,msg->altitude);
	}

	/*payload callback callback*/
	void roscontroller::payload_obt(const mavros_msgs::Mavlink::ConstPtr& msg){
		uint64_t message_obt[msg->payload64.size()];
		int i = 0;

		/* Go throught the obtained payload*/
		for(i=0;i < (int)msg->payload64.size();i++){
			message_obt[i] =(uint64_t)msg->payload64[i];
                	//cout<<"[Debug:] obtaind message "<<message_obt[i]<<endl;
			//i++;
        	}
		/* Extract neighbours position from payload*/
		double neighbours_pos_payload[3];
		memcpy(neighbours_pos_payload, message_obt, 3*sizeof(uint64_t));
		/*Convert obtained position to relative position*/
		for(i=0;i<3;i++){
			neighbours_pos_payload[i]=neighbours_pos_payload[i]-cur_pos[i];
		}
		double* cvt_neighbours_pos_payload = cvt_spherical_coordinates(neighbours_pos_payload);
		/*Extract robot id of the neighbour*/
 		uint16_t* out = buzz_utility::u64_cvt_u16((uint64_t)*(message_obt+3));  
		/*pass neighbour position to local maintaner*/
		buzz_utility::Pos_struct n_pos(cvt_neighbours_pos_payload[0],cvt_neighbours_pos_payload[1],cvt_neighbours_pos_payload[2]);
		neighbours_pos_maintain((int)out[1],n_pos);
		delete[] out;
		buzz_utility::in_msg_process((message_obt+3));
   
	}
 
	/* RC command service */
	bool roscontroller::rc_callback(mavros_msgs::CommandInt::Request  &req,
		         mavros_msgs::CommandInt::Response &res){
		int rc_cmd;
		if(req.command==oldcmdID)
		return true;
		else oldcmdID=req.command;
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
			case mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH:
   				ROS_INFO("RC_Call: GO HOME!!!!");
				rc_cmd=mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
				buzzuav_closures::rc_call(rc_cmd);
				res.success = true;
				break;
			case mavros_msgs::CommandCode::NAV_WAYPOINT:
   				ROS_INFO("RC_Call: GO TO!!!! x = %f , y = %f , Z = %f",req.param1,req.param2,req.param3);
				double rc_goto[3];
   				rc_goto[0]=req.param1;
				rc_goto[1]=req.param2;
				rc_goto[2]=req.param3;
				buzzuav_closures::set_goto(rc_goto);
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

}



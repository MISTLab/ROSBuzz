/** @file      roscontroller.cpp
 *  @version   1.0
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS.
 *  @author    Vivek Shankar Varadharajan and David St-Onge
 *  @copyright 2016 MistLab. All rights reserved.
 */

#include "roscontroller.h"
#include <thread>
namespace rosbzz_node
{
const string roscontroller::CAPTURE_SRV_DEFAULT_NAME = "/image_sender/capture_image";
const bool debug = false;

roscontroller::roscontroller(ros::NodeHandle& n_c, ros::NodeHandle& n_c_priv)
/*
/ roscontroller class Constructor
---------------*/
{
  ROS_INFO("Buzz_node");
  //  Obtain parameters from ros parameter server
  Rosparameters_get(n_c_priv);
  //  Initialize publishers, subscribers and client
  Initialize_pub_sub(n_c);
  //  Compile the .bzz file to .basm, .bo and .bdbg
  std::string fname = Compile_bzz(bzzfile_name);
  bcfname = fname + ".bo";
  dbgfname = fname + ".bdb";
  set_bzz_file(bzzfile_name.c_str());
  buzzuav_closures::setWPlist(bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/")) + "/");
  //  Initialize variables
  SetMode("LOITER", 0);
  armstate = 0;
  multi_msg = true;
  setpoint_counter = 0;
  fcu_timeout = TIMEOUT;
  cur_pos.longitude = 0;
  cur_pos.latitude = 0;
  cur_pos.altitude = 0;

  // set stream rate - wait for the FC to be started
  SetStreamRate(0, 10, 1);

  //  Get Home position - wait for none-zero value
  while (cur_pos.latitude == 0.0f)
  {
    ROS_INFO("Waiting for GPS. ");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  // Get Robot Id - wait for Xbee to be started
  // or else parse it from the launch file parameter
  if (xbeeplugged)
  {
    GetRobotId();
  }
  else
  {
    robot_id = strtol(robot_name.c_str() + 5, NULL, 10);
  }
}

roscontroller::~roscontroller()
/*
/ roscontroller class Destructor
---------------------*/
{
  // Destroy the BVM
  buzz_utility::buzz_script_destroy();
}

void roscontroller::GetRobotId()
/*
/ Get robot ID from the Xbee service
---------------------*/
{
  mavros_msgs::ParamGet::Request robot_id_srv_request;
  robot_id_srv_request.param_id = "id";
  mavros_msgs::ParamGet::Response robot_id_srv_response;
  while (!xbeestatus_srv.call(robot_id_srv_request, robot_id_srv_response))
  {
    ros::Duration(0.1).sleep();
    ROS_WARN("ROSBUZZ is waiting for Xbee device ID");
  }

  robot_id = robot_id_srv_response.value.integer;
}

void roscontroller::send_MPpayload()
{
  MPpayload_pub.publish(buzzuav_closures::get_status());
}

void roscontroller::RosControllerRun()
/*
/rosbuzz_node main loop method
/--------------------------------------------------*/
{
  //  Set the Buzz bytecode
  if (buzz_utility::buzz_script_set(bcfname.c_str(), dbgfname.c_str(), robot_id))
  {
    ROS_INFO("[%i] INIT DONE!!!", robot_id);
    int packet_loss_tmp, time_step = 0;
    double cur_packet_loss = 0;
    ROS_INFO("[%i] Bytecode file found and set", robot_id);
    std::string standby_bo = Compile_bzz(stand_by) + ".bo";
    //  Intialize  the  update monitor
    init_update_monitor(bcfname.c_str(), standby_bo.c_str(), dbgfname.c_str(), robot_id);
    //  set ROS loop rate
    ros::Rate loop_rate(BUZZRATE);
    //  DEBUG
    // ROS_WARN("[%i] -----------------------STARTING MAIN LOOP!", robot_id);
    while (ros::ok() && !buzz_utility::buzz_script_done())
    {
      //  Publish topics
      neighbours_pos_publisher();
      uavstate_publisher();
      grid_publisher();
      send_MPpayload();
      //  Check updater state and step code
      update_routine();
      if (time_step == BUZZRATE)
      {
        time_step = 0;
        cur_packet_loss = 1 - ((double)packet_loss_tmp / (BUZZRATE * ((int)no_of_robots - 1)));
        packet_loss_tmp = 0;
        if (cur_packet_loss < 0)
          cur_packet_loss = 0;
        else if (cur_packet_loss > 1)
          cur_packet_loss = 1;
      }
      else
      {
        packet_loss_tmp += (int)buzz_utility::get_inmsg_size();
        time_step++;
      }
      if (debug)
        ROS_WARN("CURRENT PACKET DROP : %f ", cur_packet_loss);
      //  Call Step from buzz script
      buzz_utility::buzz_script_step();
      //  Prepare messages and publish them
      prepare_msg_and_publish();
      // Call the flight controler service
      flight_controller_service_call();
      //  Set multi message available after update
      if (get_update_status())
      {
        set_read_update_status();
      }
      //  Set ROBOTS variable (swarm size)
      get_number_of_robots();
      buzz_utility::set_robot_var(no_of_robots);
      updates_set_robots(no_of_robots);
      // get_xbee_status();  // commented out because it may slow down the node too much, to be tested

      ros::spinOnce();
      loop_rate.sleep();
      // make sure to sleep for the rest of the loop time
      if (loop_rate.cycleTime() > ros::Duration(1.0 / (float)BUZZRATE))
        ROS_WARN("ROSBuzz loop missed its desired rate of %iHz... the loop actually took %.4f seconds", BUZZRATE,
                 loop_rate.cycleTime().toSec());
      // Safety land if the data coming from the flight controller are too old
      if (fcu_timeout <= 0)
        buzzuav_closures::rc_call(mavros_msgs::CommandCode::NAV_LAND);
      else
        fcu_timeout -= 1 / BUZZRATE;
      timer_step += 1;
      //  Force a refresh on neighbors array once in a while
      maintain_pos(timer_step);
      //  DEBUG
      // std::cout<< "HOME: " << home.latitude << ", " << home.longitude;
    }
  }
}

void roscontroller::Rosparameters_get(ros::NodeHandle& n_c)
/*
/ Get all required parameters from the ROS launch file
/-------------------------------------------------------*/
{
  // Obtain .bzz file name from launch file parameter
  if (n_c.getParam("bzzfile_name", bzzfile_name))
    ;
  else
  {
    ROS_ERROR("Provide a .bzz file to run in Launch file");
    system("rosnode kill rosbuzz_node");
  }
  // Obtain rc service option from parameter server
  if (n_c.getParam("rcclient", rcclient))
  {
    if (rcclient == true)
    {
      if (!n_c.getParam("rcservice_name", rcservice_name))
      {
        ROS_ERROR("Provide a name topic name for rc service in Launch file");
        system("rosnode kill rosbuzz_node");
      }
    }
    else if (rcclient == false)
    {
      ROS_INFO("RC service is disabled");
    }
  }
  else
  {
    ROS_ERROR("Provide a rc client option: yes or no in Launch file");
    system("rosnode kill rosbuzz_node");
  }
  // Obtain out payload name
  n_c.getParam("out_payload", out_payload);
  //  Obtain in payload name
  n_c.getParam("in_payload", in_payload);
  //  Obtain standby script to run during update
  n_c.getParam("stand_by", stand_by);

  if (n_c.getParam("xbee_plugged", xbeeplugged))
    ;
  else
  {
    ROS_ERROR("Provide the xbee plugged boolean in Launch file");
    system("rosnode kill rosbuzz_node");
  }
  if (!xbeeplugged)
  {
    if (n_c.getParam("name", robot_name))
      ;
    else
    {
      ROS_ERROR("Provide the xbee plugged boolean in Launch file");
      system("rosnode kill rosbuzz_node");
    }
  }
  else
    n_c.getParam("xbee_status_srv", xbeesrv_name);

  if (!n_c.getParam("capture_image_srv", capture_srv_name))
  {
    capture_srv_name = CAPTURE_SRV_DEFAULT_NAME;
  }

  GetSubscriptionParameters(n_c);
}

void roscontroller::GetSubscriptionParameters(ros::NodeHandle& node_handle)
/*
/Obtains publisher, subscriber and services from yml config file
/-----------------------------------------------------------------------------------*/
{
  std::string topic;
  if (node_handle.getParam("topics/gps", topic))
    ;
  else
  {
    ROS_ERROR("Provide a gps topic in YAML file");
    system("rosnode kill rosbuzz_node");
  }
  m_smTopic_infos.insert(pair<std::string, std::string>(topic, "sensor_msgs/NavSatFix"));
  if (node_handle.getParam("topics/localpos", topic))
    ;
  else
  {
    ROS_ERROR("Provide a localpos name in YAML file");
    system("rosnode kill rosbuzz_node");
  }
  m_smTopic_infos.insert(pair<std::string, std::string>(topic, "geometry_msgs/PoseStamped"));

  node_handle.getParam("topics/obstacles", topic);
  m_smTopic_infos.insert(pair<std::string, std::string>(topic, "sensor_msgs/LaserScan"));

  node_handle.getParam("topics/battery", topic);
  m_smTopic_infos.insert(pair<std::string, std::string>(topic, "sensor_msgs/BatteryState"));

  node_handle.getParam("topics/status", topic);
  m_smTopic_infos.insert(pair<std::string, std::string>(topic, "mavros_msgs/State"));
  node_handle.getParam("topics/estatus", topic);
  m_smTopic_infos.insert(pair<std::string, std::string>(topic, "mavros_msgs/ExtendedState"));

  node_handle.getParam("topics/altitude", topic);
  m_smTopic_infos.insert(pair<std::string, std::string>(topic, "std_msgs/Float64"));

  // Obtain required topic and service names from the parameter server
  if (node_handle.getParam("topics/fcclient", fcclient_name))
    ;
  else
  {
    ROS_ERROR("Provide a fc client name in Launch file");
    system("rosnode kill rosbuzz_node");
  }
  if (node_handle.getParam("topics/setpoint", setpoint_name))
    ;
  else
  {
    ROS_ERROR("Provide a Set Point name in Launch file");
    system("rosnode kill rosbuzz_node");
  }
  if (node_handle.getParam("topics/armclient", armclient))
    ;
  else
  {
    ROS_ERROR("Provide an arm client name in Launch file");
    system("rosnode kill rosbuzz_node");
  }
  if (node_handle.getParam("topics/modeclient", modeclient))
    ;
  else
  {
    ROS_ERROR("Provide a mode client name in Launch file");
    system("rosnode kill rosbuzz_node");
  }
  if (node_handle.getParam("topics/stream", stream_client_name))
    ;
  else
  {
    ROS_ERROR("Provide a mode client name in Launch file");
    system("rosnode kill rosbuzz_node");
  }
}

void roscontroller::Initialize_pub_sub(ros::NodeHandle& n_c)
/*
/ Create all required subscribers, publishers and ROS service clients
/-------------------------------------------------------*/
{
  // subscribers

  Subscribe(n_c);

  payload_sub = n_c.subscribe(in_payload, 5, &roscontroller::payload_obt, this);

  // publishers
  payload_pub = n_c.advertise<mavros_msgs::Mavlink>(out_payload, 5);
  MPpayload_pub = n_c.advertise<mavros_msgs::Mavlink>("fleet_status", 5);
  neigh_pos_pub = n_c.advertise<rosbuzz::neigh_pos>("neighbours_pos", 5);
  uavstate_pub = n_c.advertise<std_msgs::String>("uavstate", 5);
  grid_pub = n_c.advertise<nav_msgs::OccupancyGrid>("grid", 5);
  localsetpoint_nonraw_pub = n_c.advertise<geometry_msgs::PoseStamped>(setpoint_name, 5);

  // Service Clients
  arm_client = n_c.serviceClient<mavros_msgs::CommandBool>(armclient);
  mode_client = n_c.serviceClient<mavros_msgs::SetMode>(modeclient);
  mav_client = n_c.serviceClient<mavros_msgs::CommandLong>(fcclient_name);
  if (rcclient == true)
    service = n_c.advertiseService(rcservice_name, &roscontroller::rc_callback, this);
  ROS_INFO("Ready to receive Mav Commands from RC client");
  xbeestatus_srv = n_c.serviceClient<mavros_msgs::ParamGet>(xbeesrv_name);
  capture_srv = n_c.serviceClient<mavros_msgs::CommandBool>(capture_srv_name);
  stream_client = n_c.serviceClient<mavros_msgs::StreamRate>(stream_client_name);

  multi_msg = true;
}

void roscontroller::Subscribe(ros::NodeHandle& n_c)
/*
/ Robot independent subscribers
/--------------------------------------*/
{
  for (std::map<std::string, std::string>::iterator it = m_smTopic_infos.begin(); it != m_smTopic_infos.end(); ++it)
  {
    if (it->second == "mavros_msgs/ExtendedState")
    {
      flight_estatus_sub = n_c.subscribe(it->first, 5, &roscontroller::flight_extended_status_update, this);
    }
    else if (it->second == "mavros_msgs/State")
    {
      flight_status_sub = n_c.subscribe(it->first, 5, &roscontroller::flight_status_update, this);
    }
    else if (it->second == "sensor_msgs/BatteryState")
    {
      battery_sub = n_c.subscribe(it->first, 5, &roscontroller::battery, this);
    }
    else if (it->second == "sensor_msgs/NavSatFix")
    {
      current_position_sub = n_c.subscribe(it->first, 5, &roscontroller::global_gps_callback, this);
    }
    else if (it->second == "std_msgs/Float64")
    {
      relative_altitude_sub = n_c.subscribe(it->first, 5, &roscontroller::rel_alt_callback, this);
    }
    else if (it->second == "geometry_msgs/PoseStamped")
    {
      local_pos_sub = n_c.subscribe(it->first, 5, &roscontroller::local_pos_callback, this);
    }
    else if (it->second == "sensor_msgs/LaserScan")
    {
      obstacle_sub = n_c.subscribe(it->first, 5, &roscontroller::obstacle_dist_callback, this);
    }

    std::cout << "Subscribed to: " << it->first << endl;
  }
}

std::string roscontroller::Compile_bzz(std::string bzzfile_name)
/*
/ Create Buzz bytecode from the bzz script inputed
/-------------------------------------------------------*/
{
  // Compile the buzz code .bzz to .bo
  stringstream bzzfile_in_compile;
  std::string path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/")) + "/";
  std::string name = bzzfile_name.substr(bzzfile_name.find_last_of("/\\") + 1);
  name = name.substr(0, name.find_last_of("."));
  bzzfile_in_compile << "bzzc -I " << path << "include/";
  bzzfile_in_compile << " -b " << path << name << ".bo";
  bzzfile_in_compile << " -d " << path << name << ".bdb ";
  bzzfile_in_compile << bzzfile_name;

  ROS_WARN("Launching buzz compilation: %s", bzzfile_in_compile.str().c_str());

  system(bzzfile_in_compile.str().c_str());

  return path + name;
}

void roscontroller::neighbours_pos_publisher()
/*
/ Publish neighbours pos and id in neighbours pos topic
/----------------------------------------------------*/
{
  auto current_time = ros::Time::now();
  map<int, buzz_utility::Pos_struct>::iterator it;
  rosbuzz::neigh_pos neigh_pos_array;
  neigh_pos_array.header.frame_id = "/world";
  neigh_pos_array.header.stamp = current_time;
  for (it = raw_neighbours_pos_map.begin(); it != raw_neighbours_pos_map.end(); ++it)
  {
    sensor_msgs::NavSatFix neigh_tmp;
    neigh_tmp.header.stamp = current_time;
    neigh_tmp.header.frame_id = "/world";
    neigh_tmp.position_covariance_type = it->first;  // custom robot id storage
    neigh_tmp.latitude = (it->second).x;
    neigh_tmp.longitude = (it->second).y;
    neigh_tmp.altitude = (it->second).z;
    neigh_pos_array.pos_neigh.push_back(neigh_tmp);
    // DEBUG
    // cout<<"iterator it val: "<< it-> first << " After convertion: "
    // <<(uint8_t) buzz_utility::get_rid_uint8compac(it->first)<<endl;
    // std::cout<<"long obt"<<neigh_tmp.longitude<<endl;
  }
  neigh_pos_pub.publish(neigh_pos_array);
}

void roscontroller::uavstate_publisher()
/*
/ Publish current UAVState from Buzz script
/----------------------------------------------------*/
{
  std_msgs::String uavstate_msg;
  uavstate_msg.data = buzzuav_closures::getuavstate();
  uavstate_pub.publish(uavstate_msg);
}

void roscontroller::grid_publisher()
/*
/ Publish current Grid from Buzz script
/----------------------------------------------------*/
{
  std::map<int, std::map<int,int>> grid = buzzuav_closures::getgrid();
  std::map<int, std::map<int,int>>::iterator itr = grid.begin();
  int g_w = itr->second.size();
  int g_h = grid.size();

  if(g_w!=0 && g_h!=0) {
    // DEBUG
    //ROS_INFO("------> Publishing a grid of %i x %i", g_h, g_w);
    auto current_time = ros::Time::now();
    nav_msgs::OccupancyGrid grid_msg;
    grid_msg.header.frame_id = "/world";
    grid_msg.header.stamp = current_time;
    grid_msg.info.map_load_time = current_time;  // Same as header stamp as we do not load the map.
    grid_msg.info.resolution = 0.01;//gridMap.getResolution();
    grid_msg.info.width = g_w;
    grid_msg.info.height = g_h;
    grid_msg.info.origin.position.x = round(g_w/2.0) * grid_msg.info.resolution;
    grid_msg.info.origin.position.y = round(g_h/2.0) * grid_msg.info.resolution;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.x = 0.0;
    grid_msg.info.origin.orientation.y = 0.0;
    grid_msg.info.origin.orientation.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;
    grid_msg.data.resize(g_w * g_h);

    for (itr=grid.begin(); itr!=grid.end(); ++itr) {
      std::map<int,int>::iterator itc = itr->second.begin();
      for (itc=itr->second.begin(); itc!=itr->second.end(); ++itc) {
        grid_msg.data[(itr->first-1)*g_w+itc->first] = itc->second;
        // DEBUG
        //ROS_INFO("--------------> index: %i (%i,%i): %i", (itr->first-1)*g_w+itc->first, itr->first, itc->first, grid_msg.data[(itr->first-1)*g_w+itc->first]);
      }
    }
    grid_pub.publish(grid_msg);
  }
}


void roscontroller::Arm()
/*
/ Functions handling the local MAV ROS flight controller
/-------------------------------------------------------*/
{
  mavros_msgs::CommandBool arming_message;
  arming_message.request.value = armstate;
  if (arm_client.call(arming_message))
  {
    if (arming_message.response.success == 1)
      ROS_WARN("FC Arm Service called!");
    else
      ROS_WARN("FC Arm Service call failed!");
  }
  else
  {
    ROS_WARN("FC Arm Service call failed!");
  }
}

void roscontroller::prepare_msg_and_publish()
/*
/Prepare Buzz messages payload for each step and publish
/-----------------------------------------------------------------------------------------------------
/ Message format of payload (Each slot is uint64_t)
/
/
/|	|     |	    |
|			           |   /
/|Pos x|Pos y|Pos z|Size in Uint64_t|robot_id|Buzz_msg_size|Buzz_msg|Buzz_msgs
with size.........  |   /
/|_____|_____|_____|________________________________________________|______________________________|
/----------------------------------------------------------------------------------------------------*/
{
  // get the payload to be sent
  uint64_t* payload_out_ptr = buzz_utility::obt_out_msg();
  uint64_t position[3];
  // Appened current position to message
  double tmp[3];
  tmp[0] = cur_pos.latitude;
  tmp[1] = cur_pos.longitude;
  tmp[2] = cur_pos.altitude;
  memcpy(position, tmp, 3 * sizeof(uint64_t));
  mavros_msgs::Mavlink payload_out;
  payload_out.payload64.push_back(XBEE_MESSAGE_CONSTANT);
  payload_out.payload64.push_back(position[0]);
  payload_out.payload64.push_back(position[1]);
  payload_out.payload64.push_back(position[2]);
  //  Append Buzz message
  uint16_t* out = buzz_utility::u64_cvt_u16(payload_out_ptr[0]);
  for (int i = 0; i < out[0]; i++)
  {
    payload_out.payload64.push_back(payload_out_ptr[i]);
  }
  //  Add Robot id and message number to the published message
  if (message_number < 0)
    message_number = 0;
  else
    message_number++;
  payload_out.sysid = (uint8_t)robot_id;
  payload_out.msgid = (uint32_t)message_number;

  // publish prepared messages in respective topic
  payload_pub.publish(payload_out);
  delete[] out;
  delete[] payload_out_ptr;
  //  Check for updater message if present send
  if (is_msg_present())
  {
    uint8_t* buff_send = 0;
    uint16_t updater_msgSize = *(uint16_t*)(getupdate_out_msg_size());
    ;
    int tot = 0;
    mavros_msgs::Mavlink update_packets;
    fprintf(stdout, "Appending code into message ...\n");
    fprintf(stdout, "Sent Update packet Size: %u \n", updater_msgSize);
    // allocate mem and clear it
    buff_send = (uint8_t*)malloc(sizeof(uint16_t) + updater_msgSize);
    memset(buff_send, 0, sizeof(uint16_t) + updater_msgSize);
    // Append updater msg size
    *(uint16_t*)(buff_send + tot) = updater_msgSize;
    tot += sizeof(uint16_t);
    // Append updater msgs
    memcpy(buff_send + tot, (uint8_t*)(getupdater_out_msg()), updater_msgSize);
    tot += updater_msgSize;
    // Destroy the updater out msg queue
    destroy_out_msg_queue();
    uint16_t total_size = (ceil((float)(float)tot / (float)sizeof(uint64_t)));
    uint64_t* payload_64 = new uint64_t[total_size];
    memcpy((void*)payload_64, (void*)buff_send, total_size * sizeof(uint64_t));
    free(buff_send);
    // Send a constant number to differenciate updater msgs
    update_packets.payload64.push_back((uint64_t)UPDATER_MESSAGE_CONSTANT);
    for (int i = 0; i < total_size; i++)
    {
      update_packets.payload64.push_back(payload_64[i]);
    }
    // Add Robot id and message number to the published message
    if (message_number < 0)
      message_number = 0;
    else
      message_number++;
    update_packets.sysid = (uint8_t)robot_id;
    update_packets.msgid = (uint32_t)message_number;
    payload_pub.publish(update_packets);
    delete[] payload_64;
  }
}

void roscontroller::flight_controller_service_call()
/*
/Flight controller service call every step if there is a command set from bzz
script
/-------------------------------------------------------------------------------*/
{
  //  flight controller client call if requested from Buzz
  double* goto_pos;
  float* gimbal;
  switch (buzzuav_closures::bzz_cmd())
  {
    case buzzuav_closures::COMMAND_TAKEOFF:
      goto_pos = buzzuav_closures::getgoto();
      cmd_srv.request.param7 = goto_pos[2];
      cmd_srv.request.command = buzzuav_closures::getcmd();
      if (!armstate)
      {
        SetMode("LOITER", 0);
        armstate = 1;
        Arm();
        ros::Duration(0.5).sleep();
        // Registering HOME POINT.
        home = cur_pos;
      }
      if (current_mode != "GUIDED")
        SetMode("GUIDED", 2000);  // added for compatibility with 3DR Solo
      if (mav_client.call(cmd_srv))
      {
        ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success);
      }
      else
        ROS_ERROR("Failed to call service from flight controller");
      break;

    case buzzuav_closures::COMMAND_LAND:
      cmd_srv.request.command = buzzuav_closures::getcmd();
      if (current_mode != "LAND")
      {
        SetMode("LAND", 0);
        armstate = 0;
        Arm();
      }
      if (mav_client.call(cmd_srv))
      {
        ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success);
      }
      else
      {
        ROS_ERROR("Failed to call service from flight controller");
      }
      armstate = 0;
      break;

    case buzzuav_closures::COMMAND_GOHOME:  // TODO: NOT FULLY IMPLEMENTED/TESTED !!!
      cmd_srv.request.param5 = home.latitude;
      cmd_srv.request.param6 = home.longitude;
      cmd_srv.request.param7 = home.altitude;
      cmd_srv.request.command = buzzuav_closures::getcmd();
      if (mav_client.call(cmd_srv))
      {
        ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success);
      }
      else
        ROS_ERROR("Failed to call service from flight controller");
      break;

    case buzzuav_closures::COMMAND_GOTO:  // TOOD: NOT FULLY IMPLEMENTED/TESTED !!!
      goto_pos = buzzuav_closures::getgoto();
      cmd_srv.request.param5 = goto_pos[0];
      cmd_srv.request.param6 = goto_pos[1];
      cmd_srv.request.param7 = goto_pos[2];
      cmd_srv.request.command = buzzuav_closures::getcmd();
      if (mav_client.call(cmd_srv))
      {
        ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success);
      }
      else
        ROS_ERROR("Failed to call service from flight controller");
      cmd_srv.request.command = mavros_msgs::CommandCode::MISSION_START;
      if (mav_client.call(cmd_srv))
      {
        ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success);
      }
      else
        ROS_ERROR("Failed to call service from flight controller");
      break;

    case buzzuav_closures::COMMAND_ARM:
      if (!armstate)
      {
        SetMode("LOITER", 0);
        armstate = 1;
        Arm();
      }
      break;

    case buzzuav_closures::COMMAND_DISARM:
      if (armstate)
      {
        armstate = 0;
        SetMode("LOITER", 0);
        Arm();
      }
      break;

    case buzzuav_closures::COMMAND_MOVETO:
      goto_pos = buzzuav_closures::getgoto();
      roscontroller::SetLocalPosition(goto_pos[0], goto_pos[1], goto_pos[2], goto_pos[3]);
      break;

    case buzzuav_closures::COMMAND_GIMBAL:
      gimbal = buzzuav_closures::getgimbal();
      cmd_srv.request.param1 = gimbal[0];
      cmd_srv.request.param2 = gimbal[1];
      cmd_srv.request.param3 = gimbal[2];
      cmd_srv.request.param4 = gimbal[3];
      cmd_srv.request.command = mavros_msgs::CommandCode::DO_MOUNT_CONTROL;
      if (mav_client.call(cmd_srv))
      {
        ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success);
      }
      else
        ROS_ERROR("Failed to call service from flight controller");
      break;

    case buzzuav_closures::COMMAND_PICTURE:
      ROS_INFO("TAKING A PICTURE HERE!! --------------");
      mavros_msgs::CommandBool capture_command;
      if (capture_srv.call(capture_command))
      {
        ROS_INFO("Reply: %ld", (long int)capture_command.response.success);
      }
      else
        ROS_ERROR("Failed to call service from camera streamer");
      break;
  }
}

void roscontroller::maintain_pos(int tim_step)
/*
/Refresh neighbours Position for every ten step
/---------------------------------------------*/
{
  if (timer_step >= BUZZRATE)
  {
    neighbours_pos_map.clear();
    // raw_neighbours_pos_map.clear(); // TODO: currently not a problem, but
    // have to clear !
    timer_step = 0;
  }
}

void roscontroller::neighbours_pos_put(int id, buzz_utility::Pos_struct pos_arr)
/*
/Puts neighbours position into local struct storage that is cleared every 10
step
/---------------------------------------------------------------------------------*/
{
  map<int, buzz_utility::Pos_struct>::iterator it = neighbours_pos_map.find(id);
  if (it != neighbours_pos_map.end())
    neighbours_pos_map.erase(it);
  neighbours_pos_map.insert(make_pair(id, pos_arr));
}

void roscontroller::raw_neighbours_pos_put(int id, buzz_utility::Pos_struct pos_arr)
/*
/Puts raw neighbours position into local storage for neighbours pos publisher
/-----------------------------------------------------------------------------------*/
{
  map<int, buzz_utility::Pos_struct>::iterator it = raw_neighbours_pos_map.find(id);
  if (it != raw_neighbours_pos_map.end())
    raw_neighbours_pos_map.erase(it);
  raw_neighbours_pos_map.insert(make_pair(id, pos_arr));
}

void roscontroller::set_cur_pos(double latitude, double longitude, double altitude)
/*
/Set the current position of the robot callback
/--------------------------------------------------------*/
{
  cur_pos.latitude = latitude;
  cur_pos.longitude = longitude;
  cur_pos.altitude = altitude;
}

float roscontroller::constrainAngle(float x)
/*
/ Wrap the angle between -pi, pi
----------------------------------------------------------- */
{
  x = fmod(x, 2 * M_PI);
  if (x < 0.0)
    x += 2 * M_PI;
  return x;
}

void roscontroller::gps_rb(POSE nei_pos, double out[])
/*
/ Compute Range and Bearing of a neighbor in a local reference frame
/ from GPS coordinates
----------------------------------------------------------- */
{
  float ned_x = 0.0, ned_y = 0.0;
  gps_ned_cur(ned_x, ned_y, nei_pos);
  out[0] = sqrt(ned_x * ned_x + ned_y * ned_y);
  // out[0] = std::floor(out[0] * 1000000) / 1000000;
  out[1] = atan2(ned_y, ned_x);
  out[1] = constrainAngle(atan2(ned_y, ned_x));
  // out[1] = std::floor(out[1] * 1000000) / 1000000;
  out[2] = 0.0;
}

void roscontroller::gps_ned_cur(float& ned_x, float& ned_y, POSE t)
/*
/ Get GPS from NED and a reference GPS point (struct input)
----------------------------------------------------------- */
{
  gps_convert_ned(ned_x, ned_y, t.longitude, t.latitude, cur_pos.longitude, cur_pos.latitude);
}

void roscontroller::gps_convert_ned(float& ned_x, float& ned_y, double gps_t_lon, double gps_t_lat, double gps_r_lon,
                                    double gps_r_lat)
/*
/ Get GPS from NED and a reference GPS point
----------------------------------------------------------- */
{
  double d_lon = gps_t_lon - gps_r_lon;
  double d_lat = gps_t_lat - gps_r_lat;
  ned_x = DEG2RAD(d_lat) * EARTH_RADIUS;
  ned_y = DEG2RAD(d_lon) * EARTH_RADIUS * cos(DEG2RAD(gps_t_lat));
};

void roscontroller::battery(const sensor_msgs::BatteryState::ConstPtr& msg)
/*
/ Update battery status into BVM from subscriber
/------------------------------------------------------*/
{
  buzzuav_closures::set_battery(msg->voltage, msg->current, msg->percentage*100.0);
  //  DEBUG
  // ROS_INFO("voltage : %d  current : %d  remaining : %d",msg->voltage,
  // msg->current, msg ->remaining);
}

void roscontroller::flight_status_update(const mavros_msgs::State::ConstPtr& msg)
/*
/Update flight extended status into BVM from subscriber for solos
/---------------------------------------------------------------------*/
{
  // http://wiki.ros.org/mavros/CustomModes
  std::cout << "Message: " << msg->mode << std::endl;
  if (msg->mode == "GUIDED")
    buzzuav_closures::flight_status_update(2);
  else if (msg->mode == "LAND")
    buzzuav_closures::flight_status_update(1);
  else
    buzzuav_closures::flight_status_update(7);  // default to fit mavros::extended_state
}

void roscontroller::flight_extended_status_update(const mavros_msgs::ExtendedState::ConstPtr& msg)
/*
/Update flight extended status into BVM from subscriber
------------------------------------------------------------*/
{
  buzzuav_closures::flight_status_update(msg->landed_state);
}

void roscontroller::global_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
/*
/ Update current GPS position into BVM from subscriber
/-------------------------------------------------------------*/
{
  // reset timeout counter
  fcu_timeout = TIMEOUT;
  set_cur_pos(msg->latitude, msg->longitude, cur_pos.z);
  buzzuav_closures::set_currentpos(msg->latitude, msg->longitude, cur_pos.z, cur_pos.yaw);
}

void roscontroller::local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
/*
/ Update current position for flight controller NED offset
/-------------------------------------------------------------*/
{
  cur_pos.x = msg->pose.position.x;
  cur_pos.y = msg->pose.position.y;
  //  cur_pos.z = pose->pose.position.z; // Using relative altitude topic instead
  tf::Quaternion q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  cur_pos.yaw = yaw;
}

void roscontroller::rel_alt_callback(const std_msgs::Float64::ConstPtr& msg)
/*
/ Update altitude into BVM from subscriber
/-------------------------------------------------------------*/
{
  //  DEBUG
  // ROS_INFO("Altitude in: %f", msg->data);
  cur_pos.z = (double)msg->data;
}

void roscontroller::obstacle_dist_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
/*
/Set obstacle Obstacle distance table into BVM from subscriber
/-------------------------------------------------------------*/
{
  float obst[5];
  for (int i = 0; i < 5; i++)
    obst[i] = msg->ranges[i];
  buzzuav_closures::set_obstacle_dist(obst);
}

void roscontroller::SetLocalPosition(float x, float y, float z, float yaw)
/*
/ Publisher to send the flight controller navigation commands in local coordinates
/-------------------------------------------------------------*/
{
  // http://docs.ros.org/kinetic/api/mavros_msgs/html/msg/PositionTarget.html
  // http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-local-ned

  geometry_msgs::PoseStamped moveMsg;
  moveMsg.header.stamp = ros::Time::now();
  moveMsg.header.seq = setpoint_counter++;
  moveMsg.header.frame_id = 1;

  //  DEBUG
  // ROS_INFO("Lp: %.3f, %.3f - Del: %.3f, %.3f, %.3f", cur_pos.x, cur_pos.y, x, y, yaw);
  moveMsg.pose.position.x = cur_pos.x + y;
  moveMsg.pose.position.y = cur_pos.y + x;
  moveMsg.pose.position.z = z;

  tf::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  moveMsg.pose.orientation.x = q[0];
  moveMsg.pose.orientation.y = q[1];
  moveMsg.pose.orientation.z = q[2];
  moveMsg.pose.orientation.w = q[3];

  // To prevent drifting from stable position, uncomment
  // if(fabs(x)>0.005 || fabs(y)>0.005) {
  localsetpoint_nonraw_pub.publish(moveMsg);
  //        }
}

void roscontroller::SetMode(std::string mode, int delay_miliseconds)
/*
/ Use setmode service of the flight controller
/-------------------------------------------------------------*/
{
  // wait if necessary
  if (delay_miliseconds != 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_miliseconds));
  }
  // set mode
  mavros_msgs::SetMode set_mode_message;
  set_mode_message.request.base_mode = 0;
  set_mode_message.request.custom_mode = mode;
  current_mode = mode;
  if (mode_client.call(set_mode_message))
  {
    ;  // DEBUG
    // ROS_INFO("Set Mode Service call successful!");
  }
  else
  {
    ROS_INFO("Set Mode Service call failed!");
  }
}

void roscontroller::SetStreamRate(int id, int rate, int on_off)
/*
/ Set the streamrate for mavros publications
/-------------------------------------------------------------*/
{
  mavros_msgs::StreamRate message;
  message.request.stream_id = id;
  message.request.message_rate = rate;
  message.request.on_off = on_off;

  while (!stream_client.call(message))
  {
    ROS_INFO("Set stream rate call failed!, trying again...");
    ros::Duration(0.1).sleep();
  }
  //  DEBUG
  // ROS_INFO("Set stream rate call successful");
}

void roscontroller::payload_obt(const mavros_msgs::Mavlink::ConstPtr& msg)
/*
/Push payload into BVM FIFO
/----------------------------------------------------------------------------------------
/ Message format of payload (Each slot is uint64_t)
/ _________________________________________________________________________________________________
/|	|     |	    |						     |
 * |
/|Pos x|Pos y|Pos z|Size in Uint64_t|robot_id|Buzz_msg_size|Buzz_msg|Buzz_msgs
 * with size.........  |
/|_____|_____|_____|________________________________________________|______________________________|

-----------------------------------------------------------------------------------------------------*/
{
  // Check for Updater message, if updater message push it into updater FIFO
  if ((uint64_t)msg->payload64[0] == (uint64_t)UPDATER_MESSAGE_CONSTANT)
  {
    uint16_t obt_msg_size = sizeof(uint64_t) * (msg->payload64.size());
    uint64_t message_obt[obt_msg_size];
    //  Go throught the obtained payload
    for (int i = 0; i < (int)msg->payload64.size(); i++)
    {
      message_obt[i] = (uint64_t)msg->payload64[i];
    }

    uint8_t* pl = (uint8_t*)malloc(obt_msg_size);
    memset(pl, 0, obt_msg_size);
    //  Copy packet into temporary buffer neglecting update constant
    memcpy((void*)pl, (void*)(message_obt + 1), obt_msg_size);
    uint16_t unMsgSize = *(uint16_t*)(pl);
    fprintf(stdout, "Update packet received, read msg size : %u \n", unMsgSize);
    if (unMsgSize > 0)
    {
      code_message_inqueue_append((uint8_t*)(pl + sizeof(uint16_t)), unMsgSize);
      code_message_inqueue_process();
    }
    free(pl);
  }
  //  BVM FIFO message
  else if (msg->payload64[0] == (uint64_t)XBEE_MESSAGE_CONSTANT)
  {
    uint64_t message_obt[msg->payload64.size() - 1];
    //  Go throught the obtained payload
    for (int i = 1; i < (int)msg->payload64.size(); i++)
    {
      message_obt[i - 1] = (uint64_t)msg->payload64[i];
    }
    // Extract neighbours position from payload
    double neighbours_pos_payload[3];
    memcpy(neighbours_pos_payload, message_obt, 3 * sizeof(uint64_t));
    buzz_utility::Pos_struct raw_neigh_pos(neighbours_pos_payload[0], neighbours_pos_payload[1],
                                           neighbours_pos_payload[2]);
    POSE nei_pos;
    nei_pos.latitude = neighbours_pos_payload[0];
    nei_pos.longitude = neighbours_pos_payload[1];
    nei_pos.altitude = neighbours_pos_payload[2];
    double cvt_neighbours_pos_payload[3];
    gps_rb(nei_pos, cvt_neighbours_pos_payload);
    //  Extract robot id of the neighbour
    uint16_t* out = buzz_utility::u64_cvt_u16((uint64_t) * (message_obt + 3));
    if (debug)
      ROS_WARN("RAB of %i: %f, %f", (int)out[1], cvt_neighbours_pos_payload[0], cvt_neighbours_pos_payload[1]);
    //  Pass neighbour position to local maintaner
    buzz_utility::Pos_struct n_pos(cvt_neighbours_pos_payload[0], cvt_neighbours_pos_payload[1],
                                   cvt_neighbours_pos_payload[2]);
    //  Put RID and pos
    raw_neighbours_pos_put((int)out[1], raw_neigh_pos);
    //  TODO: remove roscontroller local map array for neighbors
    neighbours_pos_put((int)out[1], n_pos);
    buzzuav_closures::neighbour_pos_callback((int)out[1], n_pos.x, n_pos.y, n_pos.z);
    delete[] out;
    buzz_utility::in_msg_append((message_obt + 3));
  }
}

bool roscontroller::rc_callback(mavros_msgs::CommandLong::Request& req, mavros_msgs::CommandLong::Response& res)
/*
/ Catch the ROS service call from a custom remote controller (Mission Planner)
/ and send the requested commands to Buzz
----------------------------------------------------------- */
{
  int rc_cmd;
  switch (req.command)
  {
    case mavros_msgs::CommandCode::NAV_TAKEOFF:
      ROS_INFO("RC_call: TAKE OFF!!!!");
      rc_cmd = mavros_msgs::CommandCode::NAV_TAKEOFF;
      buzzuav_closures::rc_call(rc_cmd);
      res.success = true;
      break;
    case mavros_msgs::CommandCode::NAV_LAND:
      ROS_INFO("RC_Call: LAND!!!!");
      rc_cmd = mavros_msgs::CommandCode::NAV_LAND;
      buzzuav_closures::rc_call(rc_cmd);
      res.success = true;
      break;
    case mavros_msgs::CommandCode::COMPONENT_ARM_DISARM:
      rc_cmd = mavros_msgs::CommandCode::COMPONENT_ARM_DISARM;
      armstate = req.param1;
      if (armstate)
      {
        ROS_INFO("RC_Call: ARM!!!!");
        buzzuav_closures::rc_call(rc_cmd);
        res.success = true;
      }
      else
      {
        ROS_INFO("RC_Call: DISARM!!!!");
        buzzuav_closures::rc_call(rc_cmd + 1);
        res.success = true;
      }
      break;
    case mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH:
      ROS_INFO("RC_Call: GO HOME!!!!");
      rc_cmd = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
      buzzuav_closures::rc_call(rc_cmd);
      res.success = true;
      break;
    case mavros_msgs::CommandCode::NAV_WAYPOINT:
      ROS_INFO("RC_Call: GO TO!!!! ");
      buzzuav_closures::rc_set_goto(req.param1, req.param5, req.param6, req.param7);
      rc_cmd = mavros_msgs::CommandCode::NAV_WAYPOINT;
      buzzuav_closures::rc_call(rc_cmd);
      res.success = true;
      break;
    case mavros_msgs::CommandCode::DO_MOUNT_CONTROL:
      ROS_INFO("RC_Call: Gimbal!!!! ");
      buzzuav_closures::rc_set_gimbal(req.param1, req.param2, req.param3, req.param4, req.param5);
      rc_cmd = mavros_msgs::CommandCode::DO_MOUNT_CONTROL;
      buzzuav_closures::rc_call(rc_cmd);
      res.success = true;
      break;
    case CMD_REQUEST_UPDATE:
      rc_cmd = CMD_REQUEST_UPDATE;
      buzzuav_closures::rc_call(rc_cmd);
      res.success = true;
      break;
    default:
      buzzuav_closures::rc_call(req.command);
      ROS_INFO("----> Received unregistered command: ", req.command);
      res.success = true;
      break;
  }
  return true;
}

void roscontroller::get_number_of_robots()
/*
/ Garbage collector for the number of robots in the swarm
--------------------------------------------------------------------------*/
{
  int cur_robots = (int)buzzdict_size(buzz_utility::get_vm()->swarmmembers) + 1;
  if (no_of_robots == 0)
  {
    no_of_robots = cur_robots;
  }
  else
  {
    if (no_of_robots != cur_robots && no_cnt == 0)
    {
      no_cnt++;
      old_val = cur_robots;
    }
    else if (no_cnt != 0 && old_val == cur_robots)
    {
      no_cnt++;
      if (no_cnt >= 150 || cur_robots > no_of_robots)
      {
        no_of_robots = cur_robots;
        no_cnt = 0;
      }
    }
    else
    {
      no_cnt = 0;
    }
  }
}

/*
/ Set of functions to grab network quality data from Zbee service
--------------------------------------------------------------------------*/
bool roscontroller::GetDequeFull(bool& result)
{
  mavros_msgs::ParamGet::Request srv_request;
  srv_request.param_id = "deque_full";
  mavros_msgs::ParamGet::Response srv_response;

  if (!xbeestatus_srv.call(srv_request, srv_response))
  {
    return false;
  }

  result = (srv_response.value.integer == 1) ? true : false;
  return srv_response.success;
}

bool roscontroller::GetRssi(float& result)
{
  mavros_msgs::ParamGet::Request srv_request;
  srv_request.param_id = "rssi";
  mavros_msgs::ParamGet::Response srv_response;

  if (!xbeestatus_srv.call(srv_request, srv_response))
  {
    return false;
  }

  result = srv_response.value.real;
  return srv_response.success;
}

bool roscontroller::TriggerAPIRssi(const uint8_t short_id)
{
  mavros_msgs::ParamGet::Request srv_request;
  if (short_id == 0xFF)
  {
    srv_request.param_id = "trig_rssi_api_avg";
  }
  else
  {
    srv_request.param_id = "trig_rssi_api_" + std::to_string(short_id);
  }
  mavros_msgs::ParamGet::Response srv_response;
  if (!xbeestatus_srv.call(srv_request, srv_response))
  {
    return false;
  }

  return srv_response.success;
}

bool roscontroller::GetAPIRssi(const uint8_t short_id, float& result)
{
  mavros_msgs::ParamGet::Request srv_request;
  if (short_id == 0xFF)
  {
    srv_request.param_id = "get_rssi_api_avg";
  }
  else
  {
    srv_request.param_id = "get_rssi_api_" + std::to_string(short_id);
  }
  mavros_msgs::ParamGet::Response srv_response;
  if (!xbeestatus_srv.call(srv_request, srv_response))
  {
    return false;
  }

  result = srv_response.value.real;
  return srv_response.success;
}

bool roscontroller::GetRawPacketLoss(const uint8_t short_id, float& result)
{
  mavros_msgs::ParamGet::Request srv_request;
  if (short_id == 0xFF)
  {
    srv_request.param_id = "pl_raw_avg";
  }
  else
  {
    srv_request.param_id = "pl_raw_" + std::to_string(short_id);
  }
  mavros_msgs::ParamGet::Response srv_response;
  if (!xbeestatus_srv.call(srv_request, srv_response))
  {
    return false;
  }

  result = srv_response.value.real;
  return srv_response.success;
}

bool roscontroller::GetFilteredPacketLoss(const uint8_t short_id, float& result)
{
  mavros_msgs::ParamGet::Request srv_request;
  if (short_id == 0xFF)
  {
    srv_request.param_id = "pl_filtered_avg";
  }
  else
  {
    srv_request.param_id = "pl_filtered_" + std::to_string(short_id);
  }
  mavros_msgs::ParamGet::Response srv_response;
  if (!xbeestatus_srv.call(srv_request, srv_response))
  {
    return false;
  }

  result = srv_response.value.real;
  return srv_response.success;
}

void roscontroller::get_xbee_status()
/*
 * Call all the xbee node services and update the xbee status
 ------------------------------------------------------------------ */
{
  bool result_bool;
  float result_float;
  const uint8_t all_ids = 0xFF;
  if (GetDequeFull(result_bool))
  {
    buzzuav_closures::set_deque_full(result_bool);
  }
  if (GetRssi(result_float))
  {
    buzzuav_closures::set_rssi(result_float);
  }
  if (GetRawPacketLoss(all_ids, result_float))
  {
    buzzuav_closures::set_raw_packet_loss(result_float);
  }
  if (GetFilteredPacketLoss(all_ids, result_float))
  {
    buzzuav_closures::set_filtered_packet_loss(result_float);
  }
  // This part needs testing since it can overload the xbee module
  /*
   * if(GetAPIRssi(all_ids, result_float))
   * {
   * buzzuav_closures::set_api_rssi(result_float);
   * }
   * TriggerAPIRssi(all_ids);
   */
}
}

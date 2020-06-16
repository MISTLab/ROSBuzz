/** @file      buzz_utility.cpp
 *  @version   1.0
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS.
 *  @author    Vivek Shankar Varadharajan and David St-Onge
 *  @copyright 2016 MistLab. All rights reserved.
 */

#include <rosbuzz/buzz_utility.h>

namespace buzz_utility
{
/****************************************/
/****************************************/

static buzzvm_t VM = 0;
static char* BO_FNAME = 0;
static uint8_t* BO_BUF = 0;
static buzzdebug_t DBG_INFO = 0;
static uint32_t MAX_MSG_SIZE = 190;  // 250;  // Maximum Msg size for sending update packets
static uint8_t Robot_id = 0;
static std::vector<uint8_t*> IN_MSG;
std::map<int, Pos_struct> users_map;

/****************************************/

uint16_t* u64_cvt_u16(uint64_t u64)
/*
/ Deserializes uint64_t into 4 uint16_t, freeing out is left to the user
------------------------------------------------------------------------*/
{
  uint16_t* out = new uint16_t[4];
  uint32_t int32_1 = u64 & 0xFFFFFFFF;
  uint32_t int32_2 = (u64 & 0xFFFFFFFF00000000) >> 32;
  out[0] = int32_1 & 0xFFFF;
  out[1] = (int32_1 & (0xFFFF0000)) >> 16;
  out[2] = int32_2 & 0xFFFF;
  out[3] = (int32_2 & (0xFFFF0000)) >> 16;
  //  DEBUG
  // cout << " values " <<out[0] <<"  "<<out[1] <<"  "<<out[2] <<"  "<<out[3] <<"  ";
  return out;
}

int get_robotid()
/*
/ return this robot ID
------------------------------------------------------------------------*/
{
  return Robot_id;
}

void in_msg_append(uint64_t* payload)
/*
/  Appends obtained messages to buzz in message Queue
---------------------------------------------------------------------
/ Message format of payload (Each slot is uint64_t)
/ _______________________________________________________________________________________________________________
/|					        		             |			                  |
/|Size in Uint64_t(but size is Uint16_t)|robot_id|Update msg size|Update msg|Update msgs+Buzz_msgs with size.....|
/|__________________________________________________________________________|____________________________________|
---------------------------------------------------------------------------------------------------------------------*/
{
  // Go through messages and append them to the vector
  uint16_t* data = u64_cvt_u16((uint64_t)payload[0]);
  //  Size is at first 2 bytes
  uint16_t size = data[0] * sizeof(uint64_t);
  delete[] data;
  uint8_t* pl = (uint8_t*)malloc(size);
  // Copy packet into temporary buffer
  memcpy(pl, payload, size);
  IN_MSG.push_back(pl);
}

void in_message_process()
/*
/  Process msgs in
---------------------------------------------------------------------------------------------------------------------*/
{
  while (!IN_MSG.empty())
  {
    // Go through messages and append them to the FIFO
    uint8_t* first_INmsg = (uint8_t*)IN_MSG.front();
    size_t tot = 0;
    //  Size is at first 2 bytes
    uint16_t size = (*(uint16_t*)first_INmsg) * sizeof(uint64_t);
    tot += sizeof(uint16_t);
    //  Decode neighbor Id
    uint16_t neigh_id = *(uint16_t*)(first_INmsg + tot);
    tot += sizeof(uint16_t);
    // Go through the messages until there's nothing else to read
    uint16_t unMsgSize = 0;
    //  Obtain Buzz messages push it into queue
    do
    {
      // Get payload size
      unMsgSize = *(uint16_t*)(first_INmsg + tot);
      tot += sizeof(uint16_t);
      // Append message to the Buzz input message queue
      if (unMsgSize > 0 && unMsgSize <= size - tot)
      {
        buzzinmsg_queue_append(VM, neigh_id, buzzmsg_payload_frombuffer(first_INmsg + tot, unMsgSize));
        tot += unMsgSize;
      }
    } while (size - tot > sizeof(uint16_t) && unMsgSize > 0);
    free(first_INmsg);
    IN_MSG.erase(IN_MSG.begin());
  }
  // Process messages VM call*
  buzzvm_process_inmsgs(VM);
}

uint64_t* obt_out_msg()
/*
/ Obtains messages from buzz out message Queue
-------------------------------------------------*/
{
  // Process out messages
  buzzvm_process_outmsgs(VM);
  uint8_t* buff_send = (uint8_t*)malloc(MAX_MSG_SIZE);
  memset(buff_send, 0, MAX_MSG_SIZE);
  //  Taking into consideration the sizes included at the end
  ssize_t tot = sizeof(uint16_t);
  // Send robot id
  *(uint16_t*)(buff_send + tot) = (uint16_t)VM->robot;
  tot += sizeof(uint16_t);
  // Send messages from FIFO
  do
  {
    // Are there more messages?
    if (buzzoutmsg_queue_isempty(VM))
      break;
    // Get first message
    buzzmsg_payload_t m = buzzoutmsg_queue_first(VM);
    // Make sure the next message makes the data buffer with buzz messages to be less than MAX SIZE Bytes
    //  DEBUG
    // ROS_INFO("read size : %i", (int)(tot + buzzmsg_payload_size(m) + sizeof(uint16_t)));
    if ((uint32_t)(tot + buzzmsg_payload_size(m) + sizeof(uint16_t)) > MAX_MSG_SIZE)
    {
      buzzmsg_payload_destroy(&m);
      break;
    }

    // Add message length to data buffer
    *(uint16_t*)(buff_send + tot) = (uint16_t)buzzmsg_payload_size(m);
    tot += sizeof(uint16_t);

    // Add payload to data buffer
    memcpy(buff_send + tot, m->data, buzzmsg_payload_size(m));
    tot += buzzmsg_payload_size(m);

    // Get rid of message
    buzzoutmsg_queue_next(VM);
    buzzmsg_payload_destroy(&m);
  } while (1);

  uint16_t total_size = (ceil((float)tot / (float)sizeof(uint64_t)));
  *(uint16_t*)buff_send = (uint16_t)total_size;

  uint64_t* payload_64 = new uint64_t[total_size];

  memcpy((void*)payload_64, (void*)buff_send, total_size * sizeof(uint64_t));
  free(buff_send);
  //  DEBUG
  // for(int i=0;i<total_size;i++){
  // cout<<" payload from out msg  "<<*(payload_64+i)<<endl;
  //}
  // Send message
  return payload_64;
}

static const char* buzz_error_info()
/*
/ Buzz script not able to load
---------------------------------*/
{
  buzzdebug_entry_t dbg = *buzzdebug_info_get_fromoffset(DBG_INFO, &VM->pc);
  char* msg;
  if (dbg != NULL)
  {
    asprintf(&msg, "%s: execution terminated abnormally at %s:%" PRIu64 ":%" PRIu64 " : %s\n\n", BO_FNAME, dbg->fname,
             dbg->line, dbg->col, VM->errormsg);
  }
  else
  {
    asprintf(&msg, "%s: execution terminated abnormally at bytecode offset %d: %s\n\n", BO_FNAME, VM->pc, VM->errormsg);
  }
  return msg;
}

static int buzz_register_hooks()
/*
/ Buzz hooks that can be used inside .bzz file
------------------------------------------------*/
{
  buzzvm_pushs(VM, buzzvm_string_register(VM, "print", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzros_print));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "log", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzros_print));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "debug", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzros_print));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "goto_abs", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_moveto));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "storegoal", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_storegoal));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "setgimbal", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_setgimbal));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "takepicture", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_takepicture));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "arm", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_arm));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "disarm", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_disarm));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "uav_takeoff", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_takeoff));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "gohome", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_gohome));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "uav_land", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_land));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "add_targetrb", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_addtargetRB));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "add_neighborStatus", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_addNeiStatus));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "export_map", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzz_exportmap));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "reset_rc", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_resetrc));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "voronoi", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::voronoi_center));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "geofence", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzuav_geofence));
  buzzvm_gstore(VM);

  return VM->state;
}

static int testing_buzz_register_hooks()
/*
/ Register dummy Buzz hooks for test during update
---------------------------------------------------*/
{
  buzzvm_pushs(VM, buzzvm_string_register(VM, "print", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzros_print));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "log", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzros_print));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "debug", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::buzzros_print));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "goto_abs", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "storegoal", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "setgimbal", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "takepicture", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "arm", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "disarm", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "uav_takeoff", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "gohome", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "uav_land", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "add_targetrb", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "add_neighborStatus", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "export_map", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);
  buzzvm_pushs(VM, buzzvm_string_register(VM, "reset_rc", 1));
  buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_closures::dummy_closure));
  buzzvm_gstore(VM);

  return VM->state;
}

int buzz_script_set(const char* bo_filename, const char* bdbg_filename, int robot_id)
/*
/ Sets the .bzz and .bdbg file
---------------------------------*/
{
  ROS_INFO(" Robot ID: %i", robot_id);
  Robot_id = robot_id;
  // Read bytecode from file and fill the bo buffer
  FILE* fd = fopen(bo_filename, "rb");
  if (!fd)
  {
    perror(bo_filename);
    return 0;
  }
  fseek(fd, 0, SEEK_END);
  size_t bcode_size = ftell(fd);
  rewind(fd);
  BO_BUF = (uint8_t*)malloc(bcode_size);
  if (fread(BO_BUF, 1, bcode_size, fd) < bcode_size)
  {
    perror(bo_filename);
    buzzvm_destroy(&VM);
    buzzdebug_destroy(&DBG_INFO);
    fclose(fd);
    return 0;
  }
  fclose(fd);

  // Save bytecode file name
  BO_FNAME = strdup(bo_filename);

  return buzz_update_set(BO_BUF, bdbg_filename, bcode_size);
}

int buzz_update_set(uint8_t* UP_BO_BUF, const char* bdbg_filename, size_t bcode_size)
/*
/ Sets a new update
-----------------------*/
{
  // Reset the Buzz VM
  if (VM)
    buzzvm_destroy(&VM);
  VM = buzzvm_new(Robot_id);
  // Get rid of debug info
  if (DBG_INFO)
    buzzdebug_destroy(&DBG_INFO);
  DBG_INFO = buzzdebug_new();

  // Read debug information
  if (!buzzdebug_fromfile(DBG_INFO, bdbg_filename))
  {
    buzzvm_destroy(&VM);
    buzzdebug_destroy(&DBG_INFO);
    perror(bdbg_filename);
    return 0;
  }
  // Set byte code
  if (buzzvm_set_bcode(VM, UP_BO_BUF, bcode_size) != BUZZVM_STATE_READY)
  {
    buzzvm_destroy(&VM);
    buzzdebug_destroy(&DBG_INFO);
    ROS_ERROR("[%i] %s: Error loading Buzz bytecode (update)", Robot_id);
    return 0;
  }
  // Register hook functions
  if (buzz_register_hooks() != BUZZVM_STATE_READY)
  {
    buzzvm_destroy(&VM);
    buzzdebug_destroy(&DBG_INFO);
    ROS_ERROR("[%i] Error registering hooks (update)", Robot_id);
    return 0;
  }

  // Initialize UAVSTATE variable
  buzzvm_pushs(VM, buzzvm_string_register(VM, "BMVSTATE", 1));
  buzzvm_pushs(VM, buzzvm_string_register(VM, "TURNEDOFF", 1));
  buzzvm_gstore(VM);

  // Execute the global part of the script
  if (buzzvm_execute_script(VM) != BUZZVM_STATE_DONE)
  {
    ROS_ERROR("Error executing global part, VM state : %i", VM->state);
    return 0;
  }
  // Call the Init() function
  if (buzzvm_function_call(VM, "init", 0) != BUZZVM_STATE_READY)
  {
    ROS_ERROR("Error in  calling init, VM state : %i", VM->state);
    return 0;
  }
  // Pop the init return value
  buzzvm_pop(VM);
  // All OK
  return 1;
}

int buzz_update_init_test(uint8_t* UP_BO_BUF, const char* bdbg_filename, size_t bcode_size)
/*
/ Performs a initialization test
-----------------------------------*/
{
  // Reset the Buzz VM
  if (VM)
    buzzvm_destroy(&VM);
  VM = buzzvm_new(Robot_id);
  // Get rid of debug info
  if (DBG_INFO)
    buzzdebug_destroy(&DBG_INFO);
  DBG_INFO = buzzdebug_new();

  // Read debug information
  if (!buzzdebug_fromfile(DBG_INFO, bdbg_filename))
  {
    buzzvm_destroy(&VM);
    buzzdebug_destroy(&DBG_INFO);
    perror(bdbg_filename);
    return 0;
  }
  // Set byte code
  if (buzzvm_set_bcode(VM, UP_BO_BUF, bcode_size) != BUZZVM_STATE_READY)
  {
    buzzvm_destroy(&VM);
    buzzdebug_destroy(&DBG_INFO);
    ROS_ERROR("[%i] %s: Error loading Buzz bytecode (update init)", Robot_id);
    return 0;
  }
  // Register hook functions
  if (testing_buzz_register_hooks() != BUZZVM_STATE_READY)
  {
    buzzvm_destroy(&VM);
    buzzdebug_destroy(&DBG_INFO);
    ROS_ERROR("[%i] Error registering hooks (update init)", Robot_id);
    return 0;
  }

  // Initialize UAVSTATE variable
  buzzvm_pushs(VM, buzzvm_string_register(VM, "BVMSTATE", 1));
  buzzvm_pushs(VM, buzzvm_string_register(VM, "TURNEDOFF", 1));
  buzzvm_gstore(VM);

  // Execute the global part of the script
  if (buzzvm_execute_script(VM) != BUZZVM_STATE_DONE)
  {
    ROS_ERROR("Error executing global part, VM state : %i", VM->state);
    return 0;
  }
  // Call the Init() function
  if (buzzvm_function_call(VM, "init", 0) != BUZZVM_STATE_READY)
  {
    ROS_ERROR("Error in  calling init, VM state : %i", VM->state);
    return 0;
  }
  // Pop init return value
  buzzvm_pop(VM);
  // All OK
  return 1;
}

struct buzzswarm_elem_s
/*
/ Swarm struct
----------------*/
{
  buzzdarray_t swarms;
  uint16_t age;
};
typedef struct buzzswarm_elem_s* buzzswarm_elem_t;

void update_sensors()
/*
/ Update from all external inputs
-------------------------------*/
{
  // Update sensors
  buzzuav_closures::buzzuav_update_battery(VM);
  buzzuav_closures::buzzuav_update_xbee_status(VM);
  buzzuav_closures::buzzuav_update_prox(VM);
  buzzuav_closures::buzzuav_update_currentpos(VM);
  buzzuav_closures::update_neighbors(VM);
  buzzuav_closures::buzzuav_update_flight_status(VM);
  buzzuav_closures::buzzuav_update_yolo_boxes(VM);
}

void buzz_script_step()
/*
/ Step through the buzz script
-------------------------------*/
{
  //  Process available messages
  in_message_process();
  //  Update sensors
  update_sensors();
  // Call Buzz step() function
  if (buzzvm_function_call(VM, "step", 0) != BUZZVM_STATE_READY)
  {
    ROS_ERROR("%s: execution terminated abnormally: %s", BO_FNAME, buzz_error_info());
    buzzvm_dump(VM);
  }
  // Pop the step return value
  buzzvm_pop(VM);
}

void buzz_script_destroy()
/*
/ Destroy the bvm and other related ressources
-------------------------------------*/
{
  if (VM)
  {
    if (VM->state != BUZZVM_STATE_READY)
    {
      ROS_ERROR("%s: execution terminated abnormally: %s", BO_FNAME, buzz_error_info());
      buzzvm_dump(VM);
    }
    buzzvm_function_call(VM, "destroy", 0);
    buzzvm_destroy(&VM);
    free(BO_FNAME);
    buzzdebug_destroy(&DBG_INFO);
  }
  ROS_INFO("Script execution stopped.");
}

int buzz_script_done()
/*
/ Check if the BVM execution terminated
---------------------------------------*/
{
  return VM->state != BUZZVM_STATE_READY;
}

int update_step_test()
/*
/ Step test for the update mechanism
------------------------------------*/
{
  //  Process available messages
  in_message_process();
  buzzuav_closures::buzzuav_update_battery(VM);
  buzzuav_closures::buzzuav_update_prox(VM);
  buzzuav_closures::buzzuav_update_currentpos(VM);
  buzzuav_closures::update_neighbors(VM);
  buzzuav_closures::buzzuav_update_flight_status(VM);

  int a = buzzvm_function_call(VM, "step", 0);
  // Pop step return value
  buzzvm_pop(VM);

  if (a != BUZZVM_STATE_READY)
  {
    ROS_ERROR("%s: execution terminated abnormally: %s\n\n", BO_FNAME, buzz_error_info());
    fprintf(stdout, "step test VM state %i\n", a);
  }

  return a == BUZZVM_STATE_READY;
}

void set_robot_var(int ROBOTS)
/*
/ set swarm size in the BVM
-----------------------------*/
{
  buzzvm_pushs(VM, buzzvm_string_register(VM, "ROBOTS", 1));
  buzzvm_pushi(VM, ROBOTS);
  buzzvm_gstore(VM);
}

void set_ca_on_var(int CA_ON)
/*
/ set swarm size in the BVM
-----------------------------*/
{
  buzzvm_pushs(VM, buzzvm_string_register(VM, "CA_ON", 1));
  buzzvm_pushi(VM, CA_ON);
  buzzvm_gstore(VM);
}

int get_inmsg_size()
/*
/ get the incoming msgs size
------------------------------*/
{
  return IN_MSG.size();
}

std::vector<uint8_t*> get_inmsg_vector()
{
  return IN_MSG;
}

buzzvm_t get_vm()
/*
/ return the BVM
----------------*/
{
  return VM;
}

string get_bvmstate()
/*
/ return current BVM state
---------------------------------------*/
{
  std::string uav_state = "Unknown";
  if (VM && VM->strings)
  {
    buzzvm_pushs(VM, buzzvm_string_register(VM, "BVMSTATE", 1));
    buzzvm_gload(VM);
    buzzobj_t obj = buzzvm_stack_at(VM, 1);
    if (obj->o.type == BUZZTYPE_STRING)
      uav_state = string(obj->s.value.str);
    else
      uav_state = "Not Available";
    buzzvm_pop(VM);
  }
  return uav_state;
}

int get_swarmsize()
{
  return (int)buzzdict_size(VM->swarmmembers) + 1;
}
}

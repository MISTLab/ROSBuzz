/** @file      rosbuzz.cpp
 *  @version   1 
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS. 
 *  @copyright 2016 MistLab. All rights reserved.
 */

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/CommandInt.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/BatteryStatus.h"
#include "mavros_msgs/Mavlink.h"
#include "sensor_msgs/NavSatStatus.h"
#include "rosbuzz/neighbour_pos.h"
#include <sstream>
#include <buzz/buzzasm.h>
#include "buzzuav_closures.h"
#include "buzz_utility.h"
#include "uav_utility.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <ostream>
#include <map>
using namespace std;

static int done = 0;
static double cur_pos[3];
static uint64_t payload;
static std::map< int,  Pos_struct> neighbours_pos_map;
static int timer_step=0;
/**
 * This program implements Buzz node in ros.
 */
void maintain_pos(int tim_step){
if(timer_step >=10){
neighbours_pos_map.clear();
timer_step=0;
}

}
void neighbours_pos_maintain(int id, Pos_struct pos_arr ){
map< int, Pos_struct >::iterator it = neighbours_pos_map.find(id);
if(it!=neighbours_pos_map.end())
neighbours_pos_map.erase(it);
neighbours_pos_map.insert(make_pair(id, pos_arr));
}

/*usage*/
void usage(const char* path, int status) {
   fprintf(stderr, "Usage:\n");
   fprintf(stderr, "\t%s <stream> <msg_size> <file.bo> <file.bdb>\n\n", path);
   fprintf(stderr, "== Options ==\n\n");
   fprintf(stderr, "  stream        The stream type: tcp or bt\n");
   fprintf(stderr, "  msg_size      The message size in bytes\n");
   fprintf(stderr, "  file.bo       The Buzz bytecode file\n");
   fprintf(stderr, "  file.bdbg     The Buzz debug file\n\n");
   exit(status);
}
/*Set the current position of the robot callback*/
void set_cur_pos(double latitude,
		 double longitude,
		 double altitude){
/* set the current position of the robot*/
cur_pos [0] =latitude;
cur_pos [1] =longitude;
cur_pos [2] =altitude;

}

/*convert from catresian to spherical coordinate system callback */
double* cvt_spherical_coordinates(double neighbours_pos_payload []){
double latitude,longitude,altitude;
//for(int i=0;i<pos_vect.size();i++){
latitude=neighbours_pos_payload[0];
longitude = neighbours_pos_payload[1];
altitude=neighbours_pos_payload[2];
neighbours_pos_payload[0]=sqrt(pow(latitude,2.0)+pow(longitude,2.0)+pow(altitude,2.0));
neighbours_pos_payload[1]=atan(longitude/latitude);
neighbours_pos_payload[2]=atan((sqrt(pow(latitude,2.0)+pow(longitude,2.0)))/altitude);
return neighbours_pos_payload;
}

/*battery status callback*/ 
void battery(const mavros_msgs::BatteryStatus::ConstPtr& msg)
{
set_battery(msg->voltage,msg->current,msg->remaining);
}

/*current position callback*/
void current_pos(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
set_cur_pos(msg->latitude,msg->longitude,msg->altitude);
}

/*payload callback callback*/
void payload_obt(const mavros_msgs::Mavlink::ConstPtr& msg)
{
uint64_t message_obt[(msg->payload64.end())-(msg->payload64.begin())];
int i = 0;

	// print all the remaining numbers
	for(std::vector<long unsigned int>::const_iterator it = msg->payload64.begin(); it != msg->payload64.end(); ++it)
	{
		message_obt[i] =(uint64_t) *it;
                cout<<" obtaind message "<<message_obt[i]<<endl;
		i++;
        }
double neighbours_pos_payload[3];
memcpy(neighbours_pos_payload, message_obt, 3*sizeof(uint64_t));
//cout<<"neighbours pos recived "<<" lat " << (double)neighbours_pos_payload[0]<<"  long "<<(double)neighbours_pos_payload[1] <<"  alti "<<(double)neighbours_pos_payload[2]<<endl;

for(i=0;i<3;i++){
neighbours_pos_payload[i]=neighbours_pos_payload[i]-cur_pos[i];
}
 double* cvt_neighbours_pos_payload = cvt_spherical_coordinates(neighbours_pos_payload);
//cout<<" neighbours pos cvt " <<" lat " << cvt_neighbours_pos_payload[0]<<"  long "<<cvt_neighbours_pos_payload[1] <<"  alti "<<cvt_neighbours_pos_payload[2]<<endl;

    uint16_t* out = u64_cvt_u16((uint64_t)*(message_obt+3));  
//cout << " values " <<out[0] <<"  "<<(int)out[1] <<"  "<<out[2] <<"  "<<out[3] <<"  "<<endl;
struct pos_struct n_pos;
n_pos.x=cvt_neighbours_pos_payload[0];
n_pos.y=cvt_neighbours_pos_payload[1];
n_pos.z=cvt_neighbours_pos_payload[2];
neighbours_pos_maintain((int)out[1],n_pos);
delete[] out;
in_msg_process((message_obt+3));
   
}
 

int oldcmdID=0;
int rc_cmd;
bool rc_callback(mavros_msgs::CommandInt::Request  &req,
         mavros_msgs::CommandInt::Response &res){

if(req.command==oldcmdID)
	return true;
   else
	oldcmdID=req.command;
   switch(req.command)
   {
	case mavros_msgs::CommandCode::NAV_TAKEOFF:
   		ROS_INFO("RC_call: TAKE OFF!!!!");
		rc_cmd=mavros_msgs::CommandCode::NAV_TAKEOFF;
		rc_call(rc_cmd);
		res.success = true;
		break;
	case mavros_msgs::CommandCode::NAV_LAND:
   		ROS_INFO("RC_Call: LAND!!!!");
		rc_cmd=mavros_msgs::CommandCode::NAV_LAND;
		rc_call(rc_cmd);
		res.success = true;
		break;
	case mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH:
   		ROS_INFO("RC_Call: GO HOME!!!!");
		rc_cmd=mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
		rc_call(rc_cmd);
		res.success = true;
		break;
	case mavros_msgs::CommandCode::NAV_WAYPOINT:
   		ROS_INFO("RC_Call: GO TO!!!! x = %f , y = %f , Z = %f",req.param1,req.param2,req.param3);
		double rc_goto[3];
   		rc_goto[0]=req.param1;
		rc_goto[1]=req.param2;
		rc_goto[2]=req.param3;
                
		set_goto(rc_goto);
		rc_cmd=mavros_msgs::CommandCode::NAV_WAYPOINT;
		rc_call(rc_cmd);
		res.success = true;
		break;
	default:
		res.success = false;
		break;
   }
   return true;
}


/*controlc handler callback*/
static void ctrlc_handler(int sig) {
   done = 1;
}



int main(int argc, char **argv)
{

   /*Compile the buzz code .bzz to .bo*/
   system("rm ../catkin_ws/src/rosbuzz/src/out.basm ../catkin_ws/src/rosbuzz/src/out.bo ../catkin_ws/src/rosbuzz/src/out.bdbg");
   system("bzzparse ../catkin_ws/src/rosbuzz/src/test.bzz ../catkin_ws/src/rosbuzz/src/out.basm");
   system("bzzasm ../catkin_ws/src/rosbuzz/src/out.basm ../catkin_ws/src/rosbuzz/src/out.bo ../catkin_ws/src/rosbuzz/src/out.bdbg");
  
  /*initiate rosBuzz*/
  ros::init(argc, argv, "rosBuzz"); 
  ROS_INFO("Buzz_node");

  /*Create node Handler*/
  ros::NodeHandle n_c;

 
  /*subscribers*/
  ros::Subscriber current_position_sub = n_c.subscribe("current_pos", 1000, current_pos);

  ros::Subscriber battery_sub = n_c.subscribe("battery_state", 1000, battery);

  ros::Subscriber payload_sub = n_c.subscribe("payload_in", 1000, payload_obt);


  /*publishers*/
  ros::Publisher payload_pub = n_c.advertise<mavros_msgs::Mavlink>("payload_out", 1000);

  /* Clients*/
  ros::ServiceClient mav_client = n_c.serviceClient<mavros_msgs::CommandInt>("djicmd");
  
  /*Services*/
  ros::ServiceServer service = n_c.advertiseService("djicmd_rc", rc_callback);
  ROS_INFO("Ready to receive Mav Commands from dji RC client");
  
  /*loop rate of ros*/
  ros::Rate loop_rate(1);

   
   /* The bytecode filename */
   char* bcfname = "../catkin_ws/src/rosbuzz/src/out.bo"; //argv[1];
   /* The debugging information file name */
   char* dbgfname = "../catkin_ws/src/rosbuzz/src/out.bdbg"; //argv[2];
   /* Set CTRL-C handler */
   signal(SIGTERM, ctrlc_handler);
   signal(SIGINT, ctrlc_handler);

   
   /* Set the Buzz bytecode */
   if(buzz_script_set(bcfname, dbgfname)) {
   fprintf(stdout, "Bytecode file found and set\n");

  
  mavros_msgs::CommandInt cmd_srv;
  int count = 0;
  while (ros::ok() && !done && !buzz_script_done())
  {

      /*Update neighbors position inside Buzz*/
      neighbour_pos_callback(neighbours_pos_map);
  
      /* Main loop */
      buzz_script_step();
 
    /*prepare the goto publish message */
    
    double* goto_pos = getgoto();
    cmd_srv.request.param1 = goto_pos[0];
    cmd_srv.request.param2 = goto_pos[1];
    cmd_srv.request.param3 = goto_pos[2];
    
    cmd_srv.request.command =  getcmd();
    /* diji client call*/
    if (mav_client.call(cmd_srv)){ ROS_INFO("Reply: %ld", (long int)cmd_srv.response.success); }
    else{ ROS_ERROR("Failed to call service 'djicmd'"); }
    /*Prepare Pay load to be sent*/  
    uint64_t* payload_out_ptr= out_msg_process();
    uint64_t  position[3];
    memcpy(position, cur_pos, 3*sizeof(uint64_t));
    mavros_msgs::Mavlink payload_out;
    payload_out.payload64.push_back(position[0]);
    payload_out.payload64.push_back(position[1]);
    payload_out.payload64.push_back(position[2]);
    
    uint16_t* out = u64_cvt_u16(payload_out_ptr[0]);  
    for(int i=0;i<out[0];i++){
    payload_out.payload64.push_back(payload_out_ptr[i]);
    }
    /*publish prepared messages in respective topic*/
    payload_pub.publish(payload_out);
    delete[] out;
    delete[] payload_out_ptr;
    /*run once*/
    ros::spinOnce();
    /*sleep for the mentioned loop rate*/
    loop_rate.sleep();
    ++count;
    timer_step+=1;
    maintain_pos(timer_step);
   /* cout<<"time step"<<timer_step<<endl;
    map< int, Pos_struct >::iterator it;
    for (it=neighbours_pos_map.begin(); it!=neighbours_pos_map.end(); ++it){
    std::cout << it->first << " => " << (it->second).x <<"   " << (it->second).y <<"   " << (it->second).z<<endl;
	}
   */

  }
  /* Cleanup */
   buzz_script_destroy();

 /* Stop the robot */
   uav_done();
   
}
  /* All done */
   return 0;

}



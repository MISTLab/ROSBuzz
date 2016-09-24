/*

 * Header

 */

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/BatteryStatus.h"
#include "mavros_msgs/Mavlink.h"
#include <sstream>
#include <buzz/buzzasm.h>
#include "buzzuav_closures.h"
#include "buzz_utility.h"
#include "uav_utility.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>


static int done = 0;

static double cur_pos[3];
double neighbor_pos[4]; 
uint64_t payload;

/**
 * This program implements Buzz node in ros.
 */
/*

 * Print usage information
 */
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
void cvt_spherical_coordinates(double latitude,
 		  double longitude,
		  double altitude){
/** convert the current position coordination system from cartetion system (gps) to sperical system (Buzz) type **/ 
 
 neighbor_pos[0] = 01;
 neighbor_pos[1]  = sqrt(pow(latitude,2.0)+pow(longitude,2.0)+pow(altitude,2.0));
 neighbor_pos[2] = atan(longitude/latitude);
 neighbor_pos[3] = atan((sqrt(pow(latitude,2.0)+pow(longitude,2.0)))/altitude);
fprintf(stdout, "%.15f :distance value\n", neighbor_pos[1]);
fprintf(stdout, "%.15f :elevation\n", neighbor_pos[2]);
fprintf(stdout, "%.15f :azimuth\n", neighbor_pos[3]);

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

/*payload callback*/
void payload_obt(const mavros_msgs::Mavlink::ConstPtr& msg)
{
long unsigned int message_obt[(msg->payload64.end())-(msg->payload64.begin())];
int i = 0;
	// print all the remaining numbers
	for(std::vector<long unsigned int>::const_iterator it = msg->payload64.begin(); it != msg->payload64.end(); ++it)
	{
		message_obt[i] = *it;
		i++;
        }
in_msg_process(message_obt, neighbor_pos);

}

/*neighbours position call back */
void neighbour_pos(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  
/*obtain the neigbours position*/

double latitude =(msg->latitude-cur_pos[0]);
double longitude =(msg->longitude-cur_pos[1]);
double altitude =(msg->altitude-cur_pos[2]);

  ROS_INFO("I heard neighbour latitude: [%15f]", latitude);
  ROS_INFO("I heard neighbour longitude: [%15f]", longitude);
  ROS_INFO("I heard neighbour altitude: [%15f]", altitude);

cvt_spherical_coordinates(latitude,longitude,altitude);
//neighbour_location_handler( distance, azimuth, elevation, 01);

}

/*controlc handler callback*/
static void ctrlc_handler(int sig) {
   done = 1;
}



int main(int argc, char **argv)
{

   /*Compile the buzz code .bzz to .bo*/
   system("bzzparse /home/vivek/catkin_ws/src/rosbuzz/src/test.bzz /home/vivek/catkin_ws/src/rosbuzz/src/out.basm");
   system("bzzasm /home/vivek/catkin_ws/src/rosbuzz/src/out.basm /home/vivek/catkin_ws/src/rosbuzz/src/out.bo /home/vivek/catkin_ws/src/rosbuzz/src/out.bdbg");
  
  /*initiate rosBuzz*/
  ros::init(argc, argv, "rosBuzz"); 


  /*Create node Handler*/
  ros::NodeHandle n_c;

 
  /*subscribers*/
  ros::Subscriber current_position_sub = n_c.subscribe("current_pos", 1000, current_pos);

  ros::Subscriber neighbour_position_sub = n_c.subscribe("neighbour_pos", 1000, neighbour_pos);

  ros::Subscriber battery_sub = n_c.subscribe("battery_state", 1000, battery);

  ros::Subscriber payload_sub = n_c.subscribe("pay_load_in", 1000, payload_obt);


  /*publishers*/
  ros::Publisher goto_pub = n_c.advertise<mavros_msgs::GlobalPositionTarget>("go_to", 1000);
  
  ros::Publisher cmds_pub = n_c.advertise<mavros_msgs::State>("newstate", 1000);

  ros::Publisher payload_pub = n_c.advertise<mavros_msgs::Mavlink>("pay_load_out", 1000);

  /*loop rate of ros*/
  ros::Rate loop_rate(1);

   
   /* The bytecode filename */
   char* bcfname = "/home/vivek/catkin_ws/src/rosbuzz/src/out.bo"; //argv[1];
   /* The debugging information file name */
   char* dbgfname = "/home/vivek/catkin_ws/src/rosbuzz/src/out.bdbg"; //argv[2];
   /* Set CTRL-C handler */
   signal(SIGTERM, ctrlc_handler);
   signal(SIGINT, ctrlc_handler);

   
   /* Set the Buzz bytecode */
   if(buzz_script_set(bcfname, dbgfname)) {
   fprintf(stdout, "Bytecode file found and set\n");

// buzz setting

  int count = 0;
  while (ros::ok() && !done && !buzz_script_done())
  {
   

 
      /* Main loop */
         buzz_script_step();
         
      /* Cleanup */
        // buzz_script_destroy();
   

    /*prepare the goto publish message */
    mavros_msgs::GlobalPositionTarget goto_set;
   double* goto_pos = getgoto();
    
    goto_set.latitude = goto_pos[0];
    goto_set.longitude=goto_pos[1];
    goto_set.altitude=goto_pos[2];
    /*prepare commands for takeoff,land and gohome*/
    mavros_msgs::State cmds_set;
    char tmp[20];
    sprintf(tmp,"%i",getcmd());
    cmds_set.mode =  tmp;
    /*Prepare Pay load to be sent*/  
    unsigned long int* payload_out_ptr= out_msg_process();  
    mavros_msgs::Mavlink payload_out;
    payload_out.payload64.push_back(*payload_out_ptr);
    /*publish prepared messages in respective topic*/
    goto_pub.publish(goto_set);
    cmds_pub.publish(cmds_set);
    payload_pub.publish(payload_out);

    /*run once*/
    ros::spinOnce();
    /*sleep for the mentioned loop rate*/
    loop_rate.sleep();
    ++count;
  }

 /* Stop the robot */
   uav_done();
   
}
  /* All done */
   return 0;

}



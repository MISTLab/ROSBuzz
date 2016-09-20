/*

 * Header

 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
#include "sensor_msgs/NavSatFix.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/BatteryStatus.h"
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include "std_msgs/Float64.h"
#include <sstream>
#include <buzz/buzzasm.h>
#include "buzzuav_closures.h"
#include "buzz_utility.h"
//extern "C" {
#include "uav_utility.h"
//}
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

static int done = 0;

static double cur_pos [3];
double distance;
double azimuth;
double elevation; 


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
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
void set_cur_pos(double latitude,
		 double longitude,
		 double altitude){
/* set the current position of the robot*/
cur_pos [0] =latitude;
cur_pos [1] =longitude;
cur_pos [2] =altitude;
}

void cvt_spherical_coordinates(double latitude,
 		  double longitude,
		  double altitude){
/** convert the current position coordination system from cartetion system (gps) to sperical system (Buzz) type **/ 
 distance  = sqrt(pow(latitude,2.0)+pow(longitude,2.0)+pow(altitude,2.0));

 elevation = atan(longitude/latitude);
 azimuth = atan((sqrt(pow(latitude,2.0)+pow(longitude,2.0)))/altitude);
fprintf(stdout, "%.15f :distance value\n", distance);
fprintf(stdout, "%.15f :elevation\n", elevation);
fprintf(stdout, "%.15f :azimuth\n", azimuth);

}

// %Tag(CALLBACK)%
void battery(const mavros_msgs::BatteryStatus::ConstPtr& msg)
{
set_battery(msg->voltage,msg->current,msg->remaining);
}

void current_pos(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  /*ROS_INFO("I heard current latitude: [%15f]", msg->latitude);
  ROS_INFO("I heard current longitude: [%15f]", msg->longitude);
  ROS_INFO("I heard current altitude: [%15f]", msg->altitude);*/
/*obtain the current posituion of the robot*/
//sperical* = cvt_spherical_coordinates(msg->latitude,msg->longitude,msg->altitude);
set_cur_pos(msg->latitude,msg->longitude,msg->altitude);
}
// %EndTag(CALLBACK)%

// %Tag(CALLBACK)%
void neighbour_pos(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  
/*obtain the current position of the robot*/

double latitude =(msg->latitude-cur_pos[0]);
double longitude =(msg->longitude-cur_pos[1]);
double altitude =(msg->altitude-cur_pos[2]);

  ROS_INFO("I heard neighbour latitude: [%15f]", latitude);
  ROS_INFO("I heard neighbour longitude: [%15f]", longitude);
  ROS_INFO("I heard neighbour altitude: [%15f]", altitude);

cvt_spherical_coordinates(latitude,longitude,altitude);
neighbour_location_handler( distance, azimuth, elevation, 01);

}
// %EndTag(CALLBACK)%


static void ctrlc_handler(int sig) {
   done = 1;
}




int main(int argc, char **argv)
{


   system("bzzparse /home/vivek/catkin_ws/src/rosbuzz/src/test.bzz /home/vivek/catkin_ws/src/rosbuzz/src/out.basm");
   system("bzzasm /home/vivek/catkin_ws/src/rosbuzz/src/out.basm /home/vivek/catkin_ws/src/rosbuzz/src/out.bo /home/vivek/catkin_ws/src/rosbuzz/src/out.bdbg");
  
// %Tag(INIT)%
  ros::init(argc, argv, "rosBuzz"); 
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
  ros::NodeHandle n_c;
 // ros::NodeHandle n_n;
// %EndTag(NODEHANDLE)%

 
// %Tag(SUBSCRIBER)%
  ros::Subscriber current_position = n_c.subscribe("current_pos", 1000, current_pos);

  ros::Subscriber neighbour_position = n_c.subscribe("neighbour_pos", 1000, neighbour_pos);

  ros::Subscriber battery_sub = n_c.subscribe("battery_state", 1000, battery);
// %EndTag(SUBSCRIBER)%

// %Tag(PUBLISHER)%
  ros::Publisher goto_pub = n_c.advertise<mavros_msgs::GlobalPositionTarget>("go_to", 1000);
  
  ros::Publisher cmds_pub = n_c.advertise<mavros_msgs::State>("newstate", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(1);
// %EndTag(LOOP_RATE)%


// Buzz stuff
//

//
/* Parse command line */
 /*  if(argc != 5) usage(argv[0], 0); */
   /* The stream type */
 /*  char* stream = argv[1];
   if(strcmp(stream, "tcp") != 0 &&
      strcmp(stream, "bt") != 0) {
      fprintf(stderr, "%s: unknown stream type '%s'\n", argv[0], stream);
      usage(argv[0], 0);
   }
   /* The message size */
   /*char* endptr;
   int msg_sz = strtol(argv[2], &endptr, 10);
   if(endptr == argv[2] || *endptr != '\0') {
      fprintf(stderr, "%s: can't parse '%s' into a number\n", argv[0], argv[2]);
      return 0;
   }
   if(msg_sz <= 0) {
      fprintf(stderr, "%s: invalid value %d for message size\n", argv[0], msg_sz);
      return 0;
   } */
   
   /* The bytecode filename */
   char* bcfname = "/home/vivek/catkin_ws/src/rosbuzz/src/out.bo"; //argv[1];
   /* The debugging information file name */
   char* dbgfname = "/home/vivek/catkin_ws/src/rosbuzz/src/out.bdbg"; //argv[2];
   /* Wait for connection */
   //if(!buzz_listen(stream, msg_sz)) return 1;
   /* Set CTRL-C handler */
   signal(SIGTERM, ctrlc_handler);
   signal(SIGINT, ctrlc_handler);

   /* Initialize the robot */
   //kh4_setup();

   /* Set the Buzz bytecode */
   if(buzz_script_set(bcfname, dbgfname)) {
   fprintf(stdout, "Bytecode file found and set\n");

// buzz setting

  int count = 0;
  while (ros::ok() && !done && !buzz_script_done())
  {
   

 
   // while(!done && !buzz_script_done())
      /* Main loop */
         buzz_script_step();
         
      /* Cleanup */
    //  buzz_script_destroy();
   

// %Tag(FILL_MESSAGE)%
    mavros_msgs::GlobalPositionTarget goto_set;
   double* goto_pos = getgoto();
    
    goto_set.latitude = goto_pos[0];
    goto_set.longitude=goto_pos[1];
    goto_set.altitude=goto_pos[2];

    mavros_msgs::State cmds_set;
char tmp[20];
    	sprintf(tmp,"%i",getcmd());
cmds_set.mode =  tmp;
    
// %EndTag(FILL_MESSAGE)%


// %Tag(PUBLISH)%
    goto_pub.publish(goto_set);
    cmds_pub.publish(cmds_set);
// %EndTag(PUBLISH)%


// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%
// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }
 /* Stop the robot */
   uav_done();
   
}
  /* All done */
   return 0;

}
// %EndTag(FULLTEXT)%



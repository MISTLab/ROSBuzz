/*

 * Header

 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>
#include <buzz/buzzasm.h>
extern "C" {
#include "buzz_utility.h"
}
extern "C" {
#include "kh4_utility.h"
}
#include <stdlib.h>
#include <string.h>
#include <signal.h>

static int done = 0;

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

// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%


static void ctrlc_handler(int sig) {
   done = 1;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "outgoing");
  ros::init(argc, argv, "incoming");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n_outgoing;
  ros::NodeHandle n_incoming;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher outgoing_pub = n_outgoing.advertise<std_msgs::String>("outgoing", 1000);
// %EndTag(PUBLISHER)%

// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n_incoming.subscribe("incoming", 1000, chatterCallback);
// %EndTag(SUBSCRIBER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(2);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%


// buzz setting

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
   


// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

// %Tag(PUBLISH)%
    outgoing_pub.publish(msg);
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
   kh4_done();
   /* All done */
   return 0;
}
  

}
// %EndTag(FULLTEXT)%


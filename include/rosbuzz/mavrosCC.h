#pragma once

#if MAVROSKINETIC

const short MISSION_START = mavros_msgs::CommandCode::MISSION_START;
const short DO_MOUNT_CONTROL = mavros_msgs::CommandCode::DO_MOUNT_CONTROL;
const short COMPONENT_ARM_DISARM = mavros_msgs::CommandCode::COMPONENT_ARM_DISARM;
const short NAV_SPLINE_WAYPOINT = mavros_msgs::CommandCode::NAV_SPLINE_WAYPOINT;
const short IMAGE_START_CAPTURE = mavros_msgs::CommandCode::IMAGE_START_CAPTURE;

#else

const short MISSION_START = mavros_msgs::CommandCode::CMD_MISSION_START;
const short DO_MOUNT_CONTROL = mavros_msgs::CommandCode::CMD_DO_MOUNT_CONTROL;
const short COMPONENT_ARM_DISARM = mavros_msgs::CommandCode::CMD_COMPONENT_ARM_DISARM;
const short NAV_SPLINE_WAYPOINT = 82;
const short IMAGE_START_CAPTURE = 2000;

#endif

const short NAV_TAKEOFF = mavros_msgs::CommandCode::NAV_TAKEOFF;
const short NAV_LAND = mavros_msgs::CommandCode::NAV_LAND;
const short NAV_RETURN_TO_LAUNCH = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
const short NAV_WAYPOINT = mavros_msgs::CommandCode::NAV_WAYPOINT;
const short CMD_REQUEST_UPDATE = 666;
const short CMD_SYNC_CLOCK = 777;

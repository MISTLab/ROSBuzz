#pragma once

#include <buzz/buzzvm.h>
#include <stdio.h>
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/Mavlink.h"
#include "ros/ros.h"
#include <rosbuzz/buzz_utility.h>
#include <rosbuzz/mavrosCC.h>

#define EARTH_RADIUS (double)6371000.0
#define DEG2RAD(DEG) (double)((DEG) * ((M_PI) / (180.0)))
#define RAD2DEG(RAD) (double)((RAD) * ((180.0) / (M_PI)))

static std::string log_path;
/* Path Planner using ompl libs definition of namespaces and CONSTANST */
#if OMPL_FOUND
  #include <ompl/geometric/planners/rrt/RRTstar.h>
  #include <ompl/tools/benchmark/Benchmark.h>
  #include <ompl/base/spaces/RealVectorStateSpace.h>
  #include "../src/path_planner_utils/svg_image.h"
  #include "../src/path_planner_utils/path_existance_checking.h"
  
  namespace ob = ompl::base;
  namespace og = ompl::geometric;
  namespace pe = pathexsitance;
  /*Additional variables declared for the path planner */
  static const float MAX_ALT = 5;
  static const float PLANE_RESOLUTION = 0.1;
   /* Map file name */
  static std::string strmapFName;
  static int map_option=2; // IF 1 map is 3d, If 0 map is 2d, and IF 2 load an empty 2d map 
  static int MIN_X = 0;
  static int MAX_X;
  static int MIN_Y = 0;
  static int MAX_Y;
  static double half_map_height;
  static double half_map_length; 
  static std::vector<std::vector<std::vector<int>>> Grid_map;
  static int save_solution_svg=1;
  /* Map constants within the map file */
  static const char free_space = '.';
  static const char free_space_l = 'G';
  static const char outofbound_space = '@';
  static const char outofbound_space_l = 'O';
  static const char Tree_space = 'T';
  static const char Swap_space = 'S';
  static const char Water_space = 'W';
  static const char START_space = 'A';
  static const char TARGET_space = 'X';
  static const float ROBOT_BUFFER = 0.01; // Consider robot radius
  static const float OBSTACLE_SIDES1 = 1.0f;
  static const float OBSTACLE_SIDES2 = 1.0f;
  
#endif

namespace buzzuav_closures
{
struct bounding_box_struct
{
  std::string obj_class;
  double probability;
  int64_t xmin;
  int64_t ymin;
  int64_t xmax;
  int64_t ymax;
  bounding_box_struct(std::string c, double prob, int64_t xmi, int64_t ymi, int64_t xma, int64_t yma)
    : obj_class(c), probability(prob), xmin(xmi), ymin(ymi), xmax(xma), ymax(yma){};
  bounding_box_struct()
  {
  }
};
typedef struct bounding_box_struct bounding_box;

/*
 * prextern int() function in Buzz
 * This function is used to print data from buzz
 * The command to use in Buzz is buzzros_print takes any available datatype in Buzz
 */
int buzzros_print(buzzvm_t vm);
void setWPlist(std::string file);
void setVorlog(std::string path);
void check_targets_sim(double lat, double lon, double* res);

/*
 * closure to move following a vector
 */
int buzzuav_moveto(buzzvm_t vm);
/*
 * closure to store a new GPS goal
 */
int buzzuav_storegoal(buzzvm_t vm);
/*
 * closure to control the gimbal
 */
int buzzuav_setgimbal(buzzvm_t vm);
/*
 * parse a csv list of waypoints
 */
void parse_gpslist();
/*
 * closure to export a 2D map
 */
int buzz_exportmap(buzzvm_t vm);
/*
 * closure to take a picture
 */
int buzzuav_takepicture(buzzvm_t vm);
/*
 * closure to reset RC input
 */
int buzzuav_resetrc(buzzvm_t vm);
/*
 * Returns the current command from local variable
 */
int getcmd();
/*
 * update GPS goal value
 */
void set_gpsgoal(double goal[3]);
/*
 * Sets goto position from rc client
 */
void rc_set_goto(int id, double latitude, double longitude, double altitude);
/*
 *Sets gimbal orientation from rc client
 */
void rc_set_gimbal(int id, float yaw, float roll, float pitch, float t);
/*
 * sets rc requested command
 */
void rc_call(int rc_cmd);
/*
 * sets the battery state
 */
void set_battery(float voltage, float current, float remaining);
/*
 * Update yolo boxes into buzz
 */
int buzzuav_update_yolo_boxes(buzzvm_t vm);
/*
 *Stores the received bounding boxes
 */
void store_bounding_boxes(std::vector<bounding_box> bbox);
/*
 * sets the xbee network status
 */
void set_deque_full(bool state);
void set_rssi(float value);
void set_raw_packet_loss(float value);
void set_filtered_packet_loss(float value);
// void set_api_rssi(float value);
/*
 * sets current position
 */

void set_currentNEDpos(double x, double y, double z, double yaw, double x_offset, double y_offset);

void set_currentpos(double latitude, double longitude, float altitude, float yaw);
/*
 * returns the current go to position
 */
double* getgoto();
/*
 * returns the current grid
 */
std::map<int, std::map<int, int>> getgrid();
int voronoi_center(buzzvm_t vm);

/*
 * returns the gimbal commands
 */
float* getgimbal();
/*
 *updates flight status
 */
void flight_status_update(uint8_t state);
/*
 *Update neighbors table
 */
void neighbour_pos_callback(int id, float range, float bearing, float latitude, float longitude, float elevation);
/*
 * update neighbors from in msgs
 */
void update_neighbors(buzzvm_t vm);
/*
 *Clear neighbours struct
 */
void clear_neighbours_pos();
/*
 * closure to add a neighbor status
 */
int buzzuav_addNeiStatus(buzzvm_t vm);
/*
 * returns the current array of neighbors status
 */
mavros_msgs::Mavlink get_status();

/*
 *Flight status
 */
void set_obstacle_dist(float dist[]);

/*
 * Commands the UAV to takeoff
 */
int buzzuav_takeoff(buzzvm_t vm);
/*
 * Arm command from Buzz
 */
int buzzuav_arm(buzzvm_t vm);
/*
 * Disarm from buzz
 */
int buzzuav_disarm(buzzvm_t vm);
/* Commands the UAV to land
 */
int buzzuav_land(buzzvm_t vm);

/*
 * Command the UAV to go to home location
 */
int buzzuav_gohome(buzzvm_t vm);
int buzzuav_geofence(buzzvm_t vm);

/*
 * Updates battery information in Buzz
 */
int buzzuav_update_battery(buzzvm_t vm);
/*
 * Updates xbee_status information in Buzz
 */
int buzzuav_update_xbee_status(buzzvm_t vm);
/*
 * Updates current position in Buzz
 */
int buzzuav_update_currentpos(buzzvm_t vm);
/*
 * add new target in the BVM
 */
int buzzuav_addtargetRB(buzzvm_t vm);
/*
 * Updates flight status and rc command in Buzz, put it in a tabel to acess it
 * use flight.status for flight status
 * use flight.rc_cmd for current rc cmd
 */
int buzzuav_update_flight_status(buzzvm_t vm);

/*
 * Updates IR information in Buzz
 * Proximity and ground sensors to do !!!!
 */
int buzzuav_update_prox(buzzvm_t vm);
/*
 * returns the current FC command
 */
int bzz_cmd();

int dummy_closure(buzzvm_t vm);

void set_log_path(std::string path);

#if OMPL_FOUND

int C_InitializePathPlanner(buzzvm_t vm);

std::vector<std::vector<double>> InitializePathPlanner(buzzvm_t m_tBuzzVM,  float* start_end_time);

    // Collision checker for 2d Path planner.
    class ValidityChecker : public ob::StateValidityChecker
    {
    public:
        ValidityChecker(const ob::SpaceInformationPtr& si) :
            ob::StateValidityChecker(si),gridToCheck(3),Empty(0) {}
        ValidityChecker(const ob::SpaceInformationPtr& si, std::vector<std::vector<int>> Grid_map) :
        ob::StateValidityChecker(si),gridToCheck(3),Empty(0) {
          Grid_obst = Grid_map;
        }
        ValidityChecker(std::vector<std::vector<int>> Grid_map):ob::StateValidityChecker(nullptr),
                        gridToCheck(3),Empty(0){
          Grid_obst = Grid_map;
        }
        /* Constructor for empty map */
        ValidityChecker(const ob::SpaceInformationPtr& si, std::vector<std::vector<int>> Grid_map, int empty) :
        ob::StateValidityChecker(si),gridToCheck(3),Empty(empty){
          Grid_obst = Grid_map;
        }
        bool isValid(double* state2D) const
        {
        
          if(Grid_obst[state2D[0]][state2D[1]]) return false;
          return true;
            // return this->clearance(state) > 0.0;
        }
        // Returns whether the given state's position overlaps
        // any of the obstacles in grid map
        bool isValid(const ob::State* state) const
        {
          if(Empty){
            return true;
          }
          else{
            const ob::RealVectorStateSpace::StateType* state2D =
                  state->as<ob::RealVectorStateSpace::StateType>();
              for(int i=-1; i < gridToCheck; ++i){
                for(int j=-1; j < gridToCheck;++j){
                  if(Grid_obst[state2D->values[0]+i][state2D->values[1]+j]) return false;
                }
              }
            return true;
          }
        }
    protected:
      int gridToCheck, Empty;
      std::vector<std::vector<int>> Grid_obst;
    };
    //collision checker for 3d path planner
    class ValidityChecker3D : public ob::StateValidityChecker
    {
    public:
        ValidityChecker3D(const ob::SpaceInformationPtr& si, std::vector<std::vector<std::vector<int>>> 
          Grid_map) :ob::StateValidityChecker(si),gridToCheck(3) {
          Grid_obst= Grid_map;
        }
        // Returns whether the given state's position overlaps the
        // circular obstacle
        bool isValid(const ob::State* state) const
        {
          const ob::RealVectorStateSpace::StateType* state3D =
                state->as<ob::RealVectorStateSpace::StateType>();
          int x =round(state3D->values[0]);
          int y =round(state3D->values[1]);
              int comp_planning_plane = (state3D->values[2]/PLANE_RESOLUTION);
          if(Grid_obst[comp_planning_plane][x][y]) return false;
            int k=-2, check_z =gridToCheck;
          if(state3D->values[2] > MAX_ALT-1 || state3D->values[2] < 1 ){ k=0; check_z=1;}
          for(; k < check_z;++k){
            comp_planning_plane = (state3D->values[2]/PLANE_RESOLUTION)+k;
            if(Grid_obst[comp_planning_plane][x][y]) return false;
            else{

              for(int i=-2; i < gridToCheck; ++i){
                for(int j=-2; j < gridToCheck;++j){
                  if(x+i < Grid_obst[comp_planning_plane].size() &&
                     x+i >= 0 &&
                     y+j < Grid_obst[comp_planning_plane][x+i].size() &&
                     y+j >=0 ){
                    if(Grid_obst[comp_planning_plane][x+i][y+j]){
                      if(clearance(state,x+i,y+j,comp_planning_plane*PLANE_RESOLUTION)){
                        // std::cout<<"Clearence not satisfied"<<std::endl;
                        return false;
                      }
                    }
                  }
                }
              }
            }

        }
          return true;
            // return this->clearance(state) > 0.0;
        }
        // Returns the distance from the given state's position to
        // another state.
        bool clearance(const ob::State* state1, double x2, double y2, double z2) const
        {
            // We know we're working with a RealVectorStateSpace in this
            // example, so we downcast state into the specific type.
            const ob::RealVectorStateSpace::StateType* state_1 =
                state1->as<ob::RealVectorStateSpace::StateType>();
            // const ob::RealVectorStateSpace::StateType* state_2 =
            //     state2->as<ob::RealVectorStateSpace::StateType>();
            // Extract the robot's (x,y) position from its state
            double x1 = state_1->values[0];
            double y1 = state_1->values[1];
            double z1 = state_1->values[2];

            // double x2 = state_1->values[0];
            // double y2 = state_1->values[1];
            // double z2 = state_1->values[2];

      
            // Distance formula between two points, offset by a circle
            // radius for clearence from obst
            return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)) - 1.6 < 0.0;
        }

    protected:
      int gridToCheck;
      std::vector<std::vector<std::vector<int>>> Grid_obst;
    };
#endif
}

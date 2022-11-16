/** @file      buzzuav_closures.cpp
 *  @version   1.0
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS.
 *  @author    Vivek Shankar Varadharajan and David St-Onge
 *  @copyright 2016 MistLab. All rights reserved.
 */

#include <rosbuzz/buzzuav_closures.h>
#include "math.h"
#include <rosbuzz/VoronoiDiagramGenerator.h>

namespace buzzuav_closures
{
// TODO: Minimize the required global variables and put them in the header
// static const rosbzz_node::roscontroller* roscontroller_ptr;
static double goto_pos[4]={0,0,0,0};
static double goto_gpsgoal[3];
static double cur_pos[4];
static double cur_offset[2];
static double cur_NEDpos[4];
static float navigation_goal[2];
static float move_base_local_goal[2]={0.0,0.0}; 
static int hierarchical_status[7]={-1,-1,-1,-1,-1,-1,-1};
static float planner_home_location[5]={0.0,0.0,0.0,0.0,0.0};

static int goal_status = 0;
static int new_move_goal_available=0;
static int goal_read = 0;
static int rc_id = -1;
static int rc_cmd = 0;
static double rc_gpsgoal[3];
static float rc_gimbal[4];

static float batt[3];
static float prox_init_val = 1000;
static float obst[8] = { prox_init_val, prox_init_val, prox_init_val, prox_init_val,
                         prox_init_val, prox_init_val, prox_init_val, prox_init_val };
static const int number_of_proximity=4;
static uint8_t status;

static int cur_cmd = 0;
static int buzz_cmd = 0;
static int exploration_planner_cmd = 0;
static float height = 0;

static bool deque_full = false;
static int rssi = 0;
static float raw_packet_loss = 0.0;
static int filtered_packet_loss = 0;
static float api_rssi = 0.0;
static bool logVoronoi = false;
static buzzvm_t cur_BuzzVM;
static std::vector<bounding_box> yolo_boxes;

std::vector<std::vector<float>> exploration_path;
std::vector<std::vector<float>> homing_path;
std::vector<std::vector<float>> interpolation_homing_path;
std::vector<std::vector<float>> nav_tube;
std::map<int, buzz_utility::Pos_with_ori_struct> fiducial_tags_map;

std::ofstream voronoicsv;

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

struct Point
{
  float x;
  float y;
  Point() : x(0.0), y(0.0)
  {
  }
  Point(float x, float y) : x(x), y(y)
  {
  }
};
string WPlistname = "";

std::map<int, buzz_utility::RB_struct> targets_map;
std::map<int, buzz_utility::RB_struct> wplist_map;
std::map<int, buzz_utility::RB_struct> neighbors_map;
std::map<int, buzz_utility::RB_struct> neighbors_map_prev;
std::map<int, buzz_utility::neighbors_status> neighbors_status_map;
std::map<int, std::map<int, int>> grid;
std::map<int, std::pair<int, int>> path;
float origin_x, origin_y, resolution;

/*
/ Utility function to push a table with string
----------------------------------------------------------- */

buzzvm_state TablePut_str_fval(buzzobj_t t_table,
                               const std::string& str_key,
                               float f_value,
                               buzzvm_t m_tBuzzVM) {
  buzzvm_push(m_tBuzzVM, t_table);
  buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, str_key.c_str(), 1));
  buzzvm_pushf(m_tBuzzVM, f_value);
  buzzvm_tput(m_tBuzzVM);
  return m_tBuzzVM->state;
}

/*
/ Utility function to push a table with int
----------------------------------------------------------- */

buzzvm_state TablePut_int_fval(buzzobj_t t_table,
                               int nidx,
                               float f_value,
                               buzzvm_t m_tBuzzVM) {
  buzzvm_push(m_tBuzzVM, t_table);
  buzzvm_pushi(m_tBuzzVM, nidx);
  buzzvm_pushf(m_tBuzzVM, f_value);
  buzzvm_tput(m_tBuzzVM);
  return m_tBuzzVM->state;
}

/*
/ Utility function to push 3d vectors with string id
----------------------------------------------------------- */

buzzvm_state TablePut_str_3dvec(buzzobj_t t_table,
                                       const std::string& str_key,
                                       std::vector<float> &vec3d,
                                       buzzvm_t m_tBuzzVM) {
   buzzvm_push(m_tBuzzVM, t_table);
   buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, str_key.c_str(), 1));
   buzzvm_pusht(m_tBuzzVM);
   buzzobj_t tVecTable = buzzvm_stack_at(m_tBuzzVM, 1);
   buzzvm_tput(m_tBuzzVM);
   TablePut_str_fval(tVecTable, "x", vec3d[0], m_tBuzzVM);
   TablePut_str_fval(tVecTable, "y", vec3d[1], m_tBuzzVM);
   TablePut_str_fval(tVecTable, "z", vec3d[2], m_tBuzzVM);
   return m_tBuzzVM->state;
}

/*
/ Utility function to push 3d vectors with int id
----------------------------------------------------------- */

buzzvm_state TablePut_int_vec3d(buzzobj_t t_table,
                                       int n_idx,
                                       std::vector<float> &vec3d,
                                       buzzvm_t m_tBuzzVM) {
   buzzvm_push(m_tBuzzVM, t_table);
   buzzvm_pushi(m_tBuzzVM, n_idx);
   buzzvm_pusht(m_tBuzzVM);
   buzzobj_t tVecTable = buzzvm_stack_at(m_tBuzzVM, 1);
   buzzvm_tput(m_tBuzzVM);
   TablePut_str_fval(tVecTable, "x", vec3d[0], m_tBuzzVM);
   TablePut_str_fval(tVecTable, "y", vec3d[1], m_tBuzzVM);
   TablePut_str_fval(tVecTable, "z", vec3d[2], m_tBuzzVM);
   return m_tBuzzVM->state;
}



/****************************************/
/****************************************/

int buzzros_print(buzzvm_t vm)
/*
/ Buzz closure to print out
----------------------------------------------------------- */
{
  std::ostringstream buffer(std::ostringstream::ate);
  buffer << "[" << buzz_utility::get_robotid() << "] ";
  for (uint32_t index = 1; index < buzzdarray_size(vm->lsyms->syms); ++index)
  {
    buzzvm_lload(vm, index);
    buzzobj_t o = buzzvm_stack_at(vm, 1);
    buzzvm_pop(vm);
    switch (o->o.type)
    {
      case BUZZTYPE_NIL:
        buffer << " BUZZ - [nil]";
        break;
      case BUZZTYPE_INT:
        buffer << " " << o->i.value;
        break;
      case BUZZTYPE_FLOAT:
        buffer << " " << o->f.value;
        break;
      case BUZZTYPE_TABLE:
        buffer << " [table with " << buzzdict_size(o->t.value) << " elems]";
        break;
      case BUZZTYPE_CLOSURE:
        if (o->c.value.isnative)
        {
          buffer << " [n-closure @" << o->c.value.ref << "]";
        }
        else
        {
          buffer << " [c-closure @" << o->c.value.ref << "]";
        }
        break;
      case BUZZTYPE_STRING:
        buffer << "  " << o->s.value.str;
        break;
      case BUZZTYPE_USERDATA:
        buffer << " [userdata @" << o->u.value << "]";
        break;
      default:
        break;
    }
  }
  ROS_INFO("%s", buffer.str().c_str());
  return buzzvm_ret0(vm);
}

void setCurVm(buzzvm_t vm){
  cur_BuzzVM = vm;
}

void setWPlist(string file)
/*
/ set the absolute path for a csv list of waypoints
----------------------------------------------------------- */
{
  WPlistname = file;
  parse_gpslist();
}

void setVorlog(string path)
/*
/ set the absolute path for a csv list of waypoints
----------------------------------------------------------- */
{
  voronoicsv.open(path + "/log/voronoi_" + std::to_string(buzz_utility::get_robotid()) + ".csv",
                  std::ios_base::trunc | std::ios_base::out);
  logVoronoi = true;
}

float constrainAngle(float x)
/*
/ Wrap the angle between -pi, pi
----------------------------------------------------------- */
{
  x = fmod(x + M_PI, 2 * M_PI);
  if (x < 0.0)
    x += 2 * M_PI;
  return x - M_PI;
}

void rb_from_gps(double nei[], double out[], double cur[], double ned[])
/*
/ Compute Range and Bearing from 2 GPS set of coordinates
/----------------------------------------*/
{
  double d_lon = nei[1] - cur[1];
  double d_lat = nei[0] - cur[0];
  ned[0] = DEG2RAD(d_lat) * EARTH_RADIUS;
  ned[1] = DEG2RAD(d_lon) * EARTH_RADIUS * cos(DEG2RAD(nei[0]));
  out[0] = sqrt(ned[0] * ned[0] + ned[1] * ned[1]);
  out[1] = constrainAngle(atan2(ned[1], ned[0]));
  out[2] = 0.0;
}

void gps_from_vec(double vec[], double gps[])
{
  double Vrange = sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
  double Vbearing = constrainAngle(atan2(vec[1], vec[0]));
  double latR = cur_pos[0] * M_PI / 180.0;
  double lonR = cur_pos[1] * M_PI / 180.0;
  double target_lat =
      asin(sin(latR) * cos(Vrange / EARTH_RADIUS) + cos(latR) * sin(Vrange / EARTH_RADIUS) * cos(Vbearing));
  double target_lon = lonR + atan2(sin(Vbearing) * sin(Vrange / EARTH_RADIUS) * cos(latR),
                                   cos(Vrange / EARTH_RADIUS) - sin(latR) * sin(target_lat));
  gps[0] = target_lat * 180.0 / M_PI;
  gps[1] = target_lon * 180.0 / M_PI;
  gps[2] = cur_pos[2];
}

void parse_gpslist()
/*
/ parse a csv of GPS targets
/----------------------------------------*/
{
  // Open file:
  ROS_INFO("WP list file: %s", WPlistname.c_str());
  std::ifstream fin(WPlistname.c_str());  // Open in text-mode.

  // Opening may fail, always check.
  if (!fin)
  {
    ROS_ERROR("GPS list parser, could not open file.");
    return;
  }

  // Prepare a C-string buffer to be used when reading lines.
  const int MAX_LINE_LENGTH = 1024;  // Choose this large enough for your need.
  char buffer[MAX_LINE_LENGTH] = {};
  const char* DELIMS = "\t ,";  // Tab, space or comma.

  // Read the file and load the data:
  buzz_utility::RB_struct RB_arr;
  // Read one line at a time.
  while (fin.getline(buffer, MAX_LINE_LENGTH))
  {
    // Extract the tokens:
    int tid = atoi(strtok(buffer, DELIMS));
    double lon = atof(strtok(NULL, DELIMS));
    double lat = atof(strtok(NULL, DELIMS));
    int alt = atoi(strtok(NULL, DELIMS));
    int tilt = atoi(strtok(NULL, DELIMS));
    //  DEBUG
    // ROS_INFO("%.6f, %.6f, %i %i %i",lat, lon, alt, tilt, tid);
    RB_arr.latitude = lat;
    RB_arr.longitude = lon;
    RB_arr.altitude = alt;
    RB_arr.r = tilt;
    // Insert elements.
    map<int, buzz_utility::RB_struct>::iterator it = wplist_map.find(tid);
    if (it != wplist_map.end())
      wplist_map.erase(it);
    wplist_map.insert(make_pair(tid, RB_arr));
  }

  //DEBUG
  //ROS_INFO("----->Saved %i waypoints.", wplist_map.size());

  // Close the file:
  fin.close();
}

void check_targets_sim(double lat, double lon, double* res)
/*
/ check if a listed target is close
----------------------------------------------------------- */
{
  float visibility_radius_bounds[2] = {0.25, 3.0};
  map<int, buzz_utility::RB_struct>::iterator itnei;
  for (itnei = neighbors_map.begin(); itnei != neighbors_map.end(); ++itnei)
  {
    map<int, buzz_utility::RB_struct>::iterator itneip = neighbors_map_prev.find(itnei->first);
    if (itneip == neighbors_map_prev.end())
      break;
    double vel = abs(itnei->second.r - itneip->second.r)*20; //dRANGE * BUZZRATE
    double radtmp = visibility_radius_bounds[1]-vel*vel*(visibility_radius_bounds[1]-visibility_radius_bounds[0]);
    if(radtmp<visibility_radius_bounds[0])
      radtmp=visibility_radius_bounds[0];
    map<int, buzz_utility::RB_struct>::iterator itt;
    for (itt = wplist_map.begin(); itt != wplist_map.end(); ++itt)
    {
      double rb[3], ned[2];
      double ref[2] = { itnei->second.latitude, itnei->second.longitude };
      double tar[2] = { itt->second.latitude, itt->second.longitude };
      rb_from_gps(tar, rb, ref, ned);
      if (rb[0] < radtmp && (buzz_utility::get_bvmstate() == "WAYPOINT" && itt->second.r == 0))
      {
        //DEBUG
        if(buzz_utility::get_robotid()==1){
          ROS_WARN("[%i] TARGET FOUND: %f (%f|%f)", itnei->first, rb[0], radtmp, vel);
        }
        res[0] = itt->first;
        res[1] = itt->second.latitude;
        res[2] = itt->second.longitude;
        res[3] = itt->second.altitude;
      }
      else if (rb[0] < radtmp && (buzz_utility::get_bvmstate() == "DEPLOY" && itt->second.r == 1))
      {
        //DEBUG
        //ROS_WARN("FOUND A TARGET IN WAYPOINT!!! [%i]", itt->first);
        res[0] = itt->first;
        res[1] = itt->second.latitude;
        res[2] = itt->second.longitude;
        res[3] = itt->second.altitude;
      }
    }
  }
}

int buzz_exportmap(buzzvm_t vm)
/*
/ Buzz closure to export a 2D map
/----------------------------------------*/
{
  grid.clear();
  buzzvm_lnum_assert(vm, 4);
  // Get the parameter
  buzzvm_lload(vm, 1);  // grid
  buzzvm_lload(vm, 2);  // origin_y
  buzzvm_lload(vm, 3);  // origin_x
  buzzvm_lload(vm, 4);  // resolution
  buzzvm_type_assert(vm, 4, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);  // dictionary
  resolution = buzzvm_stack_at(vm, 4)->f.value;
  origin_x = buzzvm_stack_at(vm, 3)->f.value;
  origin_y = buzzvm_stack_at(vm, 2)->f.value;
  buzzobj_t t = buzzvm_stack_at(vm, 1);
  for (int32_t i = 1; i <= buzzdict_size(t->t.value); ++i)
  {
    buzzvm_dup(vm);
    buzzvm_pushi(vm, i);
    buzzvm_tget(vm);
    std::map<int, int> row;
    for (int32_t j = 1; j <= buzzdict_size(buzzvm_stack_at(vm, 1)->t.value); ++j)
    {
      buzzvm_dup(vm);
      buzzvm_pushi(vm, j);
      buzzvm_tget(vm);
      row.insert(std::pair<int, int>(j, 100.0 - round(buzzvm_stack_at(vm, 1)->f.value * 100.0)));
      buzzvm_pop(vm);
    }
    grid.insert(std::pair<int, std::map<int, int>>(i, row));
    buzzvm_pop(vm);
  }
  // DEBUG
  // ROS_INFO("----- Recorded a grid of %i(%i)", grid.size(), buzzdict_size(t->t.value));
  return buzzvm_ret0(vm);
}

int buzz_exportpath(buzzvm_t vm)
/*
/ Buzz closure to export a 2D path
/----------------------------------------*/
{
  path.clear();
  buzzvm_lnum_assert(vm, 1);
  // Get the parameter
  buzzvm_lload(vm, 1);
  buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);  // dictionary
  buzzobj_t t = buzzvm_stack_at(vm, 1);
  int x = 0, y = 0, ts = buzzdict_size(t->t.value);
  for (int32_t i = 1; i <= ts; ++i)
  {
    buzzvm_dup(vm);
    buzzvm_pushi(vm, i);
    buzzvm_tget(vm);

    buzzvm_dup(vm);
    buzzvm_pushi(vm, 1);
    buzzvm_tget(vm);
    x = buzzvm_stack_at(vm, 1)->i.value;
    buzzvm_pop(vm);

    buzzvm_dup(vm);
    buzzvm_pushi(vm, 2);
    buzzvm_tget(vm);
    y = buzzvm_stack_at(vm, 1)->i.value;
    buzzvm_pop(vm);
    
    path.insert(std::pair<int, std::pair<int, int>>(i, std::pair<int, int>(x, y)));
    buzzvm_pop(vm);
  }
  // DEBUG
  // std::map<int, std::pair<int, int>>::iterator itr = path.begin();
  // for (itr = path.begin(); itr != path.end(); ++itr)
  // {
  //   ROS_INFO("----- Path[%i] = %i, %i", itr->first, itr->second.first, itr->second.second);
  // }
  return buzzvm_ret0(vm);
}

int buzz_exportFollowerStatus(buzzvm_t vm)
/*
/ Buzz closure to export the follower status in the Hierarchical Swarm
/---------------------------------------------------------------------*/
{
  buzzvm_lnum_assert(vm, 5);
  buzzvm_lload(vm, 1);  // pull
  buzzvm_lload(vm, 2);  // dog_reactive_field
  buzzvm_lload(vm, 3);  // field_of_view
  buzzvm_lload(vm, 4);  // mode
  buzzvm_lload(vm, 5);  // density
  buzzvm_type_assert(vm, 5, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 4, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 3, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_INT);
  hierarchical_status[0] = buzzvm_stack_at(vm, 5)->i.value;
  hierarchical_status[1] = buzzvm_stack_at(vm, 4)->i.value;
  hierarchical_status[2] = buzzvm_stack_at(vm, 3)->i.value;
  hierarchical_status[3] = buzzvm_stack_at(vm, 2)->i.value;
  hierarchical_status[4] = buzzvm_stack_at(vm, 1)->i.value;

  return buzzvm_ret0(vm);
}

int buzz_exportGuideState(buzzvm_t vm)
/*
/ Buzz closure to export the guide state in the Hierarchical Swarm
/---------------------------------------------------------------------*/
{
  // Ids of the two guide robots
  int guide_1_id = 1;
  int guide_2_id = 2;

  buzzvm_lnum_assert(vm, 2);
  buzzvm_lload(vm, 1);  // state
  buzzvm_lload(vm, 2);  // id
  buzzvm_type_assert(vm, 2, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_INT);
  int id = buzzvm_stack_at(vm, 2)->i.value;
  int state = buzzvm_stack_at(vm, 1)->i.value;

  if (id == guide_1_id)
    hierarchical_status[5] = state;
  else if (id == guide_2_id)
    hierarchical_status[6] = state;
  else
    ROS_INFO("Unknown robot id received by guide state closure!");

  return buzzvm_ret0(vm);
}

/*
 *  Geofence(): test for a point in a polygon
 *     TAKEN from https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
 */

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(Point p, Point q, Point r)
{
  if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
    return true;
  return false;
}
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
  int val = round((q.y - p.y) * (r.x - q.x) * 100 - (q.x - p.x) * (r.y - q.y) * 100);

  if (val == 0)
    return 0;                // colinear
  return (val > 0) ? 1 : 2;  // clock or counterclock wise
}
// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
  // Find the four orientations needed for general and
  // special cases
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  // ROS_WARN("(%f,%f)->(%f,%f), 1:%d,2:%d,3:%d,4:%d",p1.x,p1.y,q1.x,q1.y,o1,o2,o3,o4);

  // General case
  if (o1 != o2 && o3 != o4)
    return true;

  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == 0 && onSegment(p1, p2, q1))
    return true;

  // p1, q1 and p2 are colinear and q2 lies on segment p1q1
  if (o2 == 0 && onSegment(p1, q2, q1))
    return true;

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == 0 && onSegment(p2, p1, q2))
    return true;

  // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == 0 && onSegment(p2, q1, q2))
    return true;

  return false;  // Doesn't fall in any of the above cases
}

float clockwise_angle_of(const Point& p)
{
  return atan2(p.y, p.x);
}

bool clockwise_compare_points(const Point& a, const Point& b)
{
  return clockwise_angle_of(a) < clockwise_angle_of(b);
}

void sortclose_polygon(vector<Point>* P)
{
  std::sort(P->begin(), P->end(), clockwise_compare_points);
  P->push_back((*P)[0]);
}

float pol_area(vector<Point> vert)
{
  float a = 0.0;
  // ROS_INFO("Polygone %d edges area.",vert.size());
  vector<Point>::iterator it;
  vector<Point>::iterator next;
  for (it = vert.begin(); it != vert.end() - 1; ++it)
  {
    next = it + 1;
    a += it->x * next->y - next->x * it->y;
  }
  a *= 0.5;
  // ROS_INFO("Polygon area: %f",a);
  return a;
}

double* polygone_center(vector<Point> vert, double* c)
{
  float A = pol_area(vert);
  int i1 = 1;
  vector<Point>::iterator it;
  vector<Point>::iterator next;
  for (it = vert.begin(); it != vert.end() - 1; ++it)
  {
    next = it + 1;
    float t = it->x * next->y - next->x * it->y;
    c[0] += (it->x + next->x) * t;
    c[1] += (it->y + next->y) * t;
  }
  c[0] = c[0] / (6.0 * A);
  c[1] = c[1] / (6.0 * A);
  return c;
}

double numerator(Point A, Point C, Point E, Point F)
{
  return (A.y - C.y) * (F.x - E.x) - (A.x - C.x) * (F.y - E.y);
}
double denominator(Point A, Point B, Point C, Point D)
{
  return (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);
}

void getintersection(Point S, Point D, std::vector<Point> Poly, Point* I)
{
  // printf("Points for intersection 1(%f,%f->%f,%f) and 2(%f,%f->%f,%f)\n",q1.x,q1.y,p1.x,p1.y,q2.x,q2.y,p2.x,p2.y);
  bool parallel = false;
  bool collinear = false;
  std::vector<Point>::iterator itc;
  std::vector<Point>::iterator next;
  for (itc = Poly.begin(); itc != Poly.end() - 1; ++itc)
  {
    next = itc + 1;
    if (doIntersect((*itc), (*next), S, D))
    {
      // Uses the determinant of the two lines. For more information, refer to one of the following:
      // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
      // http://www.faqs.org/faqs/graphics/algorithms-faq/ (Subject 1.03)

      double d = denominator(S, D, (*itc), (*next));

      if (std::abs(d) < 0.000000001)
      {
        parallel = true;
        collinear = abs(numerator(S, D, (*itc), (*next))) < 0.000000001;
        return;
      }

      double r = numerator(S, (*itc), (*itc), (*next)) / d;
      double s = numerator(S, (*itc), S, D) / d;

      // ROS_INFO("-- (%f,%f)",S.x + r * (D.x - S.x), S.y + r * (D.y - S.y));
      (*I) = Point(S.x + r * (D.x - S.x), S.y + r * (D.y - S.y));
    }
  }
  if (parallel || collinear)
    ROS_WARN("Lines are Collinear (%d) or Parallels (%d)", collinear, parallel);
}

bool isSiteout(Point S, std::vector<Point> Poly)
{
  bool onedge = false;

  // Create a point for line segment from p to infinite
  Point extreme = { 10000, S.y };

  // Count intersections of the above line with sides of polygon
  int count = 0;
  std::vector<Point>::iterator itc;
  std::vector<Point>::iterator next;
  for (itc = Poly.begin(); itc != Poly.end() - 1; ++itc)
  {
    next = itc + 1;

    // Check if the line segment from 'p' to 'extreme' intersects
    // with the line segment from 'polygon[i]' to 'polygon[next]'
    if (doIntersect((*itc), (*next), S, extreme))
    {
      // If the point 'p' is colinear with line segment 'i-next',
      // then check if it lies on segment. If it lies, return true,
      // otherwise false
      if (orientation((*itc), S, (*next)) == 0)
      {
        onedge = onSegment((*itc), S, (*next));
        if (onedge)
          break;
      }
      count++;
    }
  }

  return ((count % 2 == 0) && !onedge);
}

int buzzuav_geofence(buzzvm_t vm)
{
  Point P;
  buzzvm_lnum_assert(vm, 1);
  // Get the parameter
  buzzvm_lload(vm, 1);
  buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);  // dictionary
  buzzobj_t t = buzzvm_stack_at(vm, 1);

  if (buzzdict_size(t->t.value) < 5)
  {
    ROS_ERROR("Wrong Geofence input size (%i).", buzzdict_size(t->t.value));
    return buzzvm_ret0(vm);
  }
  int size_to = buzzdict_size(t->t.value);
  int local_fencing = 0;
  if(buzzdict_size(t->t.value) > 5){
    size_to = buzzdict_size(t->t.value)-1;
    local_fencing = 1;
  } 
  std::vector<Point> polygon_bound;
  for (int32_t i = 0; i < size_to; ++i)
  {
    Point tmp;
    buzzvm_dup(vm);
    buzzvm_pushi(vm, i);
    buzzvm_tget(vm);

    buzzvm_dup(vm);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "x", 1));
    buzzvm_tget(vm);
    if (i == 0)
    {
      P.x = buzzvm_stack_at(vm, 1)->f.value;
      // printf("px=%f\n",P.x);
    }
    else
    {
      tmp.x = buzzvm_stack_at(vm, 1)->f.value;
      // printf("c%dx=%f\n",i,tmp.x);
    }
    buzzvm_pop(vm);
    buzzvm_dup(vm);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "y", 1));
    buzzvm_tget(vm);
    // ROS_INFO("[%i]---y-->%i",buzz_utility::get_robotid(), tmp);
    if (i == 0)
    {
      P.y = buzzvm_stack_at(vm, 1)->f.value;
      // printf("py=%f\n",P.y);
    }
    else
    {
      tmp.y = buzzvm_stack_at(vm, 1)->f.value;
      // printf("c%dy=%f\n",i,tmp.y);
    }
    buzzvm_pop(vm);

    if (i != 0)
      polygon_bound.push_back(tmp);

    buzzvm_pop(vm);
  }
  sortclose_polygon(&polygon_bound);

  // Check if we are in the zone
  if (isSiteout(P, polygon_bound))
  {
    // We are! Set a new goal just before the fence.
    Point Intersection;
    getintersection(Point(0.0, 0.0), P, polygon_bound, &Intersection);
    double gps[3];
    double d[2] = { Intersection.x - sign(Intersection.x)*1.5, Intersection.y - sign(Intersection.y)*1.5 };
    gps_from_vec(d, gps);
    if(local_fencing){
      // Update the navigation command 
      /* Create empty data table */
      buzzvm_pushs(vm, buzzvm_string_register(vm, "m_navigation", 1));
      buzzvm_pusht(vm);
      buzzobj_t path_Pose_table = buzzvm_stack_at(vm, 1);
      buzzvm_gstore(vm);
      //  Fill in the new state
      buzzvm_push(vm, path_Pose_table);
      buzzvm_pushs(vm, buzzvm_string_register(vm, "x", 0));
      buzzvm_pushf(vm, d[0]);
      buzzvm_tput(vm);
      buzzvm_push(vm, path_Pose_table);
      buzzvm_pushs(vm, buzzvm_string_register(vm, "y", 0));
      buzzvm_pushf(vm, d[1]);
      buzzvm_tput(vm);
    }
    else{
      set_gpsgoal(gps);
    }
    ROS_WARN("Geofencing trigered, not going any further (%f,%f)!", d[0], d[1]);
  }

  return buzzvm_ret0(vm);
}

int voronoi_center(buzzvm_t vm)
{
  float dist_max = 300;

  buzzvm_lnum_assert(vm, 1);
  // Get the parameter
  buzzvm_lload(vm, 1);
  buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);  // dictionary
  buzzobj_t t = buzzvm_stack_at(vm, 1);

  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "np", 1));
  buzzvm_tget(vm);
  int Poly_vert = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);

  std::vector<Point> polygon_bound;
  for (int32_t i = 0; i < Poly_vert; ++i)
  {
    buzzvm_dup(vm);
    buzzvm_pushi(vm, i);
    buzzvm_tget(vm);

    buzzvm_dup(vm);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "x", 1));
    buzzvm_tget(vm);
    // ROS_INFO("---x-->%f",buzzvm_stack_at(vm, 1)->f.value);
    float tmpx = buzzvm_stack_at(vm, 1)->f.value;
    buzzvm_pop(vm);
    buzzvm_dup(vm);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "y", 1));
    buzzvm_tget(vm);
    // ROS_INFO("---y-->%f",buzzvm_stack_at(vm, 1)->f.value);
    float tmpy = buzzvm_stack_at(vm, 1)->f.value;
    buzzvm_pop(vm);

    polygon_bound.push_back(Point(tmpx, tmpy));
    // ROS_INFO("[%i] Polygon vertex: %f, %f", buzz_utility::get_robotid(),tmpx,tmpy);

    buzzvm_pop(vm);
  }
  sortclose_polygon(&polygon_bound);

  int count = buzzdict_size(t->t.value) - (Poly_vert + 1);

  // Check if we are in the zone
  if (isSiteout(Point(0, 0), polygon_bound) || count < 3)
  {
    // ROS_WARN("Not in the Zone!!!");
    double goal_tmp[2];
    do
    {
      goal_tmp[0] = polygon_bound[0].x + (rand() % 100) / 100.0 * (polygon_bound[2].x - polygon_bound[0].x);
      goal_tmp[1] = polygon_bound[0].y + (rand() % 100) / 100.0 * (polygon_bound[2].y - polygon_bound[0].y);
      // ROS_WARN(" in the Zone (%f,%f)!",goal_tmp[0],goal_tmp[1]);
    } while (isSiteout(Point(goal_tmp[0], goal_tmp[1]), polygon_bound));
    ROS_WARN("Sending at a random location in the Zone (%f,%f)!", goal_tmp[0], goal_tmp[1]);
    double gps[3];
    gps_from_vec(goal_tmp, gps);
    set_gpsgoal(gps);
    return buzzvm_ret0(vm);
  }

  ROS_WARN("NP: %d, Sites: %d", Poly_vert, count);
  float* xValues = new float[count];
  float* yValues = new float[count];
  for (int32_t i = 0; i < count; ++i)
  {
    int index = i + Poly_vert;
    buzzvm_dup(vm);
    buzzvm_pushi(vm, index);
    buzzvm_tget(vm);

    buzzvm_dup(vm);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "x", 1));
    buzzvm_tget(vm);
    // ROS_INFO("---x-->%f",buzzvm_stack_at(vm, 1)->f.value);
    xValues[i] = buzzvm_stack_at(vm, 1)->f.value;
    buzzvm_pop(vm);
    buzzvm_dup(vm);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "y", 1));
    buzzvm_tget(vm);
    // ROS_INFO("---y-->%f",buzzvm_stack_at(vm, 1)->f.value);
    yValues[i] = buzzvm_stack_at(vm, 1)->f.value;
    buzzvm_pop(vm);

    buzzvm_pop(vm);
  }

  VoronoiDiagramGenerator vdg;
  ROS_WARN("[%i] Voronoi Bounded tessellation starting with %i sites...", buzz_utility::get_robotid(), count);
  vdg.generateVoronoi(xValues, yValues, count, -dist_max, dist_max, -dist_max, dist_max, 3.0);
  if (logVoronoi)
    voronoicsv << ros::Time::now().toNSec() << ",";
  vdg.resetIterator();
  // ROS_WARN("[%i] Voronoi Bounded tessellation done!", buzz_utility::get_robotid());

  std::vector<Point>::iterator itc, next;
  for (itc = polygon_bound.begin(); itc != polygon_bound.end() - 1; ++itc)
  {
    next = itc + 1;
    if (logVoronoi)
      voronoicsv << itc->x << "," << itc->y << "," << next->x << "," << next->y << "," << 0 << "," << 0 << ",";
  }

  float x1, y1, x2, y2;
  int s[2];
  vector<Point> cell_vert;
  Point Intersection;
  int i = 0;
  while (vdg.getNext(x1, y1, x2, y2, s))
  {
    // ROS_INFO("GOT Line (%f,%f)->(%f,%f) between sites %d,%d",x1,y1,x2,y2,s[0],s[1]);
    if (sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)) < 0.1)
      continue;
    bool isout1 = isSiteout(Point(x1, y1), polygon_bound);
    bool isout2 = isSiteout(Point(x2, y2), polygon_bound);
    if (isout1 && isout2)
    {
      // ROS_INFO("Line out of area!");
      continue;
    }
    else if (isout1)
    {
      getintersection(Point(x2, y2), Point(x1, y1), polygon_bound, &Intersection);
      x1 = Intersection.x;
      y1 = Intersection.y;
      // ROS_INFO("Site out 1 -> (%f,%f)", x1, y1);
    }
    else if (isout2)
    {
      getintersection(Point(x1, y1), Point(x2, y2), polygon_bound, &Intersection);
      x2 = Intersection.x;
      y2 = Intersection.y;
      // ROS_INFO("Site out 2 -> (%f,%f)", x2, y2);
    }
    if (logVoronoi)
      voronoicsv << x1 << "," << y1 << "," << x2 << "," << y2 << "," << s[0] << "," << s[1] << ",";
    i++;
    if ((s[0] == 0 || s[1] == 0))
    {
      if (cell_vert.empty())
      {
        cell_vert.push_back(Point(x1, y1));
        cell_vert.push_back(Point(x2, y2));
      }
      else
      {
        bool alreadyin = false;
        vector<Point>::iterator itc;
        for (itc = cell_vert.begin(); itc != cell_vert.end(); ++itc)
        {
          double dist = sqrt((itc->x - x1) * (itc->x - x1) + (itc->y - y1) * (itc->y - y1));
          if (dist < 0.1)
          {
            alreadyin = true;
            break;
          }
        }
        if (!alreadyin)
          cell_vert.push_back(Point(x1, y1));
        alreadyin = false;
        for (itc = cell_vert.begin(); itc != cell_vert.end(); ++itc)
        {
          double dist = sqrt((itc->x - x2) * (itc->x - x2) + (itc->y - y2) * (itc->y - y2));
          if (dist < 0.1)
          {
            alreadyin = true;
            break;
          }
        }
        if (!alreadyin)
          cell_vert.push_back(Point(x2, y2));
      }
    }
  }
  if (cell_vert.size() < 3)
  {
    ROS_WARN("[%i] Voronoi Bounded tessellation failed (%d)!", buzz_utility::get_robotid(), cell_vert.size());
    delete xValues;
    delete yValues;
    return buzzvm_ret0(vm);
  }
  std::sort(cell_vert.begin(), cell_vert.end(), clockwise_compare_points);
  cell_vert.push_back(cell_vert[0]);

  double center_dist[2] = { 0.0, 0.0 };
  polygone_center(cell_vert, center_dist);
  if (logVoronoi)
    voronoicsv << center_dist[0] << "," << center_dist[1] << std::endl;
  center_dist[0] /= 2;
  center_dist[1] /= 2;
  double gps[3];
  gps_from_vec(center_dist, gps);
  // ROS_INFO("[%i] Voronoi cell center: %f, %f, %f, %f", buzz_utility::get_robotid(), center_dist[0], center_dist[1],
  // gps[0], gps[1]);
  set_gpsgoal(gps);

  delete xValues;
  delete yValues;
  return buzzvm_ret0(vm);
}

int buzzuav_moveto(buzzvm_t vm)
/*
/ Buzz closure to move following a 3D vector + Yaw
/----------------------------------------*/
{
  buzzvm_lnum_assert(vm, 4);
  buzzvm_lload(vm, 1);  // dx
  buzzvm_lload(vm, 2);  // dy
  buzzvm_lload(vm, 3);  // dheight
  buzzvm_lload(vm, 4);  // dyaw
  buzzvm_type_assert(vm, 4, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  float dyaw = buzzvm_stack_at(vm, 1)->f.value;
  float dh = buzzvm_stack_at(vm, 2)->f.value;
  float dy = buzzvm_stack_at(vm, 3)->f.value;
  float dx = buzzvm_stack_at(vm, 4)->f.value;
  goto_pos[0] = dx;
  goto_pos[1] = dy;
  goto_pos[2] = height + dh;
  goto_pos[3] = dyaw;
  //  DEBUG
  // ROS_WARN("[%i] Buzz requested Move To: x: %.7f , y: %.7f, z: %.7f", (int)buzz_utility::get_robotid(), goto_pos[0],
  // goto_pos[1], goto_pos[2]);
  buzz_cmd = NAV_SPLINE_WAYPOINT;
  return buzzvm_ret0(vm);
}

int buzzuav_addtargetRB(buzzvm_t vm)
/*
/ Buzz closure to add a target (goal) GPS
/----------------------------------------*/
{
  buzzvm_lnum_assert(vm, 3);
  buzzvm_lload(vm, 1);  // longitude
  buzzvm_lload(vm, 2);  // latitude
  buzzvm_lload(vm, 3);  // id
  buzzvm_type_assert(vm, 3, BUZZTYPE_INT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  double tmp[3];
  tmp[0] = buzzvm_stack_at(vm, 2)->f.value;
  tmp[1] = buzzvm_stack_at(vm, 1)->f.value;
  tmp[2] = 0.0;
  int uid = buzzvm_stack_at(vm, 3)->i.value;
  double rb[3], ned[2];

  rb_from_gps(tmp, rb, cur_pos, ned);
  if (fabs(rb[0]) < 100.0)
  {
    buzz_utility::RB_struct RB_arr;
    RB_arr.latitude = tmp[0];
    RB_arr.longitude = tmp[1];
    RB_arr.altitude = tmp[2];
    RB_arr.r = rb[0];
    RB_arr.b = rb[1];
    map<int, buzz_utility::RB_struct>::iterator it = targets_map.find(uid);
    if (it != targets_map.end())
      targets_map.erase(it);
    targets_map.insert(make_pair(uid, RB_arr));
    //  DEBUG
    // ROS_INFO("Buzz_utility got updated/new user: %i (%f,%f,%f)", id, latitude, longitude, altitude);
    return vm->state;
  }
  else
    ROS_WARN(" ---------- Target too far %f", rb[0]);

  return 0;
}

int buzzuav_get_nei_alt(buzzvm_t vm){
  // Load id
  buzzvm_lnum_assert(vm, 1);
  buzzvm_lload(vm, 1);  // id
  buzzvm_type_assert(vm, 1, BUZZTYPE_INT);
  int uid = buzzvm_stack_at(vm, 1)->i.value;

  // find altitude
  int alt = -1;
  map<int, buzz_utility::neighbors_status>::iterator it = neighbors_status_map.find(uid);
  if (it != neighbors_status_map.end()){
    alt = it->second.altitude;
  }
  //push altitude
  buzzvm_pop(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "altout", 1));
  buzzvm_pushi(vm, alt);
  buzzvm_gstore(vm);
  return buzzvm_ret0(vm);
}

int buzzuav_addNeiStatus(buzzvm_t vm)
/*
/ closure to add neighbors status to the BVM
/----------------------------------------*/
{
  buzzvm_lnum_assert(vm, 1);
  buzzvm_lload(vm, 1);  // state table
  buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);
  buzzobj_t t = buzzvm_stack_at(vm, 1);
  if (buzzdict_size(t->t.value) != 5)
  {
    ROS_ERROR("Wrong neighbor status size.");
    return buzzvm_ret0(vm);
  }

  buzz_utility::neighbors_status newRS;
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "id", 1));
  buzzvm_tget(vm);
  uint8_t id = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "ba", 1));
  buzzvm_tget(vm);
  newRS.batt_lvl = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "al", 1));
  buzzvm_tget(vm);
  newRS.altitude = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "xb", 1));
  buzzvm_tget(vm);
  newRS.xbee = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "st", 1));
  buzzvm_tget(vm);
  newRS.flight_status = buzzvm_stack_at(vm, 1)->i.value;
  buzzvm_pop(vm);

  map<int, buzz_utility::neighbors_status>::iterator it = neighbors_status_map.find(id);
  if (it != neighbors_status_map.end())
    neighbors_status_map.erase(it);
  neighbors_status_map.insert(make_pair(id, newRS));
  return buzzvm_ret0(vm);
}

mavros_msgs::Mavlink get_status()
/*
/ return neighbors status from BVM
/----------------------------------------*/
{
  mavros_msgs::Mavlink payload_out;
  map<int, buzz_utility::neighbors_status>::iterator it;
  for (it = neighbors_status_map.begin(); it != neighbors_status_map.end(); ++it)
  {
    payload_out.payload64.push_back(it->first);
    payload_out.payload64.push_back(it->second.altitude);
    payload_out.payload64.push_back(it->second.batt_lvl);
    payload_out.payload64.push_back(it->second.xbee);
    payload_out.payload64.push_back(it->second.flight_status);
  }
  //  Add Robot id and message number to the published message
  payload_out.sysid = (uint8_t)neighbors_status_map.size();

  return payload_out;
}

int planner_cmd(){
  buzzvm_pushs(cur_BuzzVM, buzzvm_string_register(cur_BuzzVM, "planner_cmd", 1));
  buzzvm_gload(cur_BuzzVM);
  buzzobj_t o = buzzvm_stack_at(cur_BuzzVM, 1);
  buzzvm_pop(cur_BuzzVM);
  int planner_cur_cmd = 0;
  if(o->o.type == BUZZTYPE_INT){
    planner_cur_cmd = o->i.value;
  }
  else if( o->o.type == BUZZTYPE_FLOAT){
    planner_cur_cmd = o->f.value;  
  }
  return planner_cur_cmd;
}

void update_uwb_anchor(int tag_id, float range){
  if(cur_BuzzVM){
    buzzvm_pushs(cur_BuzzVM, buzzvm_string_register(cur_BuzzVM, "uwb", 1));
    buzzvm_pusht(cur_BuzzVM);
    buzzobj_t tuwbTable = buzzvm_stack_at(cur_BuzzVM, 1);
    buzzvm_gstore(cur_BuzzVM);

    buzzvm_pusht(cur_BuzzVM);
    buzzobj_t tuwbRead = buzzvm_stack_at(cur_BuzzVM, 1);
    buzzvm_pop(cur_BuzzVM);
    //  Fill in the read
    buzzvm_push(cur_BuzzVM, tuwbRead);
    buzzvm_pushs(cur_BuzzVM, buzzvm_string_register(cur_BuzzVM, "range", 0));
    buzzvm_pushf(cur_BuzzVM, range);
    buzzvm_tput(cur_BuzzVM);
  
    //  Store read table in the proximity table
    buzzvm_push(cur_BuzzVM, tuwbTable);
    buzzvm_pushi(cur_BuzzVM, tag_id);
    buzzvm_push(cur_BuzzVM, tuwbRead);
    buzzvm_tput(cur_BuzzVM);
  }
}

int buzzuav_resetrc(buzzvm_t vm)
/*
/ Buzz closure to reset the RC input.
/----------------------------------------*/
{
  rc_id = -1;
  return buzzvm_ret0(vm);
}

int buzzuav_takepicture(buzzvm_t vm)
/*
/ Buzz closure to take a picture here.
/----------------------------------------*/
{
  buzz_cmd = IMAGE_START_CAPTURE;
  return buzzvm_ret0(vm);
}

/*
/ Buzz closure to do exploration.
/----------------------------------------*/
int buzzuav_call_local_planner(buzzvm_t vm)

{
  exploration_planner_cmd = EXPLORE;
  return buzzvm_ret0(vm);
}

std::vector<float> get_home_location_setter_status()
{
  std::vector<float> ret_val;
  ret_val.push_back(planner_home_location[0]);
  ret_val.push_back(planner_home_location[1]);
  ret_val.push_back(planner_home_location[2]);
  ret_val.push_back(planner_home_location[3]);
  ret_val.push_back(planner_home_location[4]);

  planner_home_location[0] = 0.0;
  planner_home_location[1] = 0.0;
  planner_home_location[2] = 0.0;
  planner_home_location[3] = 0.0;
  planner_home_location[4] = 0.0;

  return ret_val;
}

int buzzuav_buzz_CallPlannerHomeLocation(buzzvm_t vm)
{
  buzzvm_lnum_assert(vm, 4);
  /* Push the color components */
  buzzvm_lload(vm, 1);
  buzzvm_lload(vm, 2);
  buzzvm_lload(vm, 3);
  buzzvm_lload(vm, 4);
  buzzvm_type_assert(vm, 4, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  planner_home_location[0] = buzzvm_stack_at(vm, 4)->f.value;
  planner_home_location[1] = buzzvm_stack_at(vm, 3)->f.value;
  planner_home_location[2] = buzzvm_stack_at(vm, 2)->f.value;
  planner_home_location[3] = buzzvm_stack_at(vm, 1)->f.value;
  planner_home_location[4] = 1;
  return buzzvm_ret0(vm);
}

/*
/ Buzz closure to do get the global planner path with nav tube.
/----------------------------------------*/
int buzzuav_call_global_planner_for_base_paths(buzzvm_t vm)

{
  exploration_planner_cmd = GLOBAL_HOME_WITH_TUB;
  return buzzvm_ret0(vm);
}


/*
/ Buzz closure to get home path.
/----------------------------------------*/
int buzzuav_call_global_planner(buzzvm_t vm)

{
  exploration_planner_cmd = GLOBAL_HOME;
  return buzzvm_ret0(vm);
}



int buzzuav_setgimbal(buzzvm_t vm)
/*
/ Buzz closure to change locally the gimbal orientation
/----------------------------------------*/
{
  buzzvm_lnum_assert(vm, 4);
  buzzvm_lload(vm, 1);  // time
  buzzvm_lload(vm, 2);  // pitch
  buzzvm_lload(vm, 3);  // roll
  buzzvm_lload(vm, 4);  // yaw
  buzzvm_type_assert(vm, 4, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  rc_gimbal[0] = buzzvm_stack_at(vm, 4)->f.value;
  rc_gimbal[1] = buzzvm_stack_at(vm, 3)->f.value;
  rc_gimbal[2] = buzzvm_stack_at(vm, 2)->f.value;
  rc_gimbal[3] = buzzvm_stack_at(vm, 1)->f.value;

  ROS_INFO("Set RC_GIMBAL ---- %f %f %f (%f)", rc_gimbal[0], rc_gimbal[1], rc_gimbal[2], rc_gimbal[3]);
  buzz_cmd = DO_MOUNT_CONTROL;
  return buzzvm_ret0(vm);
}

int buzzuav_storegoal(buzzvm_t vm)
/*
/ Buzz closure to store locally a GPS destination from the fleet
/----------------------------------------*/
{
  buzzvm_lnum_assert(vm, 3);
  buzzvm_lload(vm, 1);  // altitude
  buzzvm_lload(vm, 2);  // longitude
  buzzvm_lload(vm, 3);  // latitude
  buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  double goal[3];
  goal[0] = buzzvm_stack_at(vm, 3)->f.value;
  goal[1] = buzzvm_stack_at(vm, 2)->f.value;
  goal[2] = buzzvm_stack_at(vm, 1)->f.value;
  if (goal[0] == -1 && goal[1] == -1 && goal[2] == -1)
  {
    if (wplist_map.size() <= 0)
      parse_gpslist();
    goal[0] = wplist_map.begin()->second.latitude;
    goal[1] = wplist_map.begin()->second.longitude;
    goal[2] = wplist_map.begin()->second.altitude;
    wplist_map.erase(wplist_map.begin()->first);
  }

  set_gpsgoal(goal);
  // prevent an overwrite
  rc_id = -1;

  return buzzvm_ret0(vm);
}

void set_gpsgoal(double goal[3])
/*
/ update GPS goal value
-----------------------------------*/
{
  double rb[3], ned[2];
  rb_from_gps(goal, rb, cur_pos, ned);
  if (fabs(rb[0]) < 250.0)
  {
    goto_gpsgoal[0] = goal[0];
    goto_gpsgoal[1] = goal[1];
    goto_gpsgoal[2] = goal[2];
    ROS_INFO("[%i] Set GPS GOAL TO ---- %f %f %f (%f %f, %f %f)", buzz_utility::get_robotid(), goal[0], goal[1],
             goal[2], cur_pos[0], cur_pos[1], rb[0], rb[1]);
  }
  else
    ROS_WARN("[%i] GPS GOAL TOO FAR !!-- %f %f %f (%f %f, %f %f)", buzz_utility::get_robotid(), goal[0], goal[1],
             goal[2], cur_pos[0], cur_pos[1], rb[0], rb[1]);
}

int buzzuav_arm(buzzvm_t vm)
/*
/ Buzz closure to arm
/---------------------------------------*/
{
  cur_cmd = COMPONENT_ARM_DISARM;
  printf(" Buzz requested Arm \n");
  buzz_cmd = cur_cmd;
  return buzzvm_ret0(vm);
}

int buzzuav_disarm(buzzvm_t vm)
/*
/ Buzz closure to disarm
/---------------------------------------*/
{
  cur_cmd = COMPONENT_ARM_DISARM + 1;
  printf(" Buzz requested Disarm  \n");
  buzz_cmd = cur_cmd;
  return buzzvm_ret0(vm);
}

int buzzuav_takeoff(buzzvm_t vm)
/*
/ Buzz closure to takeoff
/---------------------------------------*/
{
  buzzvm_lnum_assert(vm, 1);
  buzzvm_lload(vm, 1); /* Altitude */
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  goto_pos[2] = buzzvm_stack_at(vm, 1)->f.value;
  height = goto_pos[2];
  cur_cmd = NAV_TAKEOFF;
  printf(" Buzz requested Take off !!! \n");
  buzz_cmd = cur_cmd;
  return buzzvm_ret0(vm);
}

int buzzuav_land(buzzvm_t vm)
/*
/ Buzz closure to land
/-------------------------------------------------------------*/
{
  cur_cmd = NAV_LAND;
  printf(" Buzz requested Land !!! \n");
  buzz_cmd = cur_cmd;
  return buzzvm_ret0(vm);
}

int buzzuav_gohome(buzzvm_t vm)
/*
/ Buzz closure to return Home
/-------------------------------------------------------------*/
{
  cur_cmd = NAV_RETURN_TO_LAUNCH;
  printf(" Buzz requested gohome !!! \n");
  buzz_cmd = cur_cmd;
  return buzzvm_ret0(vm);
}

double* getgoto()
/*
/ return the GPS goal
/-------------------------------------------------------------*/
{
  return goto_pos;
}

int* get_hierarchical_status()
/*
/ return the Hierarchical Swarm status
/-------------------------------------------------------------*/
{
  return hierarchical_status;
}

std::map<int, std::map<int, int>> getgrid()
/*
/ return the grid
/-------------------------------------------------------------*/
{
  return grid;
}

std::map<int, std::pair<int, int>> getpath()
/*
/ return the path
/-------------------------------------------------------------*/
{
  return path;
}

float get_origin_x()
/*
/ return the origin_x
/-------------------------------------------------------------*/
{
  return origin_x;
}

float get_origin_y()
/*
/ return the origin_y
/-------------------------------------------------------------*/
{
  return origin_y;
}

float get_resolution()
/*
/ return the resolution
/-------------------------------------------------------------*/
{
  return resolution;
}

float* getgimbal()
/*
/ return current gimbal commands
---------------------------------------*/
{
  return rc_gimbal;
}

int getcmd()
/*
/ return current mavros command to the FC
---------------------------------------*/
{
  return cur_cmd;
}

int bzz_cmd()
/*
/ return and clean the custom command from Buzz to the FC
----------------------------------------------------------*/
{
  int cmd = buzz_cmd;
  buzz_cmd = 0;
  return cmd;
}

int exploration_planner_cmd_get()

{
  int cmd = exploration_planner_cmd;
  exploration_planner_cmd=0;
  return cmd;
}

void rc_set_goto(int id, double latitude, double longitude, double altitude)
/*
/ update interface RC GPS goal input
-----------------------------------*/
{
  rc_id = id;
  rc_gpsgoal[0] = latitude;
  rc_gpsgoal[1] = longitude;
  rc_gpsgoal[2] = altitude;
}

void rc_set_gimbal(int id, float yaw, float roll, float pitch, float t)
/*
/ update interface RC gimbal control input
-----------------------------------*/
{
  rc_id = id;
  rc_gimbal[0] = yaw;
  rc_gimbal[1] = roll;
  rc_gimbal[2] = pitch;
  rc_gimbal[3] = t;
}

void rc_call(int rc_cmd_in)
/*
/ update interface RC command input
-----------------------------------*/
{
  rc_cmd = rc_cmd_in;
}

void set_obstacle_dist(float dist[])
/*
/ update interface proximity value array
-----------------------------------*/
{
  for (int i = 0; i < number_of_proximity; i++)
    obst[i] = dist[i];
}

void update_explore_path(std::vector<std::vector<float>> path)
/*
/ update explorer path
-----------------------------------*/
{
  exploration_path = path;
  printf("Updating Exploration path with %i points\n",exploration_path.size());
}

void update_home_path(std::vector<std::vector<float>> path)
/*
/ update the homing path
-----------------------------------*/
{
  homing_path = path;
  printf("Updating Homing path with %i points\n",homing_path.size());
}

void update_interpolation_path(std::vector<std::vector<float>> path)
/*
/ update the interpolation homing path
-----------------------------------*/
{
  interpolation_homing_path = path;
  printf("Updating interpolation Homing path with %i points\n",homing_path.size());
}

void update_nav_tube(std::vector<std::vector<float>> tube)
/*
/ update nav tube
-----------------------------------*/
{
  nav_tube = tube;
  printf("Updating nav tube with %i points\n",nav_tube.size()/2);
}


void set_battery(float voltage, float current, float remaining)
/*
/ update interface battery value array
-----------------------------------*/
{
  batt[0] = voltage;
  batt[1] = current;
  batt[2] = remaining;
}

void clear_planner_paths()
/*
 * clear all old paths from storage.
 ----------------------------------*/
{
  exploration_path.clear();
  homing_path.clear();
  nav_tube.clear();
}

int buzzuav_update_battery(buzzvm_t vm)
/*
/ update BVM battery table
-----------------------------------*/
{
  buzzvm_pushs(vm, buzzvm_string_register(vm, "battery", 1));
  buzzvm_pusht(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "voltage", 1));
  buzzvm_pushf(vm, batt[0]);
  // printf("the voltage from c code is: %f\n",batt[0]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "current", 1));
  buzzvm_pushf(vm, batt[1]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "capacity", 1));
  buzzvm_pushi(vm, (int)batt[2]);
  buzzvm_tput(vm);
  buzzvm_gstore(vm);
  return vm->state;
}

void store_bounding_boxes(std::vector<bounding_box> bbox)
{
  yolo_boxes.clear();
  for (int i = 0; i < bbox.size(); i++)
  {
    yolo_boxes.push_back(bbox[i]);
  }
}

int buzzuav_update_yolo_boxes(buzzvm_t vm)
{
  if (yolo_boxes.size() > 0)
  {
    buzzvm_pushs(vm, buzzvm_string_register(vm, "yolo_boxes", 1));
    buzzvm_pusht(vm);
    buzzvm_dup(vm);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "size", 1));
    buzzvm_pushf(vm, yolo_boxes.size());
    buzzvm_tput(vm);

    for (int i = 0; i < yolo_boxes.size(); i++)
    {
      buzzvm_dup(vm);
      std::string index = std::to_string(i);
      buzzvm_pushs(vm, buzzvm_string_register(vm, index.c_str(), 1));
      buzzvm_pusht(vm);
      buzzvm_dup(vm);
      buzzvm_pushs(vm, buzzvm_string_register(vm, "class", 1));
      buzzvm_pushs(vm, buzzvm_string_register(vm, yolo_boxes[i].obj_class.c_str(), 1));
      buzzvm_tput(vm);
      buzzvm_dup(vm);
      buzzvm_pushs(vm, buzzvm_string_register(vm, "probability", 1));
      buzzvm_pushf(vm, yolo_boxes[i].probability);
      buzzvm_tput(vm);
      buzzvm_dup(vm);
      buzzvm_pushs(vm, buzzvm_string_register(vm, "xmin", 1));
      buzzvm_pushf(vm, yolo_boxes[i].xmin);
      buzzvm_tput(vm);
      buzzvm_dup(vm);
      buzzvm_pushs(vm, buzzvm_string_register(vm, "xmax", 1));
      buzzvm_pushf(vm, yolo_boxes[i].xmax);
      buzzvm_tput(vm);
      buzzvm_dup(vm);
      buzzvm_pushs(vm, buzzvm_string_register(vm, "ymin", 1));
      buzzvm_pushf(vm, yolo_boxes[i].ymin);
      buzzvm_tput(vm);
      buzzvm_dup(vm);
      buzzvm_pushs(vm, buzzvm_string_register(vm, "ymax", 1));
      buzzvm_pushf(vm, yolo_boxes[i].ymax);
      buzzvm_tput(vm);
      buzzvm_tput(vm);
    }
    buzzvm_gstore(vm);
    yolo_boxes.clear();
  }
  return vm->state;
}

/*
/ Set of function to update interface variable of xbee network status
----------------------------------------------------------------------*/
void set_deque_full(bool state)
{
  deque_full = state;
}

void set_rssi(float value)
{
  rssi = round(value);
}

void set_raw_packet_loss(float value)
{
  raw_packet_loss = value;
}

void set_filtered_packet_loss(float value)
{
  filtered_packet_loss = round(100 * value);
}

/*void set_api_rssi(float value)
{
  api_rssi = value;
}*/

int buzzuav_update_xbee_status(buzzvm_t vm)
/*
/ update BVM xbee_status table
-----------------------------------*/
{
  buzzvm_pushs(vm, buzzvm_string_register(vm, "xbee_status", 1));
  buzzvm_pusht(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "deque_full", 1));
  buzzvm_pushi(vm, static_cast<uint8_t>(deque_full));
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "rssi", 1));
  buzzvm_pushi(vm, rssi);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "raw_packet_loss", 1));
  buzzvm_pushf(vm, raw_packet_loss);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "filtered_packet_loss", 1));
  buzzvm_pushi(vm, filtered_packet_loss);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "api_rssi", 1));
  buzzvm_pushf(vm, api_rssi);
  buzzvm_tput(vm);
  buzzvm_gstore(vm);
  return vm->state;
}

void set_currentNEDpos(double x, double y, double z, double yaw, double x_offset, double y_offset)
/*
/ update interface position array
-----------------------------------*/
{
  cur_NEDpos[0] = x;
  cur_NEDpos[1] = y;
  cur_NEDpos[2] = z;
  cur_NEDpos[3] = yaw;
  cur_pos[3] =yaw;
  cur_offset[0] = x_offset;
  cur_offset[1] = y_offset;
}

void set_currentpos(double latitude, double longitude, float altitude, float yaw)
/*
/ update interface position array
-----------------------------------*/
{
  cur_pos[0] = latitude;
  cur_pos[1] = longitude;
  cur_pos[2] = altitude;
  cur_pos[3] = yaw;
}
//  adds neighbours position
void neighbour_pos_callback(int id, float range, float bearing, float latitude, float longitude, float elevation)
{
  buzz_utility::RB_struct pos_arr(latitude, longitude, elevation, range, bearing);
;
  map<int, buzz_utility::RB_struct>::iterator itp = neighbors_map_prev.find(id);
  if (itp != neighbors_map_prev.end())
    neighbors_map_prev.erase(itp);
  map<int, buzz_utility::RB_struct>::iterator it = neighbors_map.find(id);
  if (it != neighbors_map.end()){
    neighbors_map_prev.insert(make_pair(it->first, it->second));
    neighbors_map.erase(it);
  }
  neighbors_map.insert(make_pair(id, pos_arr));
}

//  update at each step the VM table
void update_neighbors(buzzvm_t vm)
{
  //   Reset neighbor information
  buzzneighbors_reset(vm);

  //  Get robot id and update neighbor information
  map<int, buzz_utility::RB_struct>::iterator it;
  for (it = neighbors_map.begin(); it != neighbors_map.end(); ++it)
  {
    buzzneighbors_add(vm, it->first, (it->second).r, (it->second).b, (it->second).altitude);
  }
}

// Clear neighbours pos
void clear_neighbours_pos()
{
  neighbors_map.clear();
  neighbors_map_prev.clear();
}

int buzzuav_update_currentpos(buzzvm_t vm)
/*
/ Update the BVM position table
/------------------------------------------------------*/
{
  buzzvm_pushs(vm, buzzvm_string_register(vm, "pose", 1));
  buzzvm_pusht(vm);
  buzzobj_t tPoseTable = buzzvm_stack_at(vm, 1);
  buzzvm_gstore(vm);

  //  Create table for i-th read
  buzzvm_pusht(vm);
  buzzobj_t tPosition = buzzvm_stack_at(vm, 1);
  buzzvm_pop(vm);
  //  Fill in the read
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "latitude", 0));
  buzzvm_pushf(vm, cur_pos[0]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "longitude", 0));
  buzzvm_pushf(vm, cur_pos[1]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "altitude", 0));
  buzzvm_pushf(vm, cur_pos[2]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "x", 0));
  buzzvm_pushf(vm, cur_NEDpos[0]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "y", 0));
  buzzvm_pushf(vm, cur_NEDpos[1]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "z", 0));
  buzzvm_pushf(vm, cur_NEDpos[2]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "x_offset", 0));
  buzzvm_pushf(vm, cur_offset[0]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tPosition);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "y_offset", 0));
  buzzvm_pushf(vm, cur_offset[1]);
  buzzvm_tput(vm);
  //  Store read table in the proximity table
  buzzvm_push(vm, tPoseTable);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "position", 0));
  buzzvm_push(vm, tPosition);
  buzzvm_tput(vm);

  //  Create table for i-th read
  buzzvm_pusht(vm);
  buzzobj_t tOrientation = buzzvm_stack_at(vm, 1);
  buzzvm_pop(vm);
  //  Fill in the read
  buzzvm_push(vm, tOrientation);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "yaw", 0));
  buzzvm_pushf(vm, cur_pos[3]);
  buzzvm_tput(vm);
    buzzvm_push(vm, tOrientation);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "local_yaw", 0));
  buzzvm_pushf(vm, cur_NEDpos[3]);
  buzzvm_tput(vm);
  //  Store read table in the proximity table
  buzzvm_push(vm, tPoseTable);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "orientation", 0));
  buzzvm_push(vm, tOrientation);
  buzzvm_tput(vm);

  return vm->state;
}

void flight_status_update(uint8_t state)
/*
/ Update the interface status variable
/------------------------------------------------------*/
{
  status = state;
}

int buzzuav_update_flight_status(buzzvm_t vm)
/*
/ Create the generic robot table with status, remote controller current comand and destination
/ and current position of the robot
/------------------------------------------------------*/
{
  buzzvm_pushs(vm, buzzvm_string_register(vm, "flight", 1));
  buzzvm_pusht(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "rc_cmd", 1));
  buzzvm_pushi(vm, rc_cmd);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  rc_cmd = 0;
  buzzvm_pushs(vm, buzzvm_string_register(vm, "status", 1));
  buzzvm_pushi(vm, status);
  buzzvm_tput(vm);
  buzzvm_gstore(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "rc_goto", 1));
  buzzvm_pusht(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "id", 1));
  buzzvm_pushi(vm, rc_id);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "latitude", 1));
  buzzvm_pushf(vm, rc_gpsgoal[0]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "longitude", 1));
  buzzvm_pushf(vm, rc_gpsgoal[1]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "altitude", 1));
  buzzvm_pushf(vm, rc_gpsgoal[2]);
  buzzvm_tput(vm);
  buzzvm_gstore(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "cur_goal", 1));
  buzzvm_pusht(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "latitude", 1));
  buzzvm_pushf(vm, goto_gpsgoal[0]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "longitude", 1));
  buzzvm_pushf(vm, goto_gpsgoal[1]);
  buzzvm_tput(vm);
  buzzvm_dup(vm);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "altitude", 1));
  buzzvm_pushf(vm, goto_gpsgoal[2]);
  buzzvm_tput(vm);
  buzzvm_gstore(vm);
  return vm->state;
}

int buzzuav_update_kh4prox(buzzvm_t vm){
  buzzvm_pushs(vm, buzzvm_string_register(vm, "proximity", 1));
  buzzvm_pusht(vm);
  buzzobj_t tProxTable = buzzvm_stack_at(vm, 1);
  buzzvm_gstore(vm);

  //  Fill into the proximity table
  buzzobj_t tProxRead;
  float angle = 0;
  for (size_t i = 0; i < number_of_proximity; ++i)
  {
    //  Create table for i-th read
    buzzvm_pusht(vm);
    tProxRead = buzzvm_stack_at(vm, 1);
    buzzvm_pop(vm);
    //  Fill in the read
    buzzvm_push(vm, tProxRead);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "value", 0));
    buzzvm_pushf(vm, obst[i]);
    obst[i]=prox_init_val;
    buzzvm_tput(vm);
    buzzvm_push(vm, tProxRead);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "angle", 0));
    angle = (7 - i)*0.785398;
    buzzvm_pushf(vm, angle);
    buzzvm_tput(vm);
    //  Store read table in the proximity table
    buzzvm_push(vm, tProxTable);
    buzzvm_pushi(vm, i);
    buzzvm_push(vm, tProxRead);
    buzzvm_tput(vm);
  }
  return vm->state;
}

int buzzuav_update_prox(buzzvm_t vm)
/*
/ Create an obstacle Buzz table from proximity sensors
/  Acessing proximity in buzz script
/    proximity[0].angle and proximity[0].value - front
/  ""          ""          "" 	      - right and back
/    proximity[3].angle and proximity[3].value - left
/    proximity[4].angle = -1 and proximity[4].value -bottom
-------------------------------------------------------------*/
{
  buzzvm_pushs(vm, buzzvm_string_register(vm, "proximity", 1));
  buzzvm_pusht(vm);
  buzzobj_t tProxTable = buzzvm_stack_at(vm, 1);
  buzzvm_gstore(vm);

  //  Fill into the proximity table
  buzzobj_t tProxRead;
  float angle = 0;
  for (size_t i = 0; i < 4; ++i)
  {
    //  Create table for i-th read
    buzzvm_pusht(vm);
    tProxRead = buzzvm_stack_at(vm, 1);
    buzzvm_pop(vm);
    //  Fill in the read
    buzzvm_push(vm, tProxRead);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "value", 0));
    buzzvm_pushf(vm, obst[i + 1]);
    buzzvm_tput(vm);
    buzzvm_push(vm, tProxRead);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "angle", 0));
    buzzvm_pushf(vm, angle);
    buzzvm_tput(vm);
    //  Store read table in the proximity table
    buzzvm_push(vm, tProxTable);
    buzzvm_pushi(vm, i);
    buzzvm_push(vm, tProxRead);
    buzzvm_tput(vm);
    angle += 1.5708;
  }
  //  Create table for bottom read
  angle = -1;
  buzzvm_pusht(vm);
  tProxRead = buzzvm_stack_at(vm, 1);
  buzzvm_pop(vm);
  //  Fill in the read
  buzzvm_push(vm, tProxRead);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "value", 0));
  buzzvm_pushf(vm, obst[0]);
  buzzvm_tput(vm);
  buzzvm_push(vm, tProxRead);
  buzzvm_pushs(vm, buzzvm_string_register(vm, "angle", 0));
  buzzvm_pushf(vm, angle);
  buzzvm_tput(vm);
  //  Store read table in the proximity table
  buzzvm_push(vm, tProxTable);
  buzzvm_pushi(vm, 4);
  buzzvm_push(vm, tProxRead);
  buzzvm_tput(vm);
  return vm->state;
}

int buzzuav_get_local_planner_path(buzzvm_t vm){
  /* Create empty positioning data table */
  buzzvm_pusht(vm);
  buzzobj_t path_Pose = buzzvm_stack_at(vm, 1);
  for(uint32_t i=0; i< exploration_path.size();++i){ 
    /* Store position data */
    TablePut_int_vec3d(path_Pose, i, exploration_path[i], vm);
  }
  
  // printf("Transfering Exploration path with %i points\n",exploration_path.size());
  exploration_path.clear();
  return buzzvm_ret1(vm);
}

int buzzuav_get_global_planner_path(buzzvm_t vm){
  /* Create empty positioning data table */
  buzzvm_pusht(vm);
  buzzobj_t path_Pose = buzzvm_stack_at(vm, 1);
  for(uint32_t i=0; i< homing_path.size();++i){ 
    /* Store position data */
    TablePut_int_vec3d(path_Pose, i, homing_path[i], vm);
  }
  
  printf("Transfering Homing path with %i points\n",homing_path.size());
  homing_path.clear();
  return buzzvm_ret1(vm);
}

int buzzuav_get_interpolation_path(buzzvm_t vm){
  /* Create empty positioning data table */
  buzzvm_pusht(vm);
  buzzobj_t path_Pose = buzzvm_stack_at(vm, 1);
  for(uint32_t i=0; i< interpolation_homing_path.size();++i){ 
    /* Store position data */
    TablePut_int_vec3d(path_Pose, i, interpolation_homing_path[i], vm);
  }
  
  // printf("Transfering interpolation path with %i points\n",interpolation_homing_path.size());
  interpolation_homing_path.clear();
  return buzzvm_ret1(vm);
}


int buzzuav_get_hierarchial_nav_tube(buzzvm_t vm){
  /* Create empty positioning data table */
  buzzvm_pusht(vm);
  buzzobj_t path_Pose = buzzvm_stack_at(vm, 1);
  int idx = 0;
  for(uint32_t i=0; i< nav_tube.size();i+=2){ 
    buzzvm_pushi(vm, idx);
    buzzvm_pusht(vm);
    buzzobj_t tVecTable = buzzvm_stack_at(vm, 1);
    buzzvm_tput(vm);
    /* Store position data */
    TablePut_int_vec3d(tVecTable, 1, nav_tube[i], vm);
    TablePut_int_vec3d(tVecTable, 2, nav_tube[i+1], vm);
    idx++;
  }
  printf("Transfering nav tube with %i points idx: %i \n",nav_tube.size(), idx);
  nav_tube.clear();
  buzzvm_push(vm, path_Pose);
  return buzzvm_ret1(vm);
}

int buzzuav_set_navigation_goal(buzzvm_t vm){
  buzzvm_lnum_assert(vm, 2);
  buzzvm_lload(vm, 1);  // x
  buzzvm_lload(vm, 2);  // y
  buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
  buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
  //printf("Setting movebase goal from rosbuzz  (%f,%f)", buzzvm_stack_at(vm, 2)->f.value, buzzvm_stack_at(vm, 1)->f.value); 
  new_move_goal_available = 1;
  navigation_goal[0] = buzzvm_stack_at(vm, 2)->f.value;
  navigation_goal[1] = buzzvm_stack_at(vm, 1)->f.value;
  return buzzvm_ret0(vm);
}

float* buzzuav_get_setpoint_goal(){
  return navigation_goal;
}

void clear_move_base_goal(){
  new_move_goal_available = 0;

}

int buzzuav_get_local_trajectory_goal(buzzvm_t vm){
  buzzvm_pusht(vm);
  buzzobj_t tVecTable = buzzvm_stack_at(vm, 1);
  TablePut_str_fval(tVecTable, "x", move_base_local_goal[0], vm);
  TablePut_str_fval(tVecTable, "y", move_base_local_goal[1], vm);
  move_base_local_goal[0] = 0.0;
  move_base_local_goal[1] = 0.0;
  return buzzvm_ret1(vm);
}

void set_move_base_local_trajectory_goal(float* move_base_goal){
  move_base_local_goal[0] = move_base_goal[0];
  move_base_local_goal[1] = move_base_goal[1];
  //printf("Movebase local vec received %f, %f", move_base_goal[0], move_base_goal[1]);
}

int is_new_move_goal_available(){
  return new_move_goal_available;
}

void set_move_base_status(int status){
  goal_status = status;
}

int buzzuav_get_move_base_goal_status(buzzvm_t vm){
  buzzvm_pushi(vm, goal_status);
  return buzzvm_ret1(vm);
}

int dummy_closure(buzzvm_t vm)
/*
/ Dummy closure for use during update testing
----------------------------------------------------*/
{
  return buzzvm_ret0(vm);
}

void set_log_path(std::string path){
  log_path = path;
}


void put_fiducial_tag_pose(int id, float x, float y, float z, float pitch, float roll, float yaw){
  map<int, buzz_utility::pos_ori_struct>::iterator itp = fiducial_tags_map.find(id);
  if (itp != fiducial_tags_map.end()){
    fiducial_tags_map.erase(itp);
  }  
  buzz_utility::Pos_with_ori_struct tag_pos(x, y, z,
      pitch, roll, yaw);
  fiducial_tags_map.insert(make_pair(id, tag_pos));
}

int buzzuav_update_fiducial_tags(buzzvm_t vm)
{

  buzzvm_pushs(vm, buzzvm_string_register(vm, "fiducial_tags", 1));
  buzzvm_pusht(vm);
  buzzobj_t tPoseTable = buzzvm_stack_at(vm, 1);
  buzzvm_gstore(vm);

  for (std::map<int, buzz_utility::pos_ori_struct>::iterator it=fiducial_tags_map.begin();
      it!=fiducial_tags_map.end(); ++it)
  {
    buzzvm_push(vm, tPoseTable);
    buzzvm_pushi(vm,  it->first);
    buzzvm_pusht(vm);
    buzzobj_t tVecTable = buzzvm_stack_at(vm, 1);
    buzzvm_tput(vm);
    TablePut_str_fval(tVecTable, "x", (it->second).x, vm);
    TablePut_str_fval(tVecTable, "y", (it->second).y, vm);
    TablePut_str_fval(tVecTable, "z", (it->second).z, vm);
    TablePut_str_fval(tVecTable, "pitch", (it->second).pitch, vm);
    TablePut_str_fval(tVecTable, "roll", (it->second).roll, vm);
    TablePut_str_fval(tVecTable, "yaw", (it->second).yaw, vm);
  }
  fiducial_tags_map.clear();
  return vm->state;
}

/****************************************/
/****************************************/
#if OMPL_FOUND

int C_InitializePathPlanner(buzzvm_t vm) {
  /* Obtain the args (start_x,start_y,goal_x,goal_y,time to compute, maxx, maxy)*/
  if(buzzdarray_size(vm->lsyms->syms)< 8 || buzzdarray_size(vm->lsyms->syms) > 8){
    ROS_ERROR("[ROBOT %u] expected 7 args (start_x,start_y,goal_x,goal_y,time to compute, maxx, maxy)for planner but received : %u \n\n",
              vm->robot, 
              buzzdarray_size(vm->lsyms->syms)-1);
  }
  float start_end_time[7]; 
  for(uint32_t i = 1; i < buzzdarray_size(vm->lsyms->syms); ++i) {
    buzzvm_lload(vm, i);
    buzzobj_t o = buzzvm_stack_at(vm, 1);
    buzzvm_pop(vm);
    switch(o->o.type) {
      case BUZZTYPE_NIL:
        ROS_ERROR("[ROBOT %u] expected %u arg to be int or float received nil \n\n",vm->robot, 
                i);
      break;
      case BUZZTYPE_INT:
        start_end_time[i-1]=o->i.value;
        // fprintf(stderr, "[ROBOT %u] arg %u is %f",vm->robot, 
        //         i,start_end_time[i]);
      break;
      case BUZZTYPE_FLOAT:
        start_end_time[i-1]= o->f.value;
        // fprintf(stderr, "[ROBOT %u] arg %u is %f",vm->robot, 
        //       i,start_end_time[i]);
      break;
    }
  }

  /*Get controls*/
  std::vector<std::vector<double>> controls = InitializePathPlanner(vm,start_end_time);
  /* Create empty positioning data table */
  buzzvm_pushs(vm, buzzvm_string_register(vm, "path_controls", 1));
  buzzvm_pusht(vm);
  buzzobj_t path_Pose_table = buzzvm_stack_at(vm, 1);
  buzzvm_gstore(vm);
  buzzobj_t path_Pose;
  for(uint32_t i=0; i< controls.size();++i){
    //  Create table for i-th read
    buzzvm_pusht(vm);
    path_Pose = buzzvm_stack_at(vm, 1);
    buzzvm_pop(vm);
    //  Fill in the read
    buzzvm_push(vm, path_Pose);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "x", 0));
    buzzvm_pushf(vm, controls[i][0]);
    buzzvm_tput(vm);
    buzzvm_push(vm, path_Pose);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "y", 0));
    buzzvm_pushf(vm, controls[i][1]);
    buzzvm_tput(vm);
    buzzvm_push(vm, path_Pose);
    buzzvm_pushs(vm, buzzvm_string_register(vm, "z", 0));
    buzzvm_pushf(vm, controls[i][2]);
    buzzvm_tput(vm);
    //  Store read table in the proximity table
    buzzvm_push(vm, path_Pose_table);
    buzzvm_pushi(vm, i);
    buzzvm_push(vm, path_Pose);
    buzzvm_tput(vm);
     // std::cout<<"Controls "<<i<<" X "<<controls[i][0]<<" Y "<<controls[i][1]<<" Z "<<controls[i][2]<<std::endl;
  }
   return buzzvm_ret0(vm);
}

svg::Point visualize_point(double* state, svg::Dimensions dims)
{
  double x = ((state[0]-(half_map_height/2))-MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = ((state[1]-(half_map_length/2))-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

svg::Point visualize_point(std::vector<double> state, svg::Dimensions dims)
{
  double x = ((state[0]-(half_map_height/2)) -MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = ((state[1]-(half_map_length/2))-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

svg::Point visualize_point(ob::State* state, svg::Dimensions dims)
{
  double x = (((double)state->as<ob::RealVectorStateSpace::StateType>()->values[0]-(half_map_height/2)) -MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = (((double)state->as<ob::RealVectorStateSpace::StateType>()->values[1]-(half_map_length/2))-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

svg::Point visualize_point(double* state, svg::Dimensions dims, int empty)
{
  double x = ((state[0])-MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = ((state[1])-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

svg::Point visualize_point(std::vector<double> state, svg::Dimensions dims, int empty)
{
  double x = ((state[0]) -MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = ((state[1])-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

svg::Point visualize_point(ob::State* state, svg::Dimensions dims, int empty)
{
  double x = (((double)state->as<ob::RealVectorStateSpace::StateType>()->values[0]) -MIN_X)/(MAX_X-MIN_X) * dims.width; 
  double y = (((double)state->as<ob::RealVectorStateSpace::StateType>()->values[1])-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
  return svg::Point(x,y);
}

void import_empty_map(){
  std::vector<std::vector<int>> Grid_map2d;
  for(uint32_t i=0; i< MAX_X; i++){
    std::vector<int> v_row(MAX_Y,0);
    Grid_map2d.push_back(v_row);
  }
  Grid_map.push_back(Grid_map2d);
  printf("Size of grid x %u size of grid y %u\n",Grid_map2d.size(),Grid_map2d[0].size() );
}

std::vector<std::vector<double>> import_map(std::string m_map_file_name)
{
  std::string line;
  std::ifstream m_map_file (m_map_file_name);
  int map_height = 0;
  int map_length = 0;
  // std::vector<std::vector<int>> Grid_map;
  std::vector<double> start_pos(2);
  std::vector<double> end_pos(2);

  if (m_map_file.is_open())
  {
    int line_num = 0;
    std::vector<std::vector<int>> Grid_map2d;
    while ( getline (m_map_file,line) )
    { 
      if(line_num < 4){ // header
        if(line_num == 1){
           std::string str_height = line.substr (7,line.size());
           map_height = std::stoi(str_height) ;
           // std::cout << "height "<< map_height << '\n';
        }
        if(line_num == 2){
           std::string str_length = line.substr (6,line.size());
           map_length = std::stoi(str_length) ;
           // std::cout << "length "<< map_length << '\n';
        }
        if(line_num == 3){
          half_map_height = map_height;
          half_map_length = map_length;
          MIN_X = 0; // half_map_height;
          MIN_Y = 0; //half_map_length;
          MAX_X = map_height-1; // half_map_height*-1;
          MAX_Y = map_length-1; // half_map_length*-1;
          
          // std::cout << "half length "<< half_map_length << '\n';
          // std::cout << "half height "<< half_map_height << '\n';
        }
      }
      else{
        std::vector<int> v_row;
        for(uint32_t i=0;i<line.size();i++){
          if(line[i] == free_space || line[i] == free_space_l || line[i] == START_space ||
             line[i] == TARGET_space)
            v_row.push_back(0);
          else
            v_row.push_back(1);
          if(line[i] == START_space){
            start_pos[0]=(line_num-4); //+half_map_height;
            start_pos[1]=i; // +half_map_length;
          }
          else if(line[i] == TARGET_space){
            end_pos[0]=(line_num-4); //+half_map_height;
            end_pos[1]=i; //+half_map_length;
          }
      }
      Grid_map2d.push_back(v_row);
      }
      line_num +=1;
    }
    m_map_file.close();
    Grid_map.push_back(Grid_map2d);
  printf("Size of grid x %u size of grid y %u\n",Grid_map2d.size(),Grid_map2d[0].size() );
  // half_map_length = half_map_length *-1;
  // half_map_height = half_map_height *-1;
  }
  else {printf("ERROR in Opening Map file\n");}
  std::vector<std::vector<double>> start_end_pos;
  start_end_pos.push_back(start_pos);
  start_end_pos.push_back(end_pos);
  return start_end_pos;
}


std::vector<std::vector<double>> import_3dmap(std::string m_map_file_name)
{
  std::string line;
  std::ifstream m_map_file (m_map_file_name);
  int map_height = 0;
  int map_length = 0;
  int map_planes = 0;
  // std::vector<std::vector<int>> Grid_map;
  std::vector<std::vector<int>> v_plane;
  std::vector<double> start_pos(2);
  std::vector<double> end_pos(2);
  int current_plane = 0;
  if (m_map_file.is_open())
  {
    int line_num = 0;
    while ( getline (m_map_file,line) )
    { 
      if(line_num < 6){ // header
        if(line_num == 2){
           std::string str_height = line.substr (7,line.size());
           map_height = std::stoi(str_height) ;
           // std::cout << "height "<< map_height << '\n';
        }
        if(line_num == 3){
           std::string str_length = line.substr (6,line.size());
           map_length = std::stoi(str_length) ;
           // std::cout << "length "<< map_length << '\n';
        }
        if(line_num == 4){
          half_map_height = map_height;
          half_map_length = map_length;
          MIN_X = 0; // half_map_height;
          MIN_Y = 0; //half_map_length;
          MAX_X = map_height-1; // half_map_height*-1;
          MAX_Y = map_length-1; // half_map_length*-1;
          std::string str_height = line.substr (7,line.size());
          map_planes = std::stoi(str_height);
          // std::cout << "half length "<< half_map_length << '\n';
          // std::cout << "half height "<< half_map_height << '\n';
        }
      }
      else{
        if(line_num == map_height+6) line_num = 5;
        // plane_pos = (current_plane + 1) * PLANE_RESOLUTION;
        std::string plane_str = line.substr(0,5);
        if(plane_str == "plane"){
          current_plane = std::stoi(line.substr (6,line.size()));
          Grid_map.push_back(v_plane);
          v_plane.clear();
        }
        else{
          std::vector<int> v_row;
          for(int i=0;i<line.size();i++){
            if(line[i] == free_space || line[i] == free_space_l || line[i] == START_space ||
               line[i] == TARGET_space)
              v_row.push_back(0);
            else
              v_row.push_back(1);
            if(line[i] == START_space && current_plane == 0){
              start_pos[0]=(line_num-6); //+half_map_height;
              start_pos[1]=i; // +half_map_length;
            }
            else if(line[i] == TARGET_space && current_plane == 0){
              end_pos[0]=(line_num-6); //+half_map_height;
              end_pos[1]=i; //+half_map_length;
            }
        }
        v_plane.push_back(v_row);
      }
    }
      line_num +=1;
    }
    if(v_plane.size() > 0){
            Grid_map.push_back(v_plane);
            v_plane.clear();
    }
    m_map_file.close();

  }
  else {printf("ERROR in Opening Map file\n");}
  std::vector<std::vector<double>> start_end_pos;
  start_end_pos.push_back(start_pos);
  start_end_pos.push_back(end_pos);
  // std::cout<<"start X "<<start_pos[0]<<" y "<<start_pos[1]<<std::endl;
  // std::cout<<"end x "<<end_pos[0]<<"  y "<<end_pos[1]<<std::endl; 
  return start_end_pos;
}

int obtain_2d_path_end(std::vector<std::vector<double>> solution_nodes){
  double cur_pos[2]={solution_nodes[0][0],solution_nodes[0][1]};
  bool found = false;
  double integration_step = 0.1;
  int cur_waypoint = 1;
  ValidityChecker checker = ValidityChecker(Grid_map[0]); // The loop below will Bomb if map not loaded
                                                          // Call importmap before calling this memeber fun
  while(!found){
    // compute relative vector
    double x_rel = solution_nodes[cur_waypoint][0] - cur_pos[0];
    double y_rel = solution_nodes[cur_waypoint][1] - cur_pos[1];
    if(cur_waypoint< solution_nodes.size()-1){
      if(sqrt((x_rel) * (x_rel) + (y_rel) * (y_rel)) > 0.1){
        double cur_vel = sqrt((x_rel) * (x_rel) + (y_rel) * (y_rel)); 
        double cur_heading = atan(y_rel/x_rel);
        cur_pos[0] += integration_step*cur_vel*cos(cur_heading);
        cur_pos[1] += integration_step*cur_vel*sin(cur_heading);
        bool validity = checker.isValid(cur_pos);
        if(!validity){
          found = true;
          // std::cout<<"Collsion detected at "<<cur_waypoint<<std::endl;
          return cur_waypoint - 1;
        }
      }
      else{
        cur_waypoint++;
        if(cur_waypoint >= solution_nodes.size()-1){
          found = true;
          std::cout<<"Reached last waypoint no solution found"<<std::endl;
          return 0;
        }
      }
      // std::cout<<"WP: "<<cur_waypoint<<" len2d "<<rel_vec.Length()<<" cur_pos ("<<cur_pos[0]
      //  <<","<<cur_pos[1]<<") rel vec ("<<x_rel<<","<<y_rel<<")"<<std::endl;
    }
    else{
      found = true;
      std::cout<<"Reached last waypoint"<<std::endl;
      return 0;
    }
  
  }  
  return 0;
}

std::vector<std::vector<double>> InitializePathPlanner(buzzvm_t m_tBuzzVM, float* start_end_time){

  /*Import map into the grid vector for state validity checking */



  // std::vector<std::vector<double>> start_end_pos = import_map(strmapFName);
  float Required_path_segment_len = 7;
  std::vector<std::vector<double>> start_end_pos;
  if(map_option ==1){
    start_end_pos = import_3dmap(strmapFName);
  }
  else if(map_option == 2){ // Load empty grid map for planning
    /* Fill in the parms from the buzz hook args */
    std::vector<double> start_state(2,0);
    start_state[0] = start_end_time[0];
    start_state[1] = start_end_time[1];
    start_end_pos.push_back(start_state);
    std::vector<double> end_state(2,0);
    end_state[0] = start_end_time[2];
    end_state[1] = start_end_time[3];
    start_end_pos.push_back(end_state);
    MAX_X = ceil(start_end_time[5]);
    MAX_Y =ceil(start_end_time[6]);
    half_map_height = start_end_time[5];
    half_map_length = start_end_time[6];
    import_empty_map();
  }
  else{
    start_end_pos = import_map(strmapFName);
  }
  /* If solved obtain the best solutions found */
  std::vector<std::vector<double>> solution_nodes;
  bool path_exsistence;
  if(map_option == 1){
    pe::Path_checker m_check(Grid_map[0], MAX_X+1, MAX_Y+1, 1);
    m_check.create_tree_nodes();
    m_check.add_tree_edges();
    double d_start[2]={start_end_pos[0][0],start_end_pos[0][1]};
    double d_end[2]={start_end_pos[1][0],start_end_pos[1][1]};
    m_check.set_start_goal(d_start,d_end);
    path_exsistence = m_check.searchTree();
  }
  else{
    path_exsistence = true;
  }
  if(path_exsistence){
    /* Initialize a 2d vector state space within the planner */
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    /* Set bounds to the state space */
    ob::RealVectorBounds m_bound(2);
    m_bound.setLow(0,-half_map_height);
    m_bound.setLow(1,-half_map_length);
    /* Should have been updated by import map call */
    m_bound.setHigh(0,half_map_height);
    m_bound.setHigh(1,half_map_length);

    /* Set the bounds of space */
    space->as<ob::RealVectorStateSpace>()->setBounds(m_bound);
    /* Construct a space information instance for this state space */
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    /* Set the object used to check which states in the space are valid */
    int empty = 0;
    if(map_option == 2){
      empty = 1;
    }
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si,Grid_map[0],empty)));
    si->setup();
    // Set our robot's starting state to be the one obtained by import map
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_end_pos[0][0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_end_pos[0][1];
    // Set our robot's goal state tto be the one obtained by import map
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_end_pos[1][0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_end_pos[1][1];
    // Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // pdef->setOptimizationObjective(getPathLengthObjective(si));

    // Construct our optimizing planner using the RRTstar algorithm.
    og::RRTstar* m_rrt_planner = new og::RRTstar(si);
    ob::PlannerPtr optimizingPlanner(m_rrt_planner);
    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();
    // attempt to solve the planning problem within one second of
    // planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(start_end_time[4]);

    if (solved)
    {

      // Output the length of the path found
      // std::cout 
      //     << optimizingPlanner->getName()
      //     << " found a solution of length "
      //     << pdef->getSolutionPath()->length()
      //     << " with an optimization objective value of "
      //     << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective())<<"Best path : "
      //     << std::endl;
          // std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath())->print(std::cout);
      std::shared_ptr<ompl::geometric::PathGeometric> c_path = 
            std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
      
      fprintf(stderr, "Current state count in path: %i , length: %f, Num of states req %f\n", 
                        (int)(c_path->getStateCount()), c_path->length(), c_path->length()/Required_path_segment_len);
      c_path->interpolate((unsigned int) c_path->length()/Required_path_segment_len);
      fprintf(stderr, "After intepolation: %i \n", (int)(c_path->getStateCount())); 
      std::vector<ob::State *> solutionStates = c_path->getStates();                  
      for(auto state : solutionStates){
        std::vector<double> statePoint(3,0.0);
        // std::cout<<" SOl state  X "<< state->as<ob::RealVectorStateSpace::StateType>()->values[0]
        //   <<" Y "<<state->as<ob::RealVectorStateSpace::StateType>()->values[1]<<std::endl;
          if(map_option == 2){
            statePoint[0]= state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            statePoint[1]= state->as<ob::RealVectorStateSpace::StateType>()->values[1];
          }
          else{
            statePoint[0]= state->as<ob::RealVectorStateSpace::StateType>()->values[0]-(half_map_height/2);
            statePoint[1]= state->as<ob::RealVectorStateSpace::StateType>()->values[1]-(half_map_length/2);
          }
          solution_nodes.push_back(statePoint);
      }
      // add goal state to the path waypoints 
      std::vector<double> statePoint(3,0.0);
      if(map_option == 2){
        statePoint[0]= start_end_pos[1][0];
        statePoint[1]= start_end_pos[1][1];  
      }
      else{
        statePoint[0]= start_end_pos[1][0]-(half_map_height/2);
        statePoint[1]= start_end_pos[1][1]-(half_map_length/2);  
      }
      solution_nodes.push_back(statePoint);
      buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "PATH_TYPE", 1));
      buzzvm_pushi(m_tBuzzVM, -1);
      buzzvm_gstore(m_tBuzzVM);

       std::ofstream log;
            log.open(log_path+"path.csv",
                  std::ofstream::out | std::ofstream::trunc);
      for(auto state : solutionStates){
        fprintf(stderr," SOl state  X : %f, Y: %f, Z: %f \n", state->as<ob::RealVectorStateSpace::StateType>()->values[0]
          ,state->as<ob::RealVectorStateSpace::StateType>()->values[1]
          ,state->as<ob::RealVectorStateSpace::StateType>()->values[2]);
        log<<state->as<ob::RealVectorStateSpace::StateType>()->values[0]<<","
          <<state->as<ob::RealVectorStateSpace::StateType>()->values[1]<<","
          <<state->as<ob::RealVectorStateSpace::StateType>()->values[2]<<std::endl;
      }
      log.close();

         // For boder padding
    // MIN_X =-(half_map_height/2.0)-5;
    // MIN_Y =-(half_map_length/2.0)-5;
    // MAX_X =(half_map_height/2.0)+5;
    // MAX_Y =(half_map_length/2.0)+5;
    // Save nodes to file
    if(save_solution_svg){
      std::stringstream s_name;
      s_name<<log_path<<"nodes_"<<"11"<<".svg";
      std::string dir(s_name.str());
      // fprintf(stderr, "nodes path %s\n",dir.c_str());
      
      int image_width=500;
      int image_height=500;

      svg::Dimensions dimensions(image_width, image_height);
      svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));

      // Draw solution path

      std::vector<ob::State *> s_path = 
          std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath())->getStates();
      svg::Polyline traj_line(svg::Stroke(1, svg::Color::Black));
      for(auto state : s_path){
        traj_line<<visualize_point(state,dimensions,1);
      }
      doc<<traj_line;

      // draw obstacle
      // ...
      //

      /* Store all nodes to a file */
      std::ofstream log;
      log.open(log_path+"all_nodes.csv",
            std::ofstream::out | std::ofstream::trunc);

      std::vector<std::pair <std::vector<double>,double>> m_t_nodes;
      // m_rrt_planner->getAllNodesAs2DVec(m_t_nodes);

      // std::cout<<" Size of nodes: "<<m_t_nodes.size()<<std::endl;
      for(auto node : m_t_nodes){
        svg::Circle circle4(visualize_point(node.first,dimensions,1),
                      2,svg::Fill( svg::Color(125,125,125) ));
        log<<node.first[0]<<","
            <<node.first[1]<<std::endl;
        doc<<circle4;
      }
      log.close();


      double m_point[2]={start->as<ob::RealVectorStateSpace::StateType>()->values[0], 
                 start->as<ob::RealVectorStateSpace::StateType>()->values[1]};
      svg::Circle circle(visualize_point(m_point,dimensions,1),
                      4,svg::Fill( svg::Color(255,0,0) ));
      doc<<circle;
      double e_point[2]={goal->as<ob::RealVectorStateSpace::StateType>()->values[0], 
                 goal->as<ob::RealVectorStateSpace::StateType>()->values[1]};
      svg::Circle circle2(visualize_point(e_point,dimensions,1),
                      4,svg::Fill( svg::Color(0,255,0) ));
      doc<<circle2;

      doc.save();
    }
    }
    else{
     fprintf(stderr, "No solution found \n"); 
     buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "PATH_TYPE", 1));
     buzzvm_pushi(m_tBuzzVM, -2);
     buzzvm_gstore(m_tBuzzVM);
    }
  }
  else{
    fprintf(stderr,"Path doesn't exsist trying 3D planning\n");


    // Construct the robot state space in which we're planning. We're
    // planning in [0,half_map_length]x[0,half_map_height], a subset of R^3.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));
    ob::RealVectorBounds m_bound(3);
    m_bound.setLow(0,0);
    m_bound.setLow(1,0);
    m_bound.setLow(2,0);
    m_bound.setHigh(0,half_map_height-1);
    m_bound.setHigh(1,half_map_length-1);
    m_bound.setHigh(2,MAX_ALT);
    std::vector<double> diff = m_bound.getDifference();
    // std::cout<<" X diff "<<diff[0]<< " Y diff "<< diff[1]<<"Z diff "<< diff[2]<<std::endl;
    // Set the bounds of space to be in [0,1].
    space->as<ob::RealVectorStateSpace>()->setBounds(m_bound);
    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker3D(si,Grid_map)));
    si->setup();
    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_end_pos[0][0];
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_end_pos[0][1];
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0.0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_end_pos[1][0];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_end_pos[1][1];
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = 1.0;

    // Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // pdef->setOptimizationObjective(getPathLengthObjective(si));

    // Construct our optimizing planner using the RRTstar algorithm.
    og::RRTstar* m_rrt_planner = new og::RRTstar(si);
    ob::PlannerPtr optimizingPlanner(m_rrt_planner);
    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();
    // attempt to solve the planning problem within one second of
    // planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(2.0);
    if (solved)
      {
        // Output the length of the path found
        // std::cout 
        //   << "Iteration: "<< x<<" "
        //     << optimizingPlanner->getName()
        //     << " found a solution of length "
        //     << pdef->getSolutionPath()->length()
        //     << " with an optimization objective value of "
        //     << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective())<<"Best path : "
        //     << std::endl;
        //     std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath())->print(std::cout);

        std::shared_ptr<ompl::geometric::PathGeometric> c_path = 
            std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
      
        fprintf(stderr, "Current state count in path: %i , length: %f, Num of states req %f\n", 
                          (int)(c_path->getStateCount()), c_path->length(), c_path->length()/Required_path_segment_len);
        c_path->interpolate((unsigned int) c_path->length()/Required_path_segment_len);
        fprintf(stderr, "After intepolation: %i \n", (int)(c_path->getStateCount())); 
        std::vector<ob::State *> solutionStates = c_path->getStates();      
        std::vector<std::vector<double>> solution_nodes_;
        for(auto state : solutionStates){
          std::vector<double> statePoint(3,0.0);
          // std::cout<<" SOl state  X "<< state->as<ob::RealVectorStateSpace::StateType>()->values[0]
          //   <<" Y "<<state->as<ob::RealVectorStateSpace::StateType>()->values[1]<<
          //   " Z "<<state->as<ob::RealVectorStateSpace::StateType>()->values[2]<<std::endl;
            statePoint[0]= state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            statePoint[1]= state->as<ob::RealVectorStateSpace::StateType>()->values[1];
            statePoint[2]= state->as<ob::RealVectorStateSpace::StateType>()->values[2];
            solution_nodes_.push_back(statePoint);
            statePoint[0]= state->as<ob::RealVectorStateSpace::StateType>()->values[0]-(half_map_height/2.0);
            statePoint[1]= state->as<ob::RealVectorStateSpace::StateType>()->values[1]-(half_map_length/2.0);
            
            
            solution_nodes.push_back(statePoint);
        }
        // add goal state to the path waypoints 
        std::vector<double> statePoint(3,0.0);
        statePoint[0]= start_end_pos[1][0];
        statePoint[1]= start_end_pos[1][1]; 
        solution_nodes_.push_back(statePoint);

        statePoint[0]= start_end_pos[1][0]-(half_map_height/2);
        statePoint[1]= start_end_pos[1][1]-(half_map_length/2);
        solution_nodes.push_back(statePoint);

        int path2dend = obtain_2d_path_end(solution_nodes_);
        fprintf(stderr, "2d PATH end: %i\n",path2dend);
        // std::cout<<" 2d Path end "<<path2dend<<std::endl;
        buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "PATH_TYPE", 1));
        buzzvm_pushi(m_tBuzzVM, path2dend);
        buzzvm_gstore(m_tBuzzVM);
            std::ofstream log;
            log.open("path.csv",
                  std::ofstream::out | std::ofstream::trunc);
      for(auto state : solutionStates){
              // std::cout<<" SOl state  X "<< state->as<ob::RealVectorStateSpace::StateType>()->values[0]
              //   <<" Y "<<state->as<ob::RealVectorStateSpace::StateType>()->values[1]
              //   <<" Z "<<state->as<ob::RealVectorStateSpace::StateType>()->values[2]<<std::endl;
              log<<state->as<ob::RealVectorStateSpace::StateType>()->values[0]<<","
                <<state->as<ob::RealVectorStateSpace::StateType>()->values[1]<<","
                <<state->as<ob::RealVectorStateSpace::StateType>()->values[2]<<std::endl;
            }
            log.close();
      }
      else{
        std::cout<< "No solution found." << std::endl;
        buzzvm_pushs(m_tBuzzVM, buzzvm_string_register(m_tBuzzVM, "PATH_TYPE", 1));
        buzzvm_pushi(m_tBuzzVM, -2);
        buzzvm_gstore(m_tBuzzVM);
      }



      // For boder padding
    MIN_X =-(half_map_height/2.0)-5;
    MIN_Y =-(half_map_length/2.0)-5;
    MAX_X =(half_map_height/2.0)+5;
    MAX_Y =(half_map_length/2.0)+5;
    // Save nodes to file
    if(save_solution_svg){
      std::stringstream s_name;
        s_name<<"nodes_"<<"11"<<".svg";
        std::string dir(s_name.str());
        int image_width=500;
      int image_height=500;

        svg::Dimensions dimensions(image_width, image_height);
        svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));

      // Draw solution path

      std::vector<ob::State *> s_path = 
          std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath())->getStates();
      svg::Polyline traj_line(svg::Stroke(1, svg::Color::Black));
      for(auto state : s_path){
        traj_line<<visualize_point(state,dimensions);
      }
      doc<<traj_line;

      // draw obstacle
      double x_offset= 0.0,y_offset=0.0;
      int Planning_plane = 20;
      for(int i=0;i<Grid_map[Planning_plane].size();++i){
          for(int j=0;j<Grid_map[Planning_plane][i].size();++j){
              if(i==0){ // top case
                if(Grid_map[Planning_plane][i][j] == 1 && (Grid_map[Planning_plane][i][j-1] == 0 ||
                     Grid_map[Planning_plane][i][j+1] == 0 || Grid_map[Planning_plane][i+1][j] == 0 ||
                   Grid_map[Planning_plane][i+1][j+1] == 0 || Grid_map[Planning_plane][i+1][j-1] == 0) ){
                  double temp[2];
                  // Add a column for trees
                  temp[0] = i - (OBSTACLE_SIDES1/2)+x_offset;
            temp[1] = j + (OBSTACLE_SIDES2/2)+y_offset;
              doc<<svg::Rectangle(visualize_point(temp,dimensions), 
                      (OBSTACLE_SIDES1)/(MAX_X-MIN_X) * dimensions.width,
                      (OBSTACLE_SIDES2)/(MAX_Y-MIN_Y) * dimensions.height,
                      svg::Color::Red);
                }
              }
              else if(i == Grid_map[Planning_plane].size()-1){ // bottom case 
                if(Grid_map[Planning_plane][i][j] == 1 && (Grid_map[Planning_plane][i][j-1] == 0 || 
                    Grid_map[Planning_plane][i][j+1] == 0 || Grid_map[Planning_plane][i-1][j] == 0 ||
                   Grid_map[Planning_plane][i-1][j-1] == 0 || Grid_map[Planning_plane][i-1][j+1] == 0) ){
                    double temp[2];
                  // Add a column for trees
                  temp[0] = i - (OBSTACLE_SIDES1/2)+x_offset;
            temp[1] = j + (OBSTACLE_SIDES2/2)+y_offset;
              doc<<svg::Rectangle(visualize_point(temp,dimensions), 
                      (OBSTACLE_SIDES1)/(MAX_X-MIN_X) * dimensions.width,
                      (OBSTACLE_SIDES2)/(MAX_Y-MIN_Y) * dimensions.height,
                      svg::Color::Red);
                }
              }
              else if(j == 0){ // Left case
                if(Grid_map[Planning_plane][i][j] == 1 && (Grid_map[Planning_plane][i][j+1] == 0 || 
                    Grid_map[Planning_plane][i-1][j] == 0 || Grid_map[Planning_plane][i+1][j] == 0 ||
                   Grid_map[Planning_plane][i-1][j+1] == 0 || Grid_map[Planning_plane][i+1][j+1] == 0) ){
                    double temp[2];
                  // Add a column for trees
                  temp[0] = i - (OBSTACLE_SIDES1/2)+x_offset;
            temp[1] = j + (OBSTACLE_SIDES2/2)+y_offset;
              doc<<svg::Rectangle(visualize_point(temp,dimensions), 
                      (OBSTACLE_SIDES1)/(MAX_X-MIN_X) * dimensions.width,
                      (OBSTACLE_SIDES2)/(MAX_Y-MIN_Y) * dimensions.height,
                      svg::Color::Red);
                    
                }
              }
              else if(j == Grid_map[Planning_plane][i].size()-1){ // right case
                if(Grid_map[Planning_plane][i][j] == 1 && (Grid_map[Planning_plane][i][j-1] == 0 || 
                    Grid_map[Planning_plane][i-1][j] == 0 || Grid_map[Planning_plane][i+1][j] == 0 ||
                   Grid_map[Planning_plane][i-1][j-1] == 0 || Grid_map[Planning_plane][i+1][j-1] == 0) ){
                   double temp[2];
                  // Add a column for trees
                  temp[0] = i - (OBSTACLE_SIDES1/2)+x_offset;
            temp[1] = j + (OBSTACLE_SIDES2/2)+y_offset;
              doc<<svg::Rectangle(visualize_point(temp,dimensions), 
                      (OBSTACLE_SIDES1)/(MAX_X-MIN_X) * dimensions.width,
                      (OBSTACLE_SIDES2)/(MAX_Y-MIN_Y) * dimensions.height,
                      svg::Color::Red);
                }
              }
              else{
            if(Grid_map[Planning_plane][i][j] == 1 && (Grid_map[Planning_plane][i][j-1] == 0 || 
              Grid_map[Planning_plane][i][j+1] == 0 || Grid_map[Planning_plane][i-1][j] == 0 || Grid_map[Planning_plane][i+1][j] == 0 ||
              Grid_map[Planning_plane][i-1][j-1] == 0 || Grid_map[Planning_plane][i-1][j+1] == 0 ||
              Grid_map[Planning_plane][i+1][j+1] == 0 || Grid_map[Planning_plane][i+1][j-1] == 0) ){
                double temp[2];
                  // Add a column for trees
                  temp[0] = i - (OBSTACLE_SIDES1/2)+x_offset;
            temp[1] = j + (OBSTACLE_SIDES2/2)+y_offset;
              doc<<svg::Rectangle(visualize_point(temp,dimensions), 
                      (OBSTACLE_SIDES1)/(MAX_X-MIN_X) * dimensions.width,
                      (OBSTACLE_SIDES2)/(MAX_Y-MIN_Y) * dimensions.height,
                      svg::Color::Red);
            } 
              }
            }
        }

      std::vector<std::pair <std::vector<double>,double>> m_t_nodes;
      // m_rrt_planner->getAllNodesAs2DVec(m_t_nodes);


      std::ofstream log;
            log.open("all_nodes.csv",
                  std::ofstream::out | std::ofstream::trunc);

      // std::cout<<" Size of nodes: "<<m_t_nodes.size()<<std::endl;
      for(auto node : m_t_nodes){
        svg::Circle circle4(visualize_point(node.first,dimensions),
                      2,svg::Fill( svg::Color(125,125,125) ));
        log<<node.first[0]<<","
            <<node.first[1]<<std::endl;
        doc<<circle4;
      }
      log.close();

      double m_point[2]={start->as<ob::RealVectorStateSpace::StateType>()->values[0], 
                 start->as<ob::RealVectorStateSpace::StateType>()->values[1]};
      svg::Circle circle(visualize_point(m_point,dimensions),
                      4,svg::Fill( svg::Color(255,0,0) ));
      doc<<circle;
      double e_point[2]={goal->as<ob::RealVectorStateSpace::StateType>()->values[0], 
                 goal->as<ob::RealVectorStateSpace::StateType>()->values[1]};
      svg::Circle circle2(visualize_point(e_point,dimensions),
                      4,svg::Fill( svg::Color(0,255,0) ));
      doc<<circle2;

      doc.save();
    }


  }
  return solution_nodes;
}


/****************************************/
/****************************************/





#endif

}

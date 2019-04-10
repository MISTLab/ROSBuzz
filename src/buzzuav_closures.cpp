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
static double goto_pos[4];
static double goto_gpsgoal[3];
static double cur_pos[4];
static double cur_NEDpos[2];

static int rc_id = -1;
static int rc_cmd = 0;
static double rc_gpsgoal[3];
static float rc_gimbal[4];

static float batt[3];
static float obst[5] = { 0, 0, 0, 0, 0 };
static uint8_t status;

static int cur_cmd = 0;
static int buzz_cmd = 0;
static float height = 0;

static bool deque_full = false;
static int rssi = 0;
static float raw_packet_loss = 0.0;
static int filtered_packet_loss = 0;
static float api_rssi = 0.0;
static bool logVoronoi = false;
static std::vector<bounding_box> yolo_boxes;

std::ofstream voronoicsv;

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
std::map<int, buzz_utility::Pos_struct> neighbors_map;
std::map<int, buzz_utility::neighbors_status> neighbors_status_map;
std::map<int, std::map<int, int>> grid;

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

void rb_from_gps(double nei[], double out[], double cur[])
/*
/ Compute Range and Bearing from 2 GPS set of coordinates
/----------------------------------------*/
{
  double d_lon = nei[1] - cur[1];
  double d_lat = nei[0] - cur[0];
  double ned_x = DEG2RAD(d_lat) * EARTH_RADIUS;
  double ned_y = DEG2RAD(d_lon) * EARTH_RADIUS * cos(DEG2RAD(nei[0]));
  out[0] = sqrt(ned_x * ned_x + ned_y * ned_y);
  out[1] = constrainAngle(atan2(ned_y, ned_x));
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

  ROS_INFO("----->Saved %i waypoints.", wplist_map.size());

  // Close the file:
  fin.close();
}

void check_targets_sim(double lat, double lon, double* res)
/*
/ check if a listed target is close
----------------------------------------------------------- */
{
  float visibility_radius = 5.0;
  map<int, buzz_utility::RB_struct>::iterator it;
  for (it = wplist_map.begin(); it != wplist_map.end(); ++it)
  {
    double rb[3];
    double ref[2] = { lat, lon };
    double tar[2] = { it->second.latitude, it->second.longitude };
    rb_from_gps(tar, rb, ref);
    if (rb[0] < visibility_radius && (buzz_utility::get_bvmstate() == "WAYPOINT" && it->second.r == 0))
    {
      ROS_WARN("FOUND A TARGET IN WAYPOINT!!! [%i]", it->first);
      res[0] = it->first;
      res[1] = it->second.latitude;
      res[2] = it->second.longitude;
      res[3] = it->second.altitude;
    }
    else if (rb[0] < visibility_radius && (buzz_utility::get_bvmstate() == "DEPLOY" && it->second.r == 1))
    {
      ROS_WARN("FOUND A TARGET IN WAYPOINT!!! [%i]", it->first);
      res[0] = it->first;
      res[1] = it->second.latitude;
      res[2] = it->second.longitude;
      res[3] = it->second.altitude;
    }
  }
}

int buzz_exportmap(buzzvm_t vm)
/*
/ Buzz closure to export a 2D map
/----------------------------------------*/
{
  grid.clear();
  buzzvm_lnum_assert(vm, 1);
  // Get the parameter
  buzzvm_lload(vm, 1);
  buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);  // dictionary
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
  std::vector<Point> polygon_bound;
  for (int32_t i = 0; i < buzzdict_size(t->t.value); ++i)
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
    Point Intersection;
    getintersection(Point(0.0, 0.0), P, polygon_bound, &Intersection);
    double gps[3];
    double d[2] = { Intersection.x, Intersection.y };
    gps_from_vec(d, gps);
    set_gpsgoal(gps);
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
  double rb[3];

  rb_from_gps(tmp, rb, cur_pos);
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
  buzzvm_pushs(vm, buzzvm_string_register(vm, "gp", 1));
  buzzvm_tget(vm);
  newRS.gps_strenght = buzzvm_stack_at(vm, 1)->i.value;
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
  return vm->state;
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
    payload_out.payload64.push_back(it->second.gps_strenght);
    payload_out.payload64.push_back(it->second.batt_lvl);
    payload_out.payload64.push_back(it->second.xbee);
    payload_out.payload64.push_back(it->second.flight_status);
  }
  //  Add Robot id and message number to the published message
  payload_out.sysid = (uint8_t)neighbors_status_map.size();

  return payload_out;
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
  double rb[3];
  rb_from_gps(goal, rb, cur_pos);
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

std::map<int, std::map<int, int>> getgrid()
/*
/ return the grid
/-------------------------------------------------------------*/
{
  return grid;
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
  for (int i = 0; i < 5; i++)
    obst[i] = dist[i];
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

void set_currentNEDpos(double x, double y)
/*
/ update interface position array
-----------------------------------*/
{
  cur_NEDpos[0] = x;
  cur_NEDpos[1] = y;
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
void neighbour_pos_callback(int id, float range, float bearing, float elevation)
{
  buzz_utility::Pos_struct pos_arr;
  pos_arr.x = range;
  pos_arr.y = bearing;
  pos_arr.z = elevation;
  map<int, buzz_utility::Pos_struct>::iterator it = neighbors_map.find(id);
  if (it != neighbors_map.end())
    neighbors_map.erase(it);
  neighbors_map.insert(make_pair(id, pos_arr));
}

//  update at each step the VM table
void update_neighbors(buzzvm_t vm)
{
  //   Reset neighbor information
  buzzneighbors_reset(vm);
  //  Get robot id and update neighbor information
  map<int, buzz_utility::Pos_struct>::iterator it;
  for (it = neighbors_map.begin(); it != neighbors_map.end(); ++it)
  {
    buzzneighbors_add(vm, it->first, (it->second).x, (it->second).y, (it->second).z);
  }
}

// Clear neighbours pos
void clear_neighbours_pos()
{
  neighbors_map.clear();
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

int dummy_closure(buzzvm_t vm)
/*
/ Dummy closure for use during update testing
----------------------------------------------------*/
{
  return buzzvm_ret0(vm);
}
}

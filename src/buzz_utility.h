#ifndef BUZZ_UTILITY_H
#define BUZZ_UTILITY_H
#include <vector>
#include <stdint.h>
#include <map>
struct pos_struct
{
  double x,y,z;
  pos_struct(double x,double y,double z):x(x),y(y),z(z){};
  pos_struct(){}
};
typedef struct pos_struct Pos_struct ;

uint16_t* u64_cvt_u16(uint64_t u64);

extern int buzz_listen(const char* type,
                       int msg_size);
void neighbour_pos_callback(std::map< int,  Pos_struct> neighbours_pos_map);
void in_msg_process(uint64_t* payload);

uint64_t* out_msg_process();

extern int buzz_script_set(const char* bo_filename,
                           const char* bdbg_filename);
extern void buzz_script_step();

extern void buzz_script_destroy();

extern int buzz_script_done();

#endif

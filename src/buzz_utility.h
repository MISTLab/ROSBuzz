#ifndef BUZZ_UTILITY_H
#define BUZZ_UTILITY_H
#include <vector>
#include <stdint.h>
struct pos_struct
{
  int id;
  double x,y,z;
  pos_struct(int id,double x,double y,double z):id(id),x(x),y(y),z(z){};
};

uint16_t* u64_cvt_u16(uint64_t u64);

extern int buzz_listen(const char* type,
                       int msg_size);
void neighbour_pos_callback(std::vector<pos_struct> pos_vect);
void in_msg_process(unsigned long int payload[]);

uint64_t* out_msg_process();

extern int buzz_script_set(const char* bo_filename,
                           const char* bdbg_filename);
extern void buzz_script_step();

extern void buzz_script_destroy();

extern int buzz_script_done();

#endif

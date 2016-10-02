#ifndef BUZZ_UTILITY_H
#define BUZZ_UTILITY_H
#include <vector>

struct pos_struct
{
  int id;
  double x,y,z;
  pos_struct(int id,double x,double y,double z):id(id),x(x),y(y),z(z){};
};


extern int buzz_listen(const char* type,
                       int msg_size);
void in_msg_process(unsigned long int payload[], std::vector<pos_struct> pos_vect);

unsigned long int* out_msg_process();

extern int buzz_script_set(const char* bo_filename,
                           const char* bdbg_filename);
extern void buzz_script_step();

extern void buzz_script_destroy();

extern int buzz_script_done();

#endif

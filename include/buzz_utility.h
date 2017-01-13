#pragma once
#include <stdio.h>
#include "buzz_utility.h"
#include "buzzuav_closures.h"
#include "buzz_update.h"
#include <buzz/buzzdebug.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <stdint.h>
#include <map>

using namespace std;
namespace buzz_utility{

struct pos_struct
{
  double x,y,z;
  pos_struct(double x,double y,double z):x(x),y(y),z(z){};
  pos_struct(){}
};

typedef struct pos_struct Pos_struct ;

uint16_t* u64_cvt_u16(uint64_t u64);

int buzz_listen(const char* type,
                       int msg_size);

void neighbour_pos_callback(std::map< int,  Pos_struct> neighbours_pos_map);

void in_msg_process(uint64_t* payload);

uint64_t* out_msg_process();

int buzz_script_set(const char* bo_filename,
                           const char* bdbg_filename, int robot_id);

int buzz_update_set(uint8_t* UP_BO_BUF, const char* bdbg_filename,size_t bcode_size);

int buzz_update_init_test(uint8_t* UP_BO_BUF, const char* bdbg_filename,size_t bcode_size);

void buzz_script_step();

void buzz_script_destroy();

int buzz_script_done();

int update_step_test();

uint16_t get_robotid();

uint8_t get_rid_uint8compac(int rid_old);

}

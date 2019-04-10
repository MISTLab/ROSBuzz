#ifndef BUZZ_UPDATE_H
#define BUZZ_UPDATE_H

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <sys/inotify.h>
#include <buzz/buzztype.h>
#include <buzz/buzzdict.h>
#include <buzz/buzzdarray.h>
#include <buzz/buzzvstig.h>
#include <rosbuzz/buzz_utility.h>
#include <fstream>

#define delete_p(p)                                                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    free(p);                                                                                                           \
    p = NULL;                                                                                                          \
  } while (0)
namespace buzz_update
{
static const uint16_t CODE_REQUEST_PADDING = 250;
static const uint16_t MIN_UPDATE_PACKET = 251;
static const uint16_t UPDATE_CODE_HEADER_SIZE = 5;
static const uint16_t TIMEOUT_FOR_ROLLBACK = 50;
/*********************/
/*   Updater states */
/********************/

typedef enum {
  CODE_RUNNING = 0,  // Code executing
  CODE_STANDBY,      // Standing by for others to update
} code_states_e;

/*********************/
/*Message types     */
/********************/

typedef enum {
  SENT_CODE = 0,  // Broadcast code
  RESEND_CODE,    // ReBroadcast request
} code_message_e;

/*************************/
/*Updater message queue */
/*************************/

struct updater_msgqueue_s
{
  uint8_t* queue;
  uint8_t* size;
};
typedef struct updater_msgqueue_s* updater_msgqueue_t;

struct updater_code_s
{
  uint8_t* bcode;
  uint8_t* bcode_size;
};
typedef struct updater_code_s* updater_code_t;

/**************************/
/*Updater data*/
/**************************/

struct buzz_updater_elem_s
{
  /* robot id  */
  // uint16_t robotid;
  /*current Bytecode content */
  uint8_t* bcode;
  /*old Bytecode name */
  const char* old_bcode;
  /*current bcode size*/
  size_t* bcode_size;
  /*Update patch*/
  uint8_t* patch;
  /* Update patch size*/
  size_t* patch_size;
  /*current Bytecode content */
  uint8_t* standby_bcode;
  /*current bcode size*/
  size_t* standby_bcode_size;
  /*updater out msg queue */
  updater_msgqueue_t outmsg_queue;
  /*updater in msg queue*/
  updater_msgqueue_t inmsg_queue;
  /*Current state of the updater one in code_states_e ENUM*/
  int* mode;
  uint8_t* update_no;
};
typedef struct buzz_updater_elem_s* buzz_updater_elem_t;

/**************************************************************************/
/*Updater routine from msg processing to file checks to be called from main*/
/**************************************************************************/
void update_routine();

/************************************************/
/*Initalizes the updater */
/************************************************/
int init_update_monitor(const char* bo_filename, const char* stand_by_script, const char* dbgfname, int robot_id);

/*********************************************************/
/*Appends buffer of given size to in msg queue of updater*/
/*********************************************************/

void code_message_inqueue_append(uint8_t* msg, uint16_t size);

/*********************************************************/
/*Processes messages inside the queue of the updater*/
/*********************************************************/

void code_message_inqueue_process();

/*****************************************************/
/*Obtains messages from out msgs queue of the updater*/
/*******************************************************/
uint8_t* getupdater_out_msg();

/******************************************************/
/*Obtains out msg queue size*/
/*****************************************************/
uint8_t* getupdate_out_msg_size();

/**************************************************/
/*Destroys the out msg queue*/
/*************************************************/
void destroy_out_msg_queue();

// buzz_updater_elem_t get_updater();
/***************************************************/
/*Sets bzz file name*/
/***************************************************/
void set_bzz_file(const char* in_bzz_file, bool dbg);

/****************************************************/
/*Tests the code from a buffer*/
/***************************************************/

int test_set_code(uint8_t* BO_BUF, const char* dbgfname, size_t bcode_size);

/****************************************************/
/*Destroys the updater*/
/***************************************************/

void destroy_updater();

/****************************************************/
/*Checks for updater message*/
/***************************************************/

int is_msg_present();

/****************************************************/
/*Compiles a bzz script to bo and bdbg*/
/***************************************************/

int compile_bzz(std::string bzz_file);

/****************************************************/
/*Set number of robots in the updater*/
/***************************************************/

void updates_set_robots(int robots);
}
#endif

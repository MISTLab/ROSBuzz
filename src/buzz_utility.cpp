#define _GNU_SOURCE
#include <stdio.h>

#include "buzz_utility.h"
#include "buzzuav_closures.h"
#include <buzz/buzzdebug.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <vector>
#include <iostream>
#include <bitset>
using namespace std;
/****************************************/
/****************************************/

static buzzvm_t    VM              = 0;
static char*       BO_FNAME        = 0;
static uint8_t*    BO_BUF          = 0;
static buzzdebug_t DBG_INFO        = 0;
static int         MSG_SIZE        = 32;
static int         TCP_LIST_STREAM = -1;
static int         TCP_COMM_STREAM = -1;
static uint8_t*    STREAM_SEND_BUF = NULL;




/****************************************/

void neighbour_pos_callback(std::vector<pos_struct> pos_vect){
 /* Reset neighbor information */
   buzzneighbors_reset(VM);
  /* Get robot id and update neighbor information */
for(int i=0;i<pos_vect.size();i++){
      buzzneighbors_add(VM,
                        pos_vect[i].id,
                        pos_vect[i].x,
                        pos_vect[i].y,
                        pos_vect[i].z);
}

}

uint16_t* u64_cvt_u16(uint64_t u64){
   uint16_t* out = new uint16_t[4];
   uint32_t int32_1 = u64 & 0xFFFFFFFF;
   uint32_t int32_2 = (u64 & 0xFFFFFFFF00000000 ) >> 32;
    out[0] = int32_1 & 0xFFFF;
    out[1] = (int32_1 & (0xFFFF0000) ) >> 16;
    out[2] = int32_2 & 0xFFFF;
    out[3] = (int32_2 & (0xFFFF0000) ) >> 16;
   //cout << " values " <<out[0] <<"  "<<out[1] <<"  "<<out[2] <<"  "<<out[3] <<"  ";
return out;

} 

void in_msg_process(unsigned long int payload[]){

   printf("inside payload");  
   /* Go through messages and add them to the FIFO */
   for(size_t i = 0; i < sizeof(payload)/sizeof(payload[0]); ++i) {
      /* Copy packet into temporary buffer */
      long unsigned int* pl = (long unsigned int*) &payload[i];
      size_t tot = 0;
       //fprintf(stderr, "[DEBUG] Processing packet %p from %f\n", pl, neighbour[0]);
      
      /* Go through the messages until there's nothing else to read */
      uint16_t unMsgSize;
      do {
         /* Get payload size */
         unMsgSize = *(uint16_t*)(pl + tot);
	 tot += sizeof(uint16_t);
         /* Append message to the Buzz input message queue */
         if(unMsgSize > 0 && unMsgSize <= MSG_SIZE - tot) {
            buzzinmsg_queue_append(VM->inmsgs,
                                   buzzmsg_payload_frombuffer(pl +tot, unMsgSize));
            tot += unMsgSize;

         }
      }while(MSG_SIZE - tot > sizeof(uint16_t) && unMsgSize > 0);
   }
   /* Process messages */
buzzvm_process_inmsgs(VM);

}

uint64_t* out_msg_process(){
   // buzzvm_process_outmsgs(VM);
   buzzvm_process_outmsgs(VM);
   uint8_t* buff_send =(uint8_t*)malloc(MSG_SIZE);
   memset(buff_send, 0, MSG_SIZE);
   ssize_t tot = sizeof(uint16_t);
    /* Send robot id */
   *(uint16_t*)(buff_send+tot) = (uint16_t) VM->robot;
    tot += sizeof(uint16_t);
   /* Send messages from FIFO */
   do {
      /* Are there more messages? */
      if(buzzoutmsg_queue_isempty(VM->outmsgs)) break;
      /* Get first message */
      buzzmsg_payload_t m = buzzoutmsg_queue_first(VM->outmsgs);
      /* Make sure the next message fits the data buffer */
      if(tot + buzzmsg_payload_size(m) + sizeof(uint16_t)
         >
         MSG_SIZE) {
         printf("buzz payload does not fit");
         buzzmsg_payload_destroy(&m);
         break;
      }
       
      /* Add message length to data buffer */
      *(uint16_t*)(buff_send + tot) = (uint16_t)buzzmsg_payload_size(m);
      tot += sizeof(uint16_t);
   
      /* Add payload to data buffer */
      memcpy(buff_send + tot, m->data, buzzmsg_payload_size(m));
      tot += buzzmsg_payload_size(m);
   
      //fprintf(stderr, "[DEBUG] coppied  = %u\n",
      //         *(uint64_t*)(buff_send+tot));	   
      /* Get rid of message */
      buzzoutmsg_queue_next(VM->outmsgs);
      buzzmsg_payload_destroy(&m);
   } while(1);

   int total_size =(ceil((float)tot/(float)sizeof(uint64_t))); 
   *(uint16_t*)buff_send = (uint16_t) total_size;   
   
  for(int i=0;i<16;i++){
   uint16_t* temp_buff =(uint16_t*)(buff_send+(i*2));
   cout << "buff_send address :" << (uint16_t*) buff_send+(i*2);
   uint16_t *p = reinterpret_cast<uint16_t*>(temp_buff);
      cout <<"   buff_send data " << *p <<endl;
  } 
  //uint16_t temp_buff1[4]; //=32;
  uint64_t* payload_64 = new uint64_t[total_size];
  //payload_64= temp_buff1 | (temp_buff1 << 16) | (temp_buff1 << 32) | (temp_buff1 << 48) ;
 
 memcpy((void*)payload_64, (void*)buff_send, total_size*sizeof(uint64_t));
 //cout << "payload64 "<<payload_64[0];
 //   std::string binary = std::bitset<64>(payload_64[0]).to_string(); //to binary
 //   std::cout<<"payload_64 binary " <<binary<<"\n";

 //   unsigned long decimal = std::bitset<64>(binary).to_ulong();
 //   std::cout<< " decimal "<<decimal<<"\n";
    
/*
     uint16_t* out;
    for(int dumb =0;dumb<total_size;dumb++){
     out = u64_cvt_u16(payload_64[dumb]);
      for(int i=0; i<4;i++){
      cout<< " cvt value"<<out[i]<<endl;
      }
     }
  delete[] out;
 /*
   /* Send message */
return payload_64;
}
/****************************************/

/****************************************/

static const char* buzz_error_info() {
   buzzdebug_entry_t dbg = *buzzdebug_info_get_fromoffset(DBG_INFO, &VM->pc);
   char* msg;
   if(dbg != NULL) {
      asprintf(&msg,
               "%s: execution terminated abnormally at %s:%" PRIu64 ":%" PRIu64 " : %s\n\n",
               BO_FNAME,
               dbg->fname,
               dbg->line,
               dbg->col,
               VM->errormsg);
   }
   else {
      asprintf(&msg,
               "%s: execution terminated abnormally at bytecode offset %d: %s\n\n",
               BO_FNAME,
               VM->pc,
               VM->errormsg);
   }
   return msg;
}

/****************************************/
/****************************************/

static int buzz_register_hooks() {
   buzzvm_pushs(VM,  buzzvm_string_register(VM, "print", 1));
   buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzros_print));
   buzzvm_gstore(VM);
   buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_goto", 1));
   buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_goto));
   buzzvm_gstore(VM);
   buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_takeoff", 1));
   buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_takeoff));
   buzzvm_gstore(VM);
   buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_gohome", 1));
   buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_gohome));
   buzzvm_gstore(VM);
   buzzvm_pushs(VM,  buzzvm_string_register(VM, "uav_land", 1));
   buzzvm_pushcc(VM, buzzvm_function_register(VM, buzzuav_land));
   buzzvm_gstore(VM);
   return BUZZVM_STATE_READY;
}

/****************************************/

/****************************************/

int buzz_script_set(const char* bo_filename,
                    const char* bdbg_filename) {
   /* Get hostname */
   char hstnm[30];
   gethostname(hstnm, 30);
   /* Make numeric id from hostname */
   /* NOTE: here we assume that the hostname is in the format Knn */
   int id = strtol(hstnm + 1, NULL, 10);
   /* Reset the Buzz VM */
   if(VM) buzzvm_destroy(&VM);
   VM = buzzvm_new(id);
   /* Get rid of debug info */
   if(DBG_INFO) buzzdebug_destroy(&DBG_INFO);
   DBG_INFO = buzzdebug_new();
   /* Read bytecode and fill in data structure */
   FILE* fd = fopen(bo_filename, "rb");
   if(!fd) {
      perror(bo_filename);
      return 0;
   }
   fseek(fd, 0, SEEK_END);
   size_t bcode_size = ftell(fd);
   rewind(fd);
   BO_BUF = (uint8_t*)malloc(bcode_size);
   if(fread(BO_BUF, 1, bcode_size, fd) < bcode_size) {
      perror(bo_filename);
      buzzvm_destroy(&VM);
      buzzdebug_destroy(&DBG_INFO);
      fclose(fd);
      return 0;
   }
   fclose(fd);
   /* Read debug information */
   if(!buzzdebug_fromfile(DBG_INFO, bdbg_filename)) {
      buzzvm_destroy(&VM);
      buzzdebug_destroy(&DBG_INFO);
      perror(bdbg_filename);
      return 0;
   }
   /* Set byte code */
   if(buzzvm_set_bcode(VM, BO_BUF, bcode_size) != BUZZVM_STATE_READY) {
      buzzvm_destroy(&VM);
      buzzdebug_destroy(&DBG_INFO);
      fprintf(stdout, "%s: Error loading Buzz script\n\n", bo_filename);
      return 0;
   }
   /* Register hook functions */
   if(buzz_register_hooks() != BUZZVM_STATE_READY) {
      buzzvm_destroy(&VM);
      buzzdebug_destroy(&DBG_INFO);
      fprintf(stdout, "%s: Error registering hooks\n\n", bo_filename);
      return 0;
   }
   /* Save bytecode file name */
   BO_FNAME = strdup(bo_filename);
   /* Execute the global part of the script */
   buzzvm_execute_script(VM);
   /* Call the Init() function */
   buzzvm_function_call(VM, "init", 0);
   /* All OK */
   return 1;
}

/****************************************/
/****************************************/

struct buzzswarm_elem_s {
   buzzdarray_t swarms;
   uint16_t age;
};
typedef struct buzzswarm_elem_s* buzzswarm_elem_t;

void check_swarm_members(const void* key, void* data, void* params) {
   buzzswarm_elem_t e = *(buzzswarm_elem_t*)data;
   int* status = (int*)params;
   if(*status == 3) return;
   if(buzzdarray_size(e->swarms) != 1) {
      fprintf(stderr, "Swarm list size is not 1\n");
      *status = 3;
   }
   else {
      int sid = 1;
      if(*buzzdict_get(VM->swarms, &sid, uint8_t) &&
         buzzdarray_get(e->swarms, 0, uint16_t) != sid) {
         fprintf(stderr, "I am in swarm #%d and neighbor is in %d\n",
                 sid,
                 buzzdarray_get(e->swarms, 0, uint16_t));
         *status = 3;
         return;
      }
      sid = 2;
      if(*buzzdict_get(VM->swarms, &sid, uint8_t) &&
         buzzdarray_get(e->swarms, 0, uint16_t) != sid) {
         fprintf(stderr, "I am in swarm #%d and neighbor is in %d\n",
                 sid,
                 buzzdarray_get(e->swarms, 0, uint16_t));
         *status = 3;
         return;
      }
   }
}

void buzz_script_step() {

   
   //buzzvm_process_inmsgs(VM);
   /*
    * Update sensors
    */
   buzzuav_update_battery(VM);
   buzzuav_update_prox(VM);
   /*
    * Call Buzz step() function
    */
   if(buzzvm_function_call(VM, "step", 0) != BUZZVM_STATE_READY) {
      fprintf(stderr, "%s: execution terminated abnormally: %s\n\n",
              BO_FNAME,
              buzz_error_info());
      buzzvm_dump(VM);
   }
   /* Print swarm */
   buzzswarm_members_print(stdout, VM->swarmmembers, VM->robot);
   /* Check swarm state */
   int status = 1;
   buzzdict_foreach(VM->swarmmembers, check_swarm_members, &status);
   if(status == 1 &&
      buzzdict_size(VM->swarmmembers) < 9)
      status = 2;
   buzzvm_pushs(VM, buzzvm_string_register(VM, "swarm_status", 1));
   buzzvm_pushi(VM, status);
   buzzvm_gstore(VM);
}

/****************************************/
/****************************************/

void buzz_script_destroy() {
   
   /* Get rid of stream buffer */
   //free(STREAM_SEND_BUF);
   /* Get rid of virtual machine */
   if(VM) {
      if(VM->state != BUZZVM_STATE_READY) {
         fprintf(stderr, "%s: execution terminated abnormally: %s\n\n",
                 BO_FNAME,
                 buzz_error_info());
         buzzvm_dump(VM);
      }
      buzzvm_function_call(VM, "destroy", 0);
      buzzvm_destroy(&VM);
      free(BO_FNAME);
      buzzdebug_destroy(&DBG_INFO);
   }
   fprintf(stdout, "Script execution stopped.\n");
}

/****************************************/
/****************************************/

int buzz_script_done() {
   return VM->state != BUZZVM_STATE_READY;
}

/****************************************/
/****************************************/



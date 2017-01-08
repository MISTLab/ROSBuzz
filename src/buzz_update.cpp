#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h> 
#include <unistd.h> 
#include <sys/inotify.h>
#include "buzz_update.h"
#include <buzz_utility.h>
#include <string.h>
#include <buzz/buzzdebug.h>
#include <sys/time.h>
#define MAX_BUCKETS 10

/*Temp for data collection*/
//static int neigh=-1;
//static struct timeval t1, t2;
//static clock_t begin;
//static clock_t end;
//void collect_data();
/*Temp end */
static int fd,wd =0;
static int old_update =0;
static buzz_updater_elem_t updater;
static int no_of_robot;
static char* dbgf_name;
static const char* bzz_file;

uint32_t buzz_updater_hashfunp (const void* key){
return (*(uint16_t*)key);
}


int buzz_updater_key_cmp(const void* a, const void* b){
if( *(uint16_t*)a < *(uint16_t*)b ) return -1;
if( *(uint16_t*)a > *(uint16_t*)b ) return 1;
return 0;
}

void updater_entry_destroy(const void* key, void* data, void* params) {
  // fprintf(stdout,"freeing element.\n");
   free((void*)key);
   free((void*)data);
}



void standby_barrier_test(const void* key, const void* data, void* tmp){
	//fprintf(stdout,"Checking barrier for robot :%i it has barrier : %i.\n",*(uint16_t*) key,*(uint8_t*) data );
	if(*(uint8_t*) data == CODE_STANDBY || *(uint8_t*) data == CODE_RUNNING  ){
		if(updater->update_no==*(uint8_t*) (data+sizeof(uint8_t)) )
			(*(uint8_t*)tmp)+=1;
	}
}

void init_update_monitor(const char* bo_filename,int barrier){
	fprintf(stdout,"intiialized file monitor.\n");
	fd=inotify_init1(IN_NONBLOCK);
	if ( fd < 0 ) {
	    perror( "inotify_init error" );
	  }
	/* watch /.bzz file for any activity and report it back to me */
	wd=inotify_add_watch(fd, bzz_file,IN_ALL_EVENTS );

	uint8_t*    BO_BUF          = 0;
	FILE* fp = fopen(bo_filename, "rb");

	   if(!fp) {
	      perror(bo_filename);
	    
	   }
	   fseek(fp, 0, SEEK_END);

	   size_t bcode_size = ftell(fp);
	   rewind(fp);

	   BO_BUF = (uint8_t*)malloc(bcode_size);
	   if(fread(BO_BUF, 1, bcode_size, fp) < bcode_size) {
	      perror(bo_filename);
	     
	      fclose(fp);
	      //return 0;
	   }
	   fclose(fp);

	 updater = (buzz_updater_elem_t)malloc(sizeof(struct buzz_updater_elem_s));
	  /* Create a new table for updater*/
	  updater->bcode = BO_BUF;
	  updater->bcode_size = bcode_size;
	  updater->mode=CODE_RUNNING;
	  updater->robotid= buzz_utility::get_robotid();
	  updater->state_dict=buzzdict_new(MAX_BUCKETS,
                        sizeof(uint16_t),
                        sizeof(uint16_t),
                        buzz_updater_hashfunp,
                        buzz_updater_key_cmp,
                        updater_entry_destroy);
	  updater->outmsg_queue = NULL;
	  updater->inmsg_queue = NULL;
          updater->update_no=0;
	  no_of_robot=barrier;
	  //updater->outmsg_queue=
	  // update_table->barrier=nvs;

}

int check_update(){
        
	struct inotify_event *event;
	char buf[1024];
	int check =0;
	int i=0;
	int len=read(fd,buf,1024);
	while(i<len){
		event=(struct inotify_event *) &buf[i];

		/* file was modified this flag is true in nano and self delet in gedit and other editors */
                //fprintf(stdout,"inside file monitor.\n");
		if(event->mask & (IN_MODIFY| IN_DELETE_SELF)){
		/*respawn watch if the file is self deleted */
			inotify_rm_watch(fd,wd);
			close(fd);
			fd=inotify_init1(IN_NONBLOCK);
			wd=inotify_add_watch(fd,bzz_file,IN_ALL_EVENTS);
			//fprintf(stdout,"event.\n");
			/* To mask multiple writes from editors*/
			if(!old_update){
			check=1;
			old_update =1;
			}

		}

		/* update index to start of next event */
		i+=sizeof(struct inotify_event)+event->len;

	}

	if (!check) old_update=0;
	/*if(update){
	buzz_script_set(update_bo, update_bdbg);
	update = 0;
	}*/
	return check;
}


void code_message_outqueue_append(int type){
uint16_t size =0;
//fprintf(stdout,"updater queue append.\n");
	//if(updater->outmsg_queue == NULL)
	//updater->outmsg_queue=(updater_msgqueue_t)malloc(sizeof(struct updater_msgqueue_s));
	//else {
	//	destroy_out_msg_queue();
		updater->outmsg_queue=(updater_msgqueue_t)malloc(sizeof(struct updater_msgqueue_s));
	    // }
	if(type==SEND_CODE){
		updater->outmsg_queue->queue = (uint8_t*)malloc((6*sizeof(uint8_t))+updater->bcode_size);
		updater->outmsg_queue->size  = (uint8_t*)malloc(sizeof(uint16_t));	
		//*(uint16_t*)updater->outmsg_queue->size =(3*sizeof(uint8_t))+sizeof(size_t)+updater->bcode_size;
		
		*(uint16_t*)(updater->outmsg_queue->queue+size) = (uint16_t) updater->robotid;
		size+=sizeof(uint16_t);
		*(uint8_t*)(updater->outmsg_queue->queue+size) = (uint8_t) updater->mode;
		size+=sizeof(uint8_t);
		*(uint8_t*)(updater->outmsg_queue->queue+size) =  updater->update_no;
		size+=sizeof(uint8_t);
		*(uint16_t*)(updater->outmsg_queue->queue+size) = (uint16_t) updater->bcode_size;
		size+=sizeof(uint16_t);
		memcpy(updater->outmsg_queue->queue+size, updater->bcode, updater->bcode_size);
		size+=updater->bcode_size;
		FILE *fp;
		fp=fopen("update.bo", "wb");
   		fwrite((updater->bcode), updater->bcode_size, 1, fp);
		fclose(fp);
		//fprintf(stdout,"[Debug : ]size = %i \n",(int)sizeof(size_t));
		*(uint16_t*)updater->outmsg_queue->size=size;
		/*Update local dictionary*/
		uint8_t* dict_update=(uint8_t*)malloc(sizeof(uint16_t));
		*(uint8_t*)dict_update=updater->mode;
		*(uint8_t*)(dict_update+sizeof(uint8_t))=updater->update_no;
		buzzdict_set(updater->state_dict,(uint8_t*) &(updater->robotid), (uint8_t*)dict_update);
		delete_p(dict_update);		
		
	}
	else if(type==STATE_MSG){
		updater->outmsg_queue->queue = (uint8_t*)malloc(3*sizeof(uint8_t));
		updater->outmsg_queue->size  = (uint8_t*)malloc(sizeof(uint16_t));	
		//*(uint16_t*)updater->outmsg_queue->size = 3*sizeof(uint8_t);
		*(uint16_t*)(updater->outmsg_queue->queue+size) = (uint16_t) updater->robotid;
		size+=sizeof(uint16_t);
		*(uint8_t*)(updater->outmsg_queue->queue+size) = (uint8_t) updater->mode;
		size+=sizeof(uint8_t);
		*(uint8_t*)(updater->outmsg_queue->queue+size) =  updater->update_no;
		size+=sizeof(uint8_t);
		*(uint16_t*)updater->outmsg_queue->size=size;
		/*Update local dictionary*/
		uint8_t* dict_update=(uint8_t*)malloc(sizeof(uint16_t));
		*(uint8_t*)dict_update=updater->mode;
		*(uint8_t*)(dict_update+sizeof(uint8_t))=updater->update_no;
		buzzdict_set(updater->state_dict,(uint8_t*) &(updater->robotid), (uint8_t*)dict_update);
		delete_p(dict_update);
							
		}

}

void code_message_inqueue_append(uint8_t* msg,uint16_t size){
updater->inmsg_queue=(updater_msgqueue_t)malloc(sizeof(struct updater_msgqueue_s));
updater->inmsg_queue->queue = (uint8_t*)malloc(size);
updater->inmsg_queue->size  = (uint8_t*)malloc(sizeof(uint16_t));
memcpy(updater->inmsg_queue->queue, msg, size);
*(uint16_t*)updater->inmsg_queue->size  = size;

}

void code_message_inqueue_process(){
int size=0;
	//fprintf(stdout,"received state : %i from robot : %i\n",*(uint8_t*)(updater->inmsg_queue->queue+sizeof(uint16_t)),*(uint16_t*)updater->inmsg_queue->queue);
	if(*(uint8_t*)(updater->inmsg_queue->queue+sizeof(uint16_t)) == CODE_UPDATE){
		buzzdict_set(updater->state_dict, updater->inmsg_queue->queue,(updater->inmsg_queue->queue+sizeof(uint16_t)));
		size +=3*sizeof(uint8_t);
		if(*(uint8_t*)(updater->inmsg_queue->queue+size) > (updater->update_no)){
			updater->update_no=*(uint8_t*)(updater->inmsg_queue->queue+size);
			size+=sizeof(uint8_t);
			if(updater->mode==CODE_RUNNING){				
				uint16_t update_bcode_size =*(uint16_t*)(updater->inmsg_queue->queue+size);
				size +=sizeof(uint16_t);	
				FILE *fp;
				fp=fopen("update.bo", "wb");
		   		fwrite((updater->inmsg_queue->queue+size), update_bcode_size, 1, fp);
				fclose(fp);
				if(buzz_utility::buzz_update_init_test((updater->inmsg_queue->queue+size), dbgf_name,(size_t)update_bcode_size)){
					fprintf(stdout,"Initializtion of script test passed\n");
					if(buzz_utility::update_step_test()){
						/*data logging*/
						//neigh=1;
						//gettimeofday(&t1, NULL);
						//fprintf(stdout,"start and end time in queue process of update Info : %f,%f",(double)begin,(double)end);
						/*data logging*/
						fprintf(stdout,"Step test passed\n");
						updater->mode=CODE_UPDATE;
						//fprintf(stdout,"updater value = %i\n",updater->mode);
						free(updater->bcode);
						updater->bcode = (uint8_t*)malloc(update_bcode_size);
						memcpy(updater->bcode, updater->inmsg_queue->queue+size, update_bcode_size);
						//updater->bcode = BO_BUF;
		  				updater->bcode_size = (size_t)update_bcode_size;
						code_message_outqueue_append(SEND_CODE);
						updater->mode=CODE_NEIGHBOUR;

						//return updater->mode;				
						//return 0;
						}
					/*Unable to step something wrong*/
					else{
						fprintf(stdout,"Step test failed, stick to old script\n");
						buzz_utility::buzz_update_set((updater)->bcode, dbgf_name, (updater)->bcode_size);

						//return updater->mode;
						//return 1;
					}				
				}    
				else {
					fprintf(stdout,"Initialiation test failed, stick to old script\n");
					buzz_utility::buzz_update_set((updater)->bcode, dbgf_name, (updater)->bcode_size);
					//return updater->mode;
					//return 1;				
					}
			}
		}

	}
	else {
	buzzdict_set(updater->state_dict,updater->inmsg_queue->queue, updater->inmsg_queue->queue+sizeof(uint16_t));
	}
//fprintf(stdout,"in queue freed\n");
delete_p(updater->inmsg_queue->queue);
delete_p(updater->inmsg_queue->size);
delete_p(updater->inmsg_queue);
}
int update_routine(const char* bcfname,
                           const char* dbgfname, int destroy){
dbgf_name=(char*)dbgfname;
//fprintf(stdout,"[Debug : ]updater value = %i \n",updater->mode);
	if(updater->mode==CODE_RUNNING){
		if(check_update()){
			std::string bzzfile_name(bzz_file);
			stringstream bzzfile_in_compile;
	        	std::string  path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/"));
			bzzfile_in_compile<<path<<"/";
			path = bzzfile_in_compile.str();
			bzzfile_in_compile.str("");
			std::string  name = bzzfile_name.substr(bzzfile_name.find_last_of("/\\") + 1);
 			name = name.substr(0,name.find_last_of("."));
			bzzfile_in_compile << "bzzparse "<<bzzfile_name<<" "<<path<< name<<".basm";
		
			FILE *fp;
			int comp=0;
			char buf[128];
			fprintf(stdout,"Update found \nUpdating script ...\n");
			
			if ((fp = popen(bzzfile_in_compile.str().c_str(), "r")) == NULL) { // to change file edit
				fprintf(stdout,"Error opening pipe!\n");
		 	   	}
		
				 while (fgets(buf, 128, fp) != NULL) {
					fprintf(stdout,"OUTPUT: %s \n", buf);
					comp=1;		    		
				     	}
			bzzfile_in_compile.str("");
           		bzzfile_in_compile <<"bzzasm "<<path<<name<<".basm "<<path<<name<<".bo "<<path<<name<<".bdbg";

			if ((fp = popen(bzzfile_in_compile.str().c_str(), "r")) == NULL) { // to change file edit
			fprintf(stdout,"Error opening pipe!\n");
		 	    }
		 	 while (fgets(buf, 128, fp) != NULL) {
				fprintf(stdout,"OUTPUT: %s \n", buf);
		    	    }

	  		 if(pclose(fp) || comp)  {
	     		  fprintf(stdout,"Errors in comipilg script so staying on old script\n");
	    		  //  return -1;
	 		   }
	  		 else {
				
				uint8_t*    BO_BUF          = 0;
				FILE* fp = fopen(bcfname, "rb");  // to change file edit
				   if(!fp) {
				      perror(bcfname);
				      
				   }
				   fseek(fp, 0, SEEK_END);
				   size_t bcode_size = ftell(fp);
				   rewind(fp);
				   BO_BUF = (uint8_t*)malloc(bcode_size);
				   if(fread(BO_BUF, 1, bcode_size, fp) < bcode_size) {
				      perror(bcfname);
				      
				      fclose(fp);
				      
				   }
				   fclose(fp);
				if(buzz_utility::buzz_update_init_test(BO_BUF, dbgfname,bcode_size)){
				 fprintf(stdout,"Initializtion of script test passed\n");
					if(buzz_utility::update_step_test()){
					/*data logging*/
					//neigh=0;
					//gettimeofday(&t1, NULL);
					//fprintf(stdout,"start and end time in code running state Info : %f,%f",(double)begin,(double)end);
					/*data logging*/
					fprintf(stdout,"Step test passed\n");
					updater->update_no+=1;
					updater->mode=CODE_UPDATE;
					//fprintf(stdout,"updater value = %i\n",updater->mode);
					delete_p(updater->bcode);
					updater->bcode = (uint8_t*)malloc(bcode_size);
	  				memcpy(updater->bcode, BO_BUF, bcode_size);
	  				updater->bcode_size = bcode_size;
			                code_message_outqueue_append(SEND_CODE);
					delete_p(BO_BUF);
					return updater->mode;				
					//return 0;
					}
					/*Unable to step something wrong*/
					else{
					fprintf(stdout,"step test failed, stick to old script\n");
					buzz_utility::buzz_update_set((updater)->bcode, dbgfname, (updater)->bcode_size);
					code_message_outqueue_append(STATE_MSG);
					delete_p(BO_BUF);
					return updater->mode;
					//return 1;
					}				
				}    
				else {
					fprintf(stdout,"unable to set new script to switch to old new script\n");
					delete_p(BO_BUF);
					buzz_utility::buzz_update_set((updater)->bcode, dbgfname, (updater)->bcode_size);
					code_message_outqueue_append(STATE_MSG);
					return updater->mode;
					//return 1;				
					}
		       	}
	     
		}
		code_message_outqueue_append(STATE_MSG);
	}

	else if (updater->mode==CODE_UPDATE){
	
		code_message_outqueue_append(SEND_CODE);
		updater->mode=CODE_STANDBY;

	}
	else if (updater->mode==CODE_NEIGHBOUR){
		updater->mode=CODE_STANDBY;
	}
	else if (updater->mode==CODE_STANDBY){

		uint8_t* tmp =(uint8_t*)malloc(sizeof(uint8_t));
		*(uint8_t*)tmp=0;
		buzzdict_foreach( updater->state_dict,reinterpret_cast<buzzdict_elem_funp>(standby_barrier_test), tmp);
		fprintf(stdout,"Standby barrier ................... %i\n",*(uint8_t*)tmp);
		code_message_outqueue_append(STATE_MSG);
		if(*(uint8_t*)tmp==no_of_robot) { 
			updater->mode=CODE_RUNNING;
			//gettimeofday(&t2, NULL);
			//fprintf(stdout,"start and end time in standby state Info : %f,%f",(double)begin,(double)end);
			//collect_data();
			buzz_utility::buzz_update_set((updater)->bcode, dbgf_name, (updater)->bcode_size);	
			}
		
		delete_p(tmp);
		//fprintf(stdout,"freed tmp\n");
		
	}
	
	if(destroy){
          
 	  destroy_updater();
	  fprintf(stdout,"updater destoryed.\n");
	}
return updater->mode;
}




uint8_t* getupdater_out_msg(){
return (uint8_t*)updater->outmsg_queue->queue;
}

uint8_t* getupdate_out_msg_size(){
//fprintf(stdout,"[Debug : ]size = %i \n",*(uint16_t*)updater->outmsg_queue->size);
		
return updater->outmsg_queue->size;
}

void destroy_out_msg_queue(){
//fprintf(stdout,"out queue freed\n");
delete_p(updater->outmsg_queue->queue);
delete_p(updater->outmsg_queue->size);
delete_p(updater->outmsg_queue);
}

int get_update_mode(){
return updater->mode;
}

void destroy_updater(){
buzz_utility::buzz_script_destroy();
//dictonary
//fprintf(stdout,"freeing dict.\n");
buzzdict_t* updater_dict_ptr=&(updater->state_dict);
buzzdict_destroy((updater_dict_ptr));
//fprintf(stdout,"freeing complete dict.\n");
delete_p(updater->bcode);
//fprintf(stdout,"freeing complete bcode.\n");

if(updater->outmsg_queue){
delete_p(updater->outmsg_queue->queue);
delete_p(updater->outmsg_queue->size);
delete_p(updater->outmsg_queue);
}
if(updater->inmsg_queue){
delete_p(updater->inmsg_queue->queue);
delete_p(updater->inmsg_queue->size);
delete_p(updater->inmsg_queue);
}
//
inotify_rm_watch(fd,wd);
close(fd);
}

void set_bzz_file(const char* in_bzz_file){
bzz_file=in_bzz_file;
}
void collect_data(){
//fprintf(stdout,"start and end time in data collection Info : %f,%f",(double)begin,(double)end);
//double time_spent =   (t2.tv_sec - t1.tv_sec) * 1000.0; //(double)(end - begin) / CLOCKS_PER_SEC;
//time_spent += (t2.tv_usec - t1.tv_usec) / 1000.0;
//fprintf(stdout,"Data logger Info : %i,%i,%f,%i,%i,%i\n",(int)updater->robotid,neigh,time_spent,(int)updater->bcode_size,(int)no_of_robot,(int)updater->update_no);
//FILE *Fileptr;
//Fileptr=fopen("logger.csv", "a");
//fprintf(Fileptr,"%i,%i,%f,%i,%i,%i\n",(int)updater->robotid,neigh,time_spent,(int)updater->bcode_size,(int)no_of_robot,(int)updater->update_no);
//fclose(Fileptr);
}


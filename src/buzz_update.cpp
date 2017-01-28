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
static int neigh=-1;
static int 	    updater_msg_ready ;
void init_update_monitor(const char* bo_filename, const char* stand_by_script,int barrier){
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
	uint8_t* STD_BO_BUF          = 0;
	fp = fopen(stand_by_script, "rb");

	   if(!fp) {
	      perror(stand_by_script);
	    
	   }
	   fseek(fp, 0, SEEK_END);

	   size_t stdby_bcode_size = ftell(fp);
	   rewind(fp);

	   STD_BO_BUF = (uint8_t*)malloc(stdby_bcode_size);
	   if(fread(STD_BO_BUF, 1, stdby_bcode_size, fp) < stdby_bcode_size) {
	      perror(stand_by_script);
	     
	      fclose(fp);
	      //return 0;
	   }
	   fclose(fp);

	 updater = (buzz_updater_elem_t)malloc(sizeof(struct buzz_updater_elem_s));
	  /* Create a new table for updater*/
	  updater->bcode = BO_BUF;
	  updater->outmsg_queue = NULL;
	  updater->inmsg_queue = NULL;
	  updater->bcode_size = (size_t*) malloc(sizeof(size_t));
	  updater->update_no = (uint8_t*) malloc(sizeof(uint16_t));
	  *(uint16_t*)(updater->update_no) =0;
	  *(size_t*)(updater->bcode_size)=bcode_size;
	  updater->standby_bcode = STD_BO_BUF;
	  updater->standby_bcode_size = (size_t*)malloc(sizeof(size_t));
	  *(size_t*)(updater->standby_bcode_size)=stdby_bcode_size;
	  updater->mode=(int*)malloc(sizeof(int));
	  *(int*)(updater->mode)=CODE_RUNNING;
	  no_of_robot=barrier;
	  updater_msg_ready=0;
	  //neigh = 0;
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


void code_message_outqueue_append(){
	updater->outmsg_queue=(updater_msgqueue_t)malloc(sizeof(struct updater_msgqueue_s));
	uint16_t size =0;
	updater->outmsg_queue->queue = (uint8_t*)malloc(2*sizeof(uint16_t)+ *(size_t*)(updater->bcode_size));
	updater->outmsg_queue->size  = (uint8_t*)malloc(sizeof(uint16_t));	
	/*append the update no, code size and code to out msg*/
	*(uint16_t*)(updater->outmsg_queue->queue+size) = *(uint16_t*) (updater->update_no);
	size+=sizeof(uint16_t);
	*(uint16_t*)(updater->outmsg_queue->queue+size) = *(size_t*) (updater->bcode_size);
	size+=sizeof(uint16_t);
	memcpy(updater->outmsg_queue->queue+size, updater->bcode, *(size_t*)(updater->bcode_size));
	size+=(uint16_t)*(size_t*)(updater->bcode_size);
	/*FILE *fp;
	fp=fopen("update.bo", "wb");
	fwrite((updater->bcode), updater->bcode_size, 1, fp);
	fclose(fp);*/
	updater_msg_ready=1;
	*(uint16_t*)(updater->outmsg_queue->size)=size;
	
	//fprintf(stdout,"out msg append transfer code size %d\n", (int)*(size_t*) updater->bcode_size);
}

void code_message_inqueue_append(uint8_t* msg,uint16_t size){
updater->inmsg_queue=(updater_msgqueue_t)malloc(sizeof(struct updater_msgqueue_s));
fprintf(stdout,"in ms append code size %d\n", (int) size);
updater->inmsg_queue->queue = (uint8_t*)malloc(size);
updater->inmsg_queue->size  = (uint8_t*)malloc(sizeof(uint16_t));
memcpy(updater->inmsg_queue->queue, msg, size);
*(uint16_t*)(updater->inmsg_queue->size)  = size;

}

void code_message_inqueue_process(){
int size=0;
fprintf(stdout,"[debug]Updater mode %d \n", *(int*)(updater->mode) );
fprintf(stdout,"[debug] %u : current update number, %u : received update no \n",( *(uint16_t*) (updater->update_no) ), (*(uint16_t*)(updater->inmsg_queue->queue)) );
fprintf(stdout,"[debug]Updater code size %u \n",(*(uint16_t*)(updater->inmsg_queue->queue+sizeof(uint16_t)) ) );

if(  *(int*) (updater->mode) == CODE_RUNNING){		
	fprintf(stdout,"[debug]Inside inmsg code running");
	if( *(uint16_t*)(updater->inmsg_queue->queue) >  *(uint16_t*) (updater->update_no)  ){
		fprintf(stdout,"[debug]Inside update number comparision");
		uint16_t update_no=*(uint16_t*)(updater->inmsg_queue->queue);	
		size +=sizeof(uint16_t);	
		uint16_t update_bcode_size =*(uint16_t*)(updater->inmsg_queue->queue+size);
		size +=sizeof(uint16_t);	
		//fprintf(stdout,"in queue process Update no %d\n", (int) update_no);
		//fprintf(stdout,"in queue process bcode size %d\n", (int) update_bcode_size);
		//FILE *fp;
		//fp=fopen("update.bo", "wb");
		//fwrite((updater->inmsg_queue->queue+size), update_bcode_size, 1, fp);
		//fclose(fp);
		if( test_set_code((uint8_t*)(updater->inmsg_queue->queue+size),
		   (char*) dbgf_name,(size_t)update_bcode_size) ) {
		*(uint16_t*)(updater->update_no)=update_no;
		neigh=1;
		}
	}
}

//fprintf(stdout,"in queue freed\n");
delete_p(updater->inmsg_queue->queue);
delete_p(updater->inmsg_queue->size);
delete_p(updater->inmsg_queue);
}
void update_routine(const char* bcfname,
                           const char* dbgfname){
dbgf_name=(char*)dbgfname;
buzzvm_t  VM = buzz_utility::get_vm();

buzzvm_pushs(VM, buzzvm_string_register(VM, "update_no", 1));
			buzzvm_pushi(VM, *(uint16_t*)(updater->update_no));
			buzzvm_gstore(VM);
//fprintf(stdout,"[Debug : ]updater value = %i \n",updater->mode);
	if(*(int*)updater->mode==CODE_RUNNING){
		buzzvm_function_call(VM, "updated_neigh", 0);
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
				if(test_set_code(BO_BUF, dbgfname,bcode_size)){
				 uint16_t update_no =*(uint16_t*)(updater->update_no);
				*(uint16_t*)(updater->update_no) =update_no +1;
				code_message_outqueue_append();
				VM = buzz_utility::get_vm();
				fprintf(stdout,"Update no %d\n", *(uint16_t*)(updater->update_no));
				buzzvm_pushs(VM, buzzvm_string_register(VM, "update_no", 1));
				buzzvm_pushi(VM, *(uint16_t*)(updater->update_no));
				buzzvm_gstore(VM);
				neigh=0;
				}
				delete_p(BO_BUF);
		       	}
	     
		}
		
	}

	else{
		if(neigh==0 && (!is_msg_present())){ 
			fprintf(stdout,"Sending code... \n");		
			code_message_outqueue_append();
		}		
		buzzvm_pushs(VM, buzzvm_string_register(VM, "barrier_val",1));
            	buzzvm_gload(VM);
            	buzzobj_t tObj = buzzvm_stack_at(VM, 1);
            	buzzvm_pop(VM);
		fprintf(stdout,"Barrier ..................... %i \n",tObj->i.value);
		if(tObj->i.value==no_of_robot) { 
			*(int*)(updater->mode) = CODE_RUNNING;
			neigh=-1;
			//collect_data();
			buzz_utility::buzz_update_init_test((updater)->bcode, (char*)dbgfname, *(size_t*)(updater->bcode_size));
			//buzzvm_function_call(m_tBuzzVM, "updated", 0);	
			}

	}
	
}




uint8_t* getupdater_out_msg(){
return (uint8_t*)updater->outmsg_queue->queue;
}

uint8_t* getupdate_out_msg_size(){
//fprintf(stdout,"[Debug  from get out size in util: ]size = %i \n",*(uint16_t*)updater->outmsg_queue->size);
		
return (uint8_t*)updater->outmsg_queue->size;
}

int test_set_code(uint8_t* BO_BUF, const char* dbgfname,size_t bcode_size ){
	if(buzz_utility::buzz_update_init_test(BO_BUF, dbgfname,bcode_size)){
		fprintf(stdout,"Initializtion of script test passed\n");
		if(buzz_utility::update_step_test()){
			/*data logging*/
			//start =1;
			/*data logging*/
			fprintf(stdout,"Step test passed\n");
			*(int*) (updater->mode) = CODE_STANDBY;
			//fprintf(stdout,"updater value = %i\n",updater->mode);
			delete_p(updater->bcode);
			updater->bcode = (uint8_t*)malloc(bcode_size);
			memcpy(updater->bcode, BO_BUF, bcode_size);
			*(size_t*)updater->bcode_size = bcode_size;
			buzz_utility::buzz_update_init_test((updater)->standby_bcode,
			         (char*)dbgfname,(size_t) *(size_t*)(updater->standby_bcode_size));
			buzzvm_t  VM = buzz_utility::get_vm();
			buzzvm_pushs(VM, buzzvm_string_register(VM, "ROBOTS", 1));
			buzzvm_pushi(VM, no_of_robot);
			buzzvm_gstore(VM);
			return 1;
		}
		/*Unable to step something wrong*/
		else{
			if(*(int*) (updater->mode) == CODE_RUNNING){
			fprintf(stdout,"step test failed, stick to old script\n");
			buzz_utility::buzz_update_init_test((updater)->bcode, dbgfname, (size_t)*(size_t*)(updater->bcode_size));
			}
			else{
			fprintf(stdout,"step test failed, Return to stand by\n");
			buzz_utility::buzz_update_init_test((updater)->standby_bcode,
			         (char*)dbgfname,(size_t) *(size_t*)(updater->standby_bcode_size));
			buzzvm_t  VM = buzz_utility::get_vm();
			buzzvm_pushs(VM, buzzvm_string_register(VM, "ROBOTS", 1));
			buzzvm_pushi(VM, no_of_robot);
			buzzvm_gstore(VM);
			
			}
			return 0;
			
		}				
	}    
	else {
		if(*(int*) (updater->mode) == CODE_RUNNING){
		fprintf(stdout,"Initialization test failed, stick to old script\n");
		buzz_utility::buzz_update_init_test((updater)->bcode, dbgfname,(int)*(size_t*) (updater->bcode_size));
		}
		else{
		fprintf(stdout,"Initialization test failed, Return to stand by\n");
			buzz_utility::buzz_update_init_test((updater)->standby_bcode,
			         (char*)dbgfname,(size_t) *(size_t*)(updater->standby_bcode_size));
			buzzvm_t  VM = buzz_utility::get_vm();
			buzzvm_pushs(VM, buzzvm_string_register(VM, "ROBOTS", 1));
			buzzvm_pushi(VM, no_of_robot);
			buzzvm_gstore(VM);
		}
		return 0;				
		}
}

void destroy_out_msg_queue(){
//fprintf(stdout,"out queue freed\n");
delete_p(updater->outmsg_queue->queue);
delete_p(updater->outmsg_queue->size);
delete_p(updater->outmsg_queue);
updater_msg_ready=0;
}

int get_update_mode(){
return (int)*(int*)(updater->mode);
}

int is_msg_present(){
return updater_msg_ready;
}
void destroy_updater(){
delete_p(updater->bcode);
delete_p(updater->bcode_size);
delete_p(updater->standby_bcode);
delete_p(updater->standby_bcode_size);
delete_p(updater->mode);
delete_p(updater->update_no);
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


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
static struct timeval t1, t2;
static int timer_steps=0;
//static clock_t end;
void collect_data();
/*Temp end */
static int fd,wd =0;
static int old_update =0;
static buzz_updater_elem_t updater;
static int no_of_robot;
static char* dbgf_name;
static const char* bzz_file;
static int neigh=-1;
static int 	    updater_msg_ready ;
static int updated=0;

/*Initialize updater*/
void init_update_monitor(const char* bo_filename, const char* stand_by_script){
	ROS_INFO("intiialized file monitor.\n");
	fd=inotify_init1(IN_NONBLOCK);
	if ( fd < 0 ) {
		perror( "inotify_init error" );
	}
	/* watch /.bzz file for any activity and report it back to update */
	wd=inotify_add_watch(fd, bzz_file,IN_ALL_EVENTS );
	/*load the .bo under execution into the updater*/
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
		//fclose(fp);
	//return 0;
	}
	fclose(fp);
	/*Load stand_by .bo file into the updater*/
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
	//fclose(fp);
	//return 0;
	}
	fclose(fp);
	/*Create the updater*/	
	updater = (buzz_updater_elem_t)malloc(sizeof(struct buzz_updater_elem_s));
	/*Intialize the updater with the required data*/
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
	//no_of_robot=barrier;
	updater_msg_ready=0;
	//neigh = 0;
	//updater->outmsg_queue=
	// update_table->barrier=nvs;

}
/*Check for .bzz file chages*/
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
	//ROS_INFO("[DEBUG] Updater append code of size %d\n", (int) size);
	updater->inmsg_queue->queue = (uint8_t*)malloc(size);
	updater->inmsg_queue->size  = (uint8_t*)malloc(sizeof(uint16_t));
	memcpy(updater->inmsg_queue->queue, msg, size);
	*(uint16_t*)(updater->inmsg_queue->size)  = size;
}

void code_message_inqueue_process(){
	int size=0;
	ROS_INFO("[Debug] Updater processing in msg with mode %d \n", *(int*)(updater->mode) );
	ROS_INFO("[Debug] %u : Current update number, %u : Received update no \n",( *(uint16_t*) (updater->update_no) ), (*(uint16_t*)(updater->inmsg_queue->queue)) );
	ROS_INFO("[Debug] Updater received code of size %u \n",(*(uint16_t*)(updater->inmsg_queue->queue+sizeof(uint16_t)) ) );

	if(  *(int*) (updater->mode) == CODE_RUNNING){		
		//fprintf(stdout,"[debug]Inside inmsg code running");
		if( *(uint16_t*)(updater->inmsg_queue->queue) >  *(uint16_t*) (updater->update_no)  ){
			//fprintf(stdout,"[debug]Inside update number comparision");
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

			
			ROS_INFO("Update found \nUpdating script ...\n");
			
			if(compile_bzz()){ 
				ROS_WARN("Errors in comipilg script so staying on old script\n");
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
					ROS_INFO("Current Update no %d\n", *(uint16_t*)(updater->update_no));
					buzzvm_pushs(VM, buzzvm_string_register(VM, "update_no", 1));
					buzzvm_pushi(VM, *(uint16_t*)(updater->update_no));
					buzzvm_gstore(VM);
					neigh=-1;
					ROS_INFO("Sending code... \n");		
					code_message_outqueue_append();
				}
				delete_p(BO_BUF);
		       	}
	     
		}
		
	}

	else{
		//gettimeofday(&t1, NULL);
		if(neigh==0 && (!is_msg_present())){ 
			ROS_INFO("Sending code... \n");		
			code_message_outqueue_append();
		
		}	
		timer_steps++;
		buzzvm_pushs(VM, buzzvm_string_register(VM, "barrier_val",1));
            	buzzvm_gload(VM);
            	buzzobj_t tObj = buzzvm_stack_at(VM, 1);
            	buzzvm_pop(VM);
		ROS_INFO("Barrier ..................... %i \n",tObj->i.value);
		if(tObj->i.value==no_of_robot) { 
			*(int*)(updater->mode) = CODE_RUNNING;
			gettimeofday(&t2, NULL);
			//collect_data();
			timer_steps=0;
			neigh=0;
			buzz_utility::buzz_update_set((updater)->bcode, (char*)dbgfname, *(size_t*)(updater->bcode_size));
			//buzzvm_function_call(m_tBuzzVM, "updated", 0);
			updated=1;	
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
		ROS_WARN("Initializtion of script test passed\n");
		if(buzz_utility::update_step_test()){
			/*data logging*/
			//start =1;
			/*data logging*/
			ROS_WARN("Step test passed\n");
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
			ROS_ERROR("step test failed, stick to old script\n");
			buzz_utility::buzz_update_init_test((updater)->bcode, dbgfname, (size_t)*(size_t*)(updater->bcode_size));
			}
			else{
			/*You will never reach here*/
			ROS_ERROR("step test failed, Return to stand by\n");
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
		ROS_ERROR("Initialization test failed, stick to old script\n");
		buzz_utility::buzz_update_init_test((updater)->bcode, dbgfname,(int)*(size_t*) (updater->bcode_size));
		}
		else{
		/*You will never reach here*/
		ROS_ERROR("Initialization test failed, Return to stand by\n");
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
int get_update_status(){
return updated;
}
void set_read_update_status(){
	updated=0;
}
int get_update_mode(){
return (int)*(int*)(updater->mode);
}

int is_msg_present(){
return updater_msg_ready;
}
buzz_updater_elem_t get_updater(){
return updater;
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

void updates_set_robots(int robots){
	no_of_robot=robots;
}

/*--------------------------------------------------------
/ Create Buzz bytecode from the bzz script inputed
/-------------------------------------------------------*/
int compile_bzz(){
	/*Compile the buzz code .bzz to .bo*/
	std::string bzzfile_name(bzz_file);
	stringstream bzzfile_in_compile;
	std::string  path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/")) + "/";
	std::string  name = bzzfile_name.substr(bzzfile_name.find_last_of("/\\") + 1);
	name = name.substr(0,name.find_last_of("."));
	bzzfile_in_compile << "bzzc -I " << path << "include/"; //<<" "<<path<< name<<".basm";
	bzzfile_in_compile << " -b " << path << name << ".bo";
	bzzfile_in_compile << " -d " << path << name << ".bdb ";
	bzzfile_in_compile << bzzfile_name;
	ROS_WARN("Launching buzz compilation for update: %s", bzzfile_in_compile.str().c_str());
	return system(bzzfile_in_compile.str().c_str());
}
void collect_data(){
	//fprintf(stdout,"start and end time in data collection Info : %f,%f",(double)begin,(double)end);
	double time_spent =   (t2.tv_sec - t1.tv_sec) * 1000.0; //(double)(end - begin) / CLOCKS_PER_SEC;
	time_spent += (t2.tv_usec - t1.tv_usec) / 1000.0;
	//int bytecodesize=(int);
	fprintf(stdout,"Data logger Info : %i , %i , %f , %ld , %i , %d \n",(int)no_of_robot,neigh,time_spent,*(size_t*)updater->bcode_size,(int)no_of_robot,*(uint8_t*)updater->update_no);
	//FILE *Fileptr;
	//Fileptr=fopen("/home/ubuntu/ROS_WS/update_logger.csv", "a");
	//fprintf(Fileptr,"%i,%i,%i,%i,%i,%u\n",(int)buzz_utility::get_robotid(),neigh,timer_steps,(int)*(size_t*)updater->bcode_size,(int)no_of_robot, *(uint8_t*)updater->update_no);
	//fclose(Fileptr);
}


/** @file      buzz_utility.cpp
 *  @version   1.0
 *  @date      27.09.2016
 *  @brief     Buzz Implementation as a node in ROS.
 *  @author    Vivek Shankar Varadharajan and David St-Onge
 *  @copyright 2016 MistLab. All rights reserved.
 */

#include <rosbuzz/buzz_update.h>

namespace buzz_update
{
/*Temp for data collection*/
// static int neigh=-1;
static struct timeval t1, t2;
static int timer_steps = 0;
// static clock_t end;

/*Temp end */
static int fd, wd = 0;
static int old_update = 0;
static buzz_updater_elem_t updater;
static int no_of_robot;
static const char* dbgf_name;
static const char* bcfname;
static const char* old_bcfname = "old_bcode.bo";
static const char* bzz_file;
static int Robot_id = 0;
static int neigh = -1;
static int updater_msg_ready;
static uint16_t update_try_counter = 0;
static const uint16_t MAX_UPDATE_TRY = 5;
static size_t old_byte_code_size = 0;
static bool debug = false;

/*Initialize updater*/
int init_update_monitor(const char* bo_filename, const char* stand_by_script, const char* dbgfname, int robot_id)
{
  buzzvm_t VM = buzz_utility::get_vm();
  buzzvm_pushs(VM, buzzvm_string_register(VM, "updates_active", 1));
  buzzvm_gload(VM);
  buzzobj_t obj = buzzvm_stack_at(VM, 1);
  if (obj->o.type == BUZZTYPE_INT && obj->i.value == 1)
  {
    Robot_id = robot_id;
    dbgf_name = dbgfname;
    bcfname = bo_filename;
    ROS_WARN("UPDATES TURNED ON (modifying .bzz script file will update all robots)");
    if (debug)
      ROS_INFO("Initializing file monitor...");
    fd = inotify_init1(IN_NONBLOCK);
    if (fd < 0)
    {
      perror("inotify_init error");
    }
    /*If simulation set the file monitor only for robot 0*/
    if (SIMULATION == 1 && robot_id == 0)
    {
      /* watch /.bzz file for any activity and report it back to update */
      wd = inotify_add_watch(fd, bzz_file, IN_ALL_EVENTS);
    }
    else if (SIMULATION == 0)
    {
      /* watch /.bzz file for any activity and report it back to update */
      wd = inotify_add_watch(fd, bzz_file, IN_ALL_EVENTS);
    }
    /*load the .bo under execution into the updater*/
    uint8_t* BO_BUF = 0;
    FILE* fp = fopen(bo_filename, "rb");
    if (!fp)
    {
      perror(bo_filename);
    }
    fseek(fp, 0, SEEK_END);
    size_t bcode_size = ftell(fp);
    rewind(fp);
    BO_BUF = (uint8_t*)malloc(bcode_size);
    if (fread(BO_BUF, 1, bcode_size, fp) < bcode_size)
    {
      perror(bo_filename);
      // fclose(fp);
      // return 0;
    }
    fclose(fp);
    /*Load stand_by .bo file into the updater*/
    uint8_t* STD_BO_BUF = 0;
    fp = fopen(stand_by_script, "rb");
    if (!fp)
    {
      perror(stand_by_script);
    }
    fseek(fp, 0, SEEK_END);
    size_t stdby_bcode_size = ftell(fp);
    rewind(fp);
    STD_BO_BUF = (uint8_t*)malloc(stdby_bcode_size);
    if (fread(STD_BO_BUF, 1, stdby_bcode_size, fp) < stdby_bcode_size)
    {
      perror(stand_by_script);
      // fclose(fp);
      // return 0;
    }
    fclose(fp);
    /*Create the updater*/
    updater = (buzz_updater_elem_t)malloc(sizeof(struct buzz_updater_elem_s));
    /*Intialize the updater with the required data*/
    updater->bcode = BO_BUF;
    /*Store a copy of the Bcode for rollback*/
    updater->outmsg_queue = NULL;
    updater->inmsg_queue = NULL;
    updater->patch = NULL;
    updater->patch_size = (size_t*)malloc(sizeof(size_t));
    updater->bcode_size = (size_t*)malloc(sizeof(size_t));
    updater->update_no = (uint8_t*)malloc(sizeof(uint16_t));
    *(uint16_t*)(updater->update_no) = 0;
    *(size_t*)(updater->bcode_size) = bcode_size;
    updater->standby_bcode = STD_BO_BUF;
    updater->standby_bcode_size = (size_t*)malloc(sizeof(size_t));
    *(size_t*)(updater->standby_bcode_size) = stdby_bcode_size;
    updater->mode = (int*)malloc(sizeof(int));
    *(int*)(updater->mode) = CODE_RUNNING;
    // no_of_robot=barrier;
    updater_msg_ready = 0;

    /*Write the bcode to a file for rollback*/
    fp = fopen("old_bcode.bo", "wb");
    fwrite((updater->bcode), *(size_t*)updater->bcode_size, 1, fp);
    fclose(fp);
    return 1;
  }
  else
  {
    ROS_WARN("UPDATES TURNED OFF... (Hint: Include update.bzz to turn on updates)");
    return 0;
  }
}
/*Check for .bzz file chages*/
int check_update()
{
  char buf[1024];
  int check = 0;
  int i = 0;
  int len = read(fd, buf, 1024);
  while (i < len)
  {
    struct inotify_event* event = (struct inotify_event*)&buf[i];
    /*Report file changes and self deletes*/
    if (event->mask & (IN_MODIFY | IN_DELETE_SELF))
    {
      /*Respawn watch if the file is self deleted */
      inotify_rm_watch(fd, wd);
      close(fd);
      fd = inotify_init1(IN_NONBLOCK);
      wd = inotify_add_watch(fd, bzz_file, IN_ALL_EVENTS);
      /*To mask multiple writes from editors*/
      if (!old_update)
      {
        check = 1;
        old_update = 1;
      }
    }
    /*Update index to start of next event*/
    i += sizeof(struct inotify_event) + event->len;
  }
  if (!check)
    old_update = 0;
  return check;
}

int test_patch(std::string& path, std::string& name1, size_t update_patch_size, uint8_t* patch)
{
  if (SIMULATION == 1)
  {
    return 1;
  }
  else
  {
    /*Patch the old bo with new patch*/
    std::stringstream patch_writefile;
    patch_writefile << path << name1 << "tmp_patch.bo";
    /*Write the patch to a file and call bsdiff to patch*/
    FILE* fp = fopen(patch_writefile.str().c_str(), "wb");
    fwrite(patch, update_patch_size, 1, fp);
    fclose(fp);
    std::stringstream patch_exsisting;
    patch_exsisting << "bspatch " << path << name1 << ".bo " << path << name1 << "-patched.bo " << path << name1
                    << "tmp_patch.bo";
    if (debug)
      ROS_WARN("Launching bspatch command: %s", patch_exsisting.str().c_str());
    if (system(patch_exsisting.str().c_str()))
      return 0;
    else
      return 1;
  }
}

updater_code_t obtain_patched_bo(std::string& path, std::string& name1)
{
  if (SIMULATION == 1)
  {
    /*Read the exsisting file to simulate the patching*/
    std::stringstream read_patched;
    read_patched << path << name1 << ".bo";
    if (debug)
    {
      ROS_WARN("Simulation patching ...");
      ROS_WARN("Read file for patching %s", read_patched.str().c_str());
    }
    uint8_t* patched_bo_buf = 0;
    FILE* fp = fopen(read_patched.str().c_str(), "rb");
    if (!fp)
    {
      perror(read_patched.str().c_str());
    }
    fseek(fp, 0, SEEK_END);
    size_t patched_size = ftell(fp);
    rewind(fp);
    patched_bo_buf = (uint8_t*)malloc(patched_size);
    if (fread(patched_bo_buf, 1, patched_size, fp) < patched_size)
    {
      perror(read_patched.str().c_str());
    }
    fclose(fp);
    /*Write the patched to a code struct and return*/
    updater_code_t update_code = (updater_code_t)malloc(sizeof(struct updater_code_s));
    update_code->bcode = patched_bo_buf;
    update_code->bcode_size = (uint8_t*)malloc(sizeof(uint16_t));
    *(uint16_t*)(update_code->bcode_size) = patched_size;

    return update_code;
  }

  else
  {
    /*Read the new patched file*/
    std::stringstream read_patched;
    read_patched << path << name1 << "-patched.bo";
    if (debug)
    {
      ROS_WARN("Robot patching ...");
      ROS_WARN("Read file for patching %s", read_patched.str().c_str());
    }
    uint8_t* patched_bo_buf = 0;
    FILE* fp = fopen(read_patched.str().c_str(), "rb");
    if (!fp)
    {
      perror(read_patched.str().c_str());
    }
    fseek(fp, 0, SEEK_END);
    size_t patched_size = ftell(fp);
    rewind(fp);
    patched_bo_buf = (uint8_t*)malloc(patched_size);
    if (fread(patched_bo_buf, 1, patched_size, fp) < patched_size)
    {
      perror(read_patched.str().c_str());
    }
    fclose(fp);

    /* delete old bo file & rename new bo file */
    remove((path + name1 + ".bo").c_str());
    rename((path + name1 + "-patched.bo").c_str(), (path + name1 + ".bo").c_str());

    /*Write the patched to a code struct and return*/
    updater_code_t update_code = (updater_code_t)malloc(sizeof(struct updater_code_s));
    update_code->bcode = patched_bo_buf;
    update_code->bcode_size = (uint8_t*)malloc(sizeof(uint16_t));
    *(uint16_t*)(update_code->bcode_size) = patched_size;

    return update_code;
  }
}

void code_message_outqueue_append()
{
  updater->outmsg_queue = (updater_msgqueue_t)malloc(sizeof(struct updater_msgqueue_s));
  /* if size less than 250 Pad with zeors to assure transmission*/
  uint16_t size = UPDATE_CODE_HEADER_SIZE + *(size_t*)(updater->patch_size);
  uint16_t padding_size = (size > MIN_UPDATE_PACKET) ? 0 : MIN_UPDATE_PACKET - size;
  size += padding_size;
  updater->outmsg_queue->queue = (uint8_t*)malloc(size);
  memset(updater->outmsg_queue->queue, 0, size);
  updater->outmsg_queue->size = (uint8_t*)malloc(sizeof(uint16_t));
  *(uint16_t*)(updater->outmsg_queue->size) = size;
  size = 0;
  /*Append message type*/
  *(uint8_t*)(updater->outmsg_queue->queue + size) = SENT_CODE;
  size += sizeof(uint8_t);
  /*Append the update no, code size and code to out msg*/
  *(uint16_t*)(updater->outmsg_queue->queue + size) = *(uint16_t*)(updater->update_no);
  size += sizeof(uint16_t);
  *(uint16_t*)(updater->outmsg_queue->queue + size) = (uint16_t) * (size_t*)(updater->patch_size);
  size += sizeof(uint16_t);
  memcpy(updater->outmsg_queue->queue + size, updater->patch, *(size_t*)(updater->patch_size));
  // size += (uint16_t) * (size_t*)(updater->patch_size);
  updater_msg_ready = 1;
}

void outqueue_append_code_request(uint16_t update_no)
{
  updater->outmsg_queue = (updater_msgqueue_t)malloc(sizeof(struct updater_msgqueue_s));
  uint16_t size = 0;
  updater->outmsg_queue->queue = (uint8_t*)malloc(2 * sizeof(uint16_t) + sizeof(uint8_t) + CODE_REQUEST_PADDING);
  updater->outmsg_queue->size = (uint8_t*)malloc(sizeof(uint16_t));
  memset(updater->outmsg_queue->queue, 0, sizeof(uint16_t) + sizeof(uint8_t) + CODE_REQUEST_PADDING);
  /*Append message type*/
  *(uint8_t*)(updater->outmsg_queue->queue + size) = RESEND_CODE;
  size += sizeof(uint8_t);
  /*Append the update no, code size and code to out msg*/
  *(uint16_t*)(updater->outmsg_queue->queue + size) = update_no;
  size += sizeof(uint16_t);
  *(uint16_t*)(updater->outmsg_queue->queue + size) = update_try_counter;
  size += sizeof(uint16_t);
  *(uint16_t*)(updater->outmsg_queue->size) = size + CODE_REQUEST_PADDING;
  updater_msg_ready = 1;
  if (debug)
    ROS_WARN("[Debug] Requesting update no. %u for rebroadcast, try: %u", update_no, update_try_counter);
}

void code_message_inqueue_append(uint8_t* msg, uint16_t size)
{
  updater->inmsg_queue = (updater_msgqueue_t)malloc(sizeof(struct updater_msgqueue_s));
  // ROS_INFO("[DEBUG] Updater append code of size %d", (int) size);
  updater->inmsg_queue->queue = (uint8_t*)malloc(size);
  updater->inmsg_queue->size = (uint8_t*)malloc(sizeof(uint16_t));
  memcpy(updater->inmsg_queue->queue, msg, size);
  *(uint16_t*)(updater->inmsg_queue->size) = size;
}
/*Used for data logging*/
/*void set_packet_id(int packet_id)
{
  packet_id_ = packet_id;
}*/
void code_message_inqueue_process()
{
  int size = 0;
  updater_code_t out_code = NULL;
  if (debug)
  {
    ROS_WARN("[Debug] Updater processing in msg with mode %d", *(int*)(updater->mode));
    ROS_WARN("[Debug] Current update no: %u, Received update no: %u", (*(uint16_t*)(updater->update_no)),
             (*(uint16_t*)(updater->inmsg_queue->queue + sizeof(uint8_t))));
    ROS_WARN("[Debug] Updater received patch of size %u",
             (*(uint16_t*)(updater->inmsg_queue->queue + sizeof(uint16_t) + sizeof(uint8_t))));
  }
  if (*(int*)(updater->mode) == CODE_RUNNING)
  {
    // fprintf(stdout,"[debug]Inside inmsg code running");
    if (*(uint8_t*)(updater->inmsg_queue->queue) == SENT_CODE)
    {
      size += sizeof(uint8_t);
      if (*(uint16_t*)(updater->inmsg_queue->queue + size) > *(uint16_t*)(updater->update_no))
      {
        // fprintf(stdout,"[debug]Inside update number comparision");
        uint16_t update_no = *(uint16_t*)(updater->inmsg_queue->queue + size);
        size += sizeof(uint16_t);
        uint16_t update_patch_size = *(uint16_t*)(updater->inmsg_queue->queue + size);
        size += sizeof(uint16_t);
        /*Generate patch*/
        std::string bzzfile_name(bzz_file);
        std::string path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/")) + "/";
        std::string name1 = bzzfile_name.substr(bzzfile_name.find_last_of("/\\") + 1);
        name1 = name1.substr(0, name1.find_last_of("."));
        if (test_patch(path, name1, update_patch_size, (updater->inmsg_queue->queue + size)))
        {
          out_code = obtain_patched_bo(path, name1);

          // fprintf(stdout,"in queue process Update no %d\n", (int) update_no);
          // fprintf(stdout,"in queue process bcode size %d\n", (int) update_bcode_size);
          // FILE *fp;
          // fp=fopen("update.bo", "wb");
          // fwrite((updater->inmsg_queue->queue+size), update_bcode_size, 1, fp);
          // fclose(fp);

          if (test_set_code((out_code->bcode), (char*)dbgf_name, (size_t)(*(uint16_t*)out_code->bcode_size)))
          {
            if (debug)
              ROS_WARN("Received patch PASSED tests!");
            *(uint16_t*)updater->update_no = update_no;
            /*Clear exisiting patch if any*/
            delete_p(updater->patch);
            /*copy the patch into the updater*/
            updater->patch = (uint8_t*)malloc(update_patch_size);
            memcpy(updater->patch, (updater->inmsg_queue->queue + size), update_patch_size);
            *(size_t*)(updater->patch_size) = update_patch_size;
            // code_message_outqueue_append();
            neigh = 1;
          }
          /*clear the temp code buff*/
          delete_p(out_code->bcode);
          delete_p(out_code->bcode_size);
          delete_p(out_code);
        }
        else
        {
          ROS_ERROR("Patching the .bo file failed");
          update_try_counter++;
          outqueue_append_code_request(update_no);
        }
      }
    }
  }
  size = 0;
  if (*(uint8_t*)(updater->inmsg_queue->queue) == RESEND_CODE)
  {
    if (debug)
      ROS_WARN("Patch rebroadcast request received.");
    size += sizeof(uint8_t);
    if (*(uint16_t*)(updater->inmsg_queue->queue + size) == *(uint16_t*)(updater->update_no))
    {
      size += sizeof(uint16_t);
      if (*(uint16_t*)(updater->inmsg_queue->queue + size) > update_try_counter)
      {
        update_try_counter = *(uint16_t*)(updater->inmsg_queue->queue + size);
        if (debug)
          ROS_WARN("Rebroadcasting patch, try : %u", update_try_counter);
        code_message_outqueue_append();
      }
      if (update_try_counter > MAX_UPDATE_TRY)
        ROS_ERROR("TODO: Max rebroadcast retry reached, ROLL BACK !!");
    }
  }
  // fprintf(stdout,"in queue freed\n");
  delete_p(updater->inmsg_queue->queue);
  delete_p(updater->inmsg_queue->size);
  delete_p(updater->inmsg_queue);
}

void create_update_patch()
{
  std::stringstream genpatch;

  std::string bzzfile_name(bzz_file);

  std::string path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/")) + "/";

  std::string name1 = bzzfile_name.substr(bzzfile_name.find_last_of("/\\") + 1);
  name1 = name1.substr(0, name1.find_last_of("."));

  std::string name2 = name1 + "-update";

  /*Launch bsdiff and create a patch*/
  genpatch << "bsdiff " << path << name1 << ".bo " << path << name2 << ".bo " << path << "diff.bo";
  if (debug)
    ROS_WARN("Launching bsdiff: %s", genpatch.str().c_str());
  system(genpatch.str().c_str());

  /*Delete old files & rename new files*/

  remove((path + name1 + ".bo").c_str());
  remove((path + name1 + ".bdb").c_str());

  rename((path + name2 + ".bo").c_str(), (path + name1 + ".bo").c_str());
  rename((path + name2 + ".bdb").c_str(), (path + name1 + ".bdb").c_str());

  /*Read the diff file */
  std::stringstream patchfileName;
  patchfileName << path << "diff.bo";

  uint8_t* patch_buff = 0;
  FILE* fp = fopen(patchfileName.str().c_str(), "rb");
  if (!fp)
  {
    perror(patchfileName.str().c_str());
  }
  fseek(fp, 0, SEEK_END);
  size_t patch_size = ftell(fp);
  rewind(fp);
  patch_buff = (uint8_t*)malloc(patch_size);
  if (fread(patch_buff, 1, patch_size, fp) < patch_size)
  {
    perror(patchfileName.str().c_str());
  }
  fclose(fp);
  delete_p(updater->patch);
  updater->patch = patch_buff;
  *(size_t*)(updater->patch_size) = patch_size;

  /* Delete the diff file */
  remove(patchfileName.str().c_str());
}

void update_routine()
{
  buzzvm_t VM = buzz_utility::get_vm();
  buzzvm_pushs(VM, buzzvm_string_register(VM, "update_no", 1));
  buzzvm_pushi(VM, *(uint16_t*)(updater->update_no));
  buzzvm_gstore(VM);
  // fprintf(stdout,"[Debug : ]updater value = %i \n",updater->mode);
  if (*(int*)updater->mode == CODE_RUNNING)
  {
    buzzvm_function_call(VM, "updated_no_bct", 0);
    buzzvm_pop(VM); // Pop return value
    // Check for update
    if (check_update())
    {
      ROS_INFO("Update found \tUpdating script ...");

      if (compile_bzz(bzz_file))
      {
        ROS_ERROR("Error in compiling script, resuming old script.");
      }
      else
      {
        std::string bzzfile_name(bzz_file);
        stringstream bzzfile_in_compile;
        std::string path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/")) + "/";
        std::string name = bzzfile_name.substr(bzzfile_name.find_last_of("/\\") + 1);
        name = name.substr(0, name.find_last_of("."));
        bzzfile_in_compile << path << name << "-update.bo";
        uint8_t* BO_BUF = 0;
        FILE* fp = fopen(bzzfile_in_compile.str().c_str(), "rb");
        if (!fp)
        {
          perror(bzzfile_in_compile.str().c_str());
        }
        fseek(fp, 0, SEEK_END);
        size_t bcode_size = ftell(fp);
        rewind(fp);
        BO_BUF = (uint8_t*)malloc(bcode_size);
        if (fread(BO_BUF, 1, bcode_size, fp) < bcode_size)
        {
          perror(bcfname);
        }
        fclose(fp);
        if (test_set_code(BO_BUF, dbgf_name, bcode_size))
        {
          uint16_t update_no = *(uint16_t*)(updater->update_no);
          *(uint16_t*)(updater->update_no) = update_no + 1;

          create_update_patch();
          VM = buzz_utility::get_vm();
          if (debug)
            ROS_INFO("Current Update no %d", *(uint16_t*)(updater->update_no));
          buzzvm_pushs(VM, buzzvm_string_register(VM, "update_no", 1));
          buzzvm_pushi(VM, *(uint16_t*)(updater->update_no));
          buzzvm_gstore(VM);
          neigh = -1;
          if (debug)
            ROS_INFO("Broadcasting patch ...");
          code_message_outqueue_append();
        }
        delete_p(BO_BUF);
      }
    }
  }

  else
  {
    timer_steps++;
    buzzvm_pushs(VM, buzzvm_string_register(VM, "barrier_val", 1));
    buzzvm_gload(VM);
    buzzobj_t tObj = buzzvm_stack_at(VM, 1);
    buzzvm_pop(VM);
    ROS_INFO("Barrier ............. No. of robots deployed: %i", tObj->i.value);
    if (tObj->i.value == no_of_robot)
    {
      ROS_WARN("Patch deployment successful, rolling forward");
      *(int*)(updater->mode) = CODE_RUNNING;
      gettimeofday(&t2, NULL);
      // collect_data();
      buzz_utility::buzz_update_set((updater)->bcode, (char*)dbgf_name, *(size_t*)(updater->bcode_size));
      // buzzvm_function_call(m_tBuzzVM, "updated", 0);
      update_try_counter = 0;
      timer_steps = 0;
    }
    else if (timer_steps > TIMEOUT_FOR_ROLLBACK)
    {
      ROS_ERROR("TIME OUT Reached, rolling back");
      /*Time out hit deceided to roll back*/
      *(int*)(updater->mode) = CODE_RUNNING;
      buzz_utility::buzz_script_set(old_bcfname, dbgf_name, (int)VM->robot);
      update_try_counter = 0;
      timer_steps = 0;
    }
  }
}

uint8_t* getupdater_out_msg()
{
  return (uint8_t*)updater->outmsg_queue->queue;
}

uint8_t* getupdate_out_msg_size()
{
  // fprintf(stdout,"[Debug  from get out size in util: ]size = %i \n",*(uint16_t*)updater->outmsg_queue->size);
  return (uint8_t*)updater->outmsg_queue->size;
}

int test_set_code(uint8_t* BO_BUF, const char* dbgfname, size_t bcode_size)
{
  if (buzz_utility::buzz_update_init_test(BO_BUF, dbgfname, bcode_size))
  {
    if (debug)
      ROS_WARN("Initializtion test passed");
    if (buzz_utility::update_step_test())
    {
      /*data logging*/
      old_byte_code_size = *(size_t*)updater->bcode_size;
      /*data logging*/
      if (debug)
        ROS_WARN("Step test passed");
      *(int*)(updater->mode) = CODE_STANDBY;
      // fprintf(stdout,"updater value = %i\n",updater->mode);
      delete_p(updater->bcode);
      updater->bcode = (uint8_t*)malloc(bcode_size);
      memcpy(updater->bcode, BO_BUF, bcode_size);
      *(size_t*)updater->bcode_size = bcode_size;
      buzz_utility::buzz_update_init_test((updater)->standby_bcode, (char*)dbgfname,
                                          (size_t) * (size_t*)(updater->standby_bcode_size));
      buzzvm_t VM = buzz_utility::get_vm();
      gettimeofday(&t1, NULL);
      buzzvm_pushs(VM, buzzvm_string_register(VM, "ROBOTS", 1));
      buzzvm_pushi(VM, no_of_robot);
      buzzvm_gstore(VM);
      return 1;
    }
    /*Unable to step something wrong*/
    else
    {
      if (*(int*)(updater->mode) == CODE_RUNNING)
      {
        ROS_ERROR("Step test failed, resuming old script");
        buzz_utility::buzz_update_init_test((updater)->bcode, dbgfname, (size_t) * (size_t*)(updater->bcode_size));
      }
      else
      {
        /*You will never reach here*/
        ROS_ERROR("Step test failed, returning to standby");
        buzz_utility::buzz_update_init_test((updater)->standby_bcode, (char*)dbgfname,
                                            (size_t) * (size_t*)(updater->standby_bcode_size));
        buzzvm_t VM = buzz_utility::get_vm();
        buzzvm_pushs(VM, buzzvm_string_register(VM, "ROBOTS", 1));
        buzzvm_pushi(VM, no_of_robot);
        buzzvm_gstore(VM);
      }
      return 0;
    }
  }
  else
  {
    if (*(int*)(updater->mode) == CODE_RUNNING)
    {
      ROS_ERROR("Initialization test failed, resuming old script");
      buzz_utility::buzz_update_init_test((updater)->bcode, dbgfname, (int)*(size_t*)(updater->bcode_size));
    }
    else
    {
      /*You will never reach here*/
      ROS_ERROR("Initialization test failed, returning to standby");
      buzz_utility::buzz_update_init_test((updater)->standby_bcode, (char*)dbgfname,
                                          (size_t) * (size_t*)(updater->standby_bcode_size));
      buzzvm_t VM = buzz_utility::get_vm();
      buzzvm_pushs(VM, buzzvm_string_register(VM, "ROBOTS", 1));
      buzzvm_pushi(VM, no_of_robot);
      buzzvm_gstore(VM);
    }
    return 0;
  }
}

void destroy_out_msg_queue()
{
  // fprintf(stdout,"out queue freed\n");
  delete_p(updater->outmsg_queue->queue);
  delete_p(updater->outmsg_queue->size);
  delete_p(updater->outmsg_queue);
  updater_msg_ready = 0;
}

int is_msg_present()
{
  return updater_msg_ready;
}
/*buzz_updater_elem_t get_updater()
{
  return updater;
}*/
void destroy_updater()
{
  delete_p(updater->bcode);
  delete_p(updater->bcode_size);
  delete_p(updater->standby_bcode);
  delete_p(updater->standby_bcode_size);
  delete_p(updater->mode);
  delete_p(updater->update_no);
  if (updater->outmsg_queue)
  {
    delete_p(updater->outmsg_queue->queue);
    delete_p(updater->outmsg_queue->size);
    delete_p(updater->outmsg_queue);
  }
  if (updater->inmsg_queue)
  {
    delete_p(updater->inmsg_queue->queue);
    delete_p(updater->inmsg_queue->size);
    delete_p(updater->inmsg_queue);
  }
  inotify_rm_watch(fd, wd);
  close(fd);
}

void set_bzz_file(const char* in_bzz_file, bool dbg)
{
  debug = dbg;
  bzz_file = in_bzz_file;
}

void updates_set_robots(int robots)
{
  no_of_robot = robots;
}

/*--------------------------------------------------------
/ Create Buzz bytecode from the bzz script input
/-------------------------------------------------------*/
int compile_bzz(std::string bzz_file)
{
  /*Compile the buzz code .bzz to .bo*/
  std::string bzzfile_name(bzz_file);
  stringstream bzzfile_in_compile;
  std::string path = bzzfile_name.substr(0, bzzfile_name.find_last_of("\\/")) + "/";
  std::string name = bzzfile_name.substr(bzzfile_name.find_last_of("/\\") + 1);
  name = name.substr(0, name.find_last_of("."));
  bzzfile_in_compile << "bzzc -I " << path << "include/";  //<<" "<<path<< name<<".basm";
  bzzfile_in_compile << " -b " << path << name << "-update.bo";
  bzzfile_in_compile << " -d " << path << name << "-update.bdb ";
  bzzfile_in_compile << bzzfile_name;
  ROS_WARN("Launching buzz compilation for update: %s", bzzfile_in_compile.str().c_str());
  return system(bzzfile_in_compile.str().c_str());
}

/*void collect_data(std::ofstream& logger)
{
  double time_spent = (t2.tv_sec - t1.tv_sec) * 1000.0;  //(double)(end - begin) / CLOCKS_PER_SEC;
  time_spent += (t2.tv_usec - t1.tv_usec) / 1000.0;
  // RID,update trigger,time steps taken,old byte code size, new bytecode size, patch size,update number,
  //
Patch_packets_received_counter,Patch_request_packets_received,Patch_packets_sent_counter,Patch_request_packets_sent_counter
  logger << (int)no_of_robot << "," << neigh << "," << (double)time_spent << "," << (int)timer_steps << ","
         << old_byte_code_size << "," << *(size_t*)updater->bcode_size << "," << *(size_t*)updater->patch_size << ","
         << (int)*(uint8_t*)updater->update_no << "," << (int)packet_id_;
}*/
}

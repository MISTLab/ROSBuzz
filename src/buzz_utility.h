#ifndef BUZZ_UTILITY_H
#define BUZZ_UTILITY_H

extern int buzz_listen(const char* type,
                       int msg_size);
void in_msg_process(unsigned long int payload[], double negighbour[]);

unsigned long int* out_msg_process();

extern int buzz_script_set(const char* bo_filename,
                           const char* bdbg_filename);
extern void buzz_script_step();

extern void buzz_script_destroy();

extern int buzz_script_done();

#endif

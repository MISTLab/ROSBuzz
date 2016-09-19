#ifndef BUZZ_UTILITY_H
#define BUZZ_UTILITY_H

extern int buzz_listen(const char* type,
                       int msg_size);

extern int buzz_script_set(const char* bo_filename,
                           const char* bdbg_filename);
extern void neighbour_location_handler(double distance, double azimuth, double elevation, int robot_id);

extern void buzz_script_step();

extern void buzz_script_destroy();

extern int buzz_script_done();

#endif

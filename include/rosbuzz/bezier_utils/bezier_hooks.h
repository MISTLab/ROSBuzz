#ifndef MAGIC_H
#define MAGIC_H
// #include "include/VoronoiDiagramGenerator.h"
// #include "buzz/buzz_utility.c"
#ifdef __cplusplus
extern "C"
{
#endif
#include <buzz/buzzvm.h>
// #include "buzz/buzz_utility.c"
// #include "buzzkh4_closures.h"
#include <sys/time.h>
// #include "include/QPSolver.h"

int Bezier_calc(buzzvm_t vm);
int Bezier_calc_frequent(buzzvm_t vm);
int Bezier_calc_frequent_multiwise(buzzvm_t vm);
int merge_multiwise_explicit_test(buzzvm_t vm);
int Bezier_calc_smooth_junction(buzzvm_t vm);
int get_original_bezier_path(buzzvm_t vm);
int Fit_Bezier(buzzvm_t vm);
int Fit_Bezier_least_order(buzzvm_t vm);
int get_fitted_bezier_path(buzzvm_t vm);
int random_number_generator(buzzvm_t vm);

int qp_solver(buzzvm_t vm);


#ifdef __cplusplus
} // extern "C"
#endif

#endif

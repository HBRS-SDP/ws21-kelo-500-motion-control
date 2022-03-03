#ifndef PLATFORM_WHEEL_INVERSE_KINEMATICS_SOLVER_H
#define PLATFORM_WHEEL_INVERSE_KINEMATICS_SOLVER_H

#include <gsl/gsl_matrix_double.h>

void functions_main(double *wheel_torques,
                    double *pivot_angles,
                    const gsl_matrix *b,
                    gsl_matrix *b_verify,
                    gsl_matrix *A,
                    gsl_matrix *A_inv_T,
                    gsl_matrix *A_tmp,
                    gsl_matrix *A_inv_T_tmp,
                    gsl_vector *work,
                    const gsl_matrix *W,
                    const gsl_matrix *K,
                    gsl_vector *u,
                    gsl_matrix *V,
                    gsl_matrix *u_inv,
                    const unsigned int M,
                    const unsigned int N);

#endif
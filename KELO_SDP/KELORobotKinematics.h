#ifndef KELO_ROBOT_KINEMATICS_H
#define KELO_ROBOT_KINEMATICS_H

#include <gsl/gsl_matrix_double.h>

void jacobian_matrix_calculator(gsl_matrix *A,
                                const double *pivot_angles,
                                const double *wheel_coordinates);


void force_vector_finder(double *pivot_forces,
                        gsl_matrix *A,
                        gsl_matrix *A_prime_tmp,
                        gsl_matrix *A_prime_tmp_T,
                        gsl_matrix *A_prime,
                        gsl_vector *u,
                        gsl_matrix *u_inv,
                        gsl_matrix *V,
                        const gsl_matrix *W,
                        const gsl_matrix *K,
                        gsl_vector *work,
                        const gsl_matrix *b,
                        const unsigned int M);

#endif
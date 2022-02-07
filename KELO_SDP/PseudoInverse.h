#ifndef PSEUDO_INVERSE_H
#define PSEUDO_INVERSE_H

#include <gsl/gsl_matrix_double.h>

void force_vector_finder(double *pivot_forces,
                        gsl_matrix *A,
                        gsl_matrix *A_prime_tmp,
                        gsl_matrix *A_prime_tmp_T,
                        gsl_matrix *A_prime,
                        gsl_vector *u,
                        gsl_matrix *u_inv,
                        gsl_matrix *V,
                        gsl_matrix *W,
                        gsl_matrix *K,
                        gsl_vector *work,
                        const gsl_matrix *b,
                        const unsigned int M);

#endif
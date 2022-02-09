#ifndef JACOBIAN_MATRIX_H
#define JACOBIAN_MATRIX_H

#include <gsl/gsl_matrix_double.h>

void jacobian_matrix_calculator(gsl_matrix *A,
                                const double *pivot_angles,
                                const double *wheel_coordinates);

#endif
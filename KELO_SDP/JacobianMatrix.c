#include <stdio.h>
#include <gsl/gsl_sf_trig.h>
#include "JacobianMatrix.h"

void jacobian_matrix_calculator(gsl_matrix *A,
                                const double *pivot_angles,
                                const double *wheel_coordinates)
{
  int i;
  for (i = 0; i < 4; ++i)
  {
    gsl_matrix_set(A, 0, 2 * i, gsl_sf_cos(pivot_angles[i]));
    gsl_matrix_set(A, 0, 2 * i + 1, gsl_sf_sin(pivot_angles[i]));
    gsl_matrix_set(A, 1, 2 * i, -gsl_sf_sin(pivot_angles[i]));
    gsl_matrix_set(A, 1, 2 * i + 1, gsl_sf_cos(pivot_angles[i]));
    gsl_matrix_set(A, 2, 2 * i, -wheel_coordinates[2 * i] * gsl_sf_sin(pivot_angles[i]));
    gsl_matrix_set(A, 2, 2 * i + 1, wheel_coordinates[2 * i + 1] * gsl_sf_cos(pivot_angles[i]));
  }
}

#include "PseudoInverse.c"
#include "TorqueTransmission.c"
#include "JacobianMatrix.c"
#include "OptimisationUpdate.c"
#include <stdio.h>

void main()
{
    // 1. initialise arrays and constants
    double radius = 0.1;
    double castor_offset = 0.2;
    double half_wheel_distance = 0.1;
    double motor_const = 3.5714;                                                                    //(A/Nm)
    double wheel_coordinates[8] = {0.175, 0.1605, -0.175, 0.1605, -0.175, -0.1605, 0.175, -0.1605}; //x1,y1,x2,y2,..,y4
    double pivot_angles[4] = {1.0, 0.0, 0.0, 0.0};
    double wheel_torques[8];
    double pivot_forces[8];
    const unsigned int N = 3;
    const unsigned int M = 8;
// 2. initialise memory to matrices and vectors
    gsl_matrix *A = gsl_matrix_alloc(N, M);
    gsl_matrix *A_inv_T = gsl_matrix_alloc(M, N);
    gsl_matrix *A_tmp = gsl_matrix_alloc(N, M);
    gsl_matrix *A_inv_T_tmp = gsl_matrix_alloc(M, N);
    gsl_vector *work = gsl_vector_alloc(N);
    gsl_matrix *W = gsl_matrix_alloc(N, N); // assign values
    gsl_matrix *K = gsl_matrix_alloc(M, M); // assign values
    gsl_matrix *b = gsl_matrix_alloc(N, 1); // assign values
    gsl_vector *u = gsl_vector_alloc(N);
    gsl_matrix *V = gsl_matrix_alloc(N,N);
    gsl_matrix *u_inv = gsl_matrix_alloc(N, N);

    // substitute platform force values
    gsl_matrix_set(b, 0, 0, 0.);
    gsl_matrix_set(b, 1, 0, 10.);
    gsl_matrix_set(b, 2, 0, 0.);

    // substitute W,K values
    size_t i, j;
    for (i = 0; i < M; i++)
    {
        gsl_matrix_set(K, i, i, 1.0);
        if (i < N)
        {
            gsl_matrix_set(W, i, i, 1.0);
        }
    }

    

}
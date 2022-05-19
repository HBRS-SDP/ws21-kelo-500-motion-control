/**
 * @file PlatformToWheelInverseKinematicsSolver.c
 * @author Sivva Rahul Sai (rahul.sivva@smail.inf.h-brs.de)
 * @brief this file consists of a functionality to print a matrix and to integrate all functions related to calculating torques at individual wheel units
 * @date 2022-03-12
 *
 */
#include "SmartWheelKinematics.c"
#include "KELORobotKinematics.c"
#include <stdio.h>
#include "PlatformToWheelInverseKinematicsSolver.h"
#include <gsl/gsl_matrix_double.h>
#define _USE_MATH_DEFINES
#include <math.h>

void print_matrix(const gsl_matrix *m)
{
    size_t i, j;

    for (i = 0; i < m->size1; i++)
    {
        for (j = 0; j < m->size2; j++)
        {
            printf("%f\t", gsl_matrix_get(m, i, j));
        }
    }
}

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
                    const unsigned int N,
                    const bool debug)
{
    /**
     * @brief 1. initialise robot geometrical parameters
     * https://github.com/kelo-robotics/kelo_tulip/blob/73e6d134bd31da6c580846dc907ff1ce2565b406/src/VelocityPlatformController.cpp
     * https://github.com/kelo-robotics/kelo_tulip/blob/master/src/PlatformDriver.cpp
     *
     */

    double radius = 0.052;               // 0.105
    double castor_offset = 0.01;         // 0.01
    double half_wheel_distance = 0.0275; // 0.0775

    double wheel_coordinates[8] = {0.175, 0.1605, -0.175, 0.1605, -0.175, -0.1605, 0.175, -0.1605}; // x1,y1,x2,y2,..,y4
    double pivot_angles_deviation[4] = {-2.5, -1.25, -2.14, 1.49};                                  // https://github.com/kelo-robotics/kelo_tulip/blob/master/config/example.yaml

    /**
     * @brief updating pivot angles
     *
     */
    size_t i;
    for (i = 0; i < 4; i++)
    {
        pivot_angles[i] = pivot_angles[i] - pivot_angles_deviation[i];
        if (pivot_angles[i] > 2 * M_PI) pivot_angles[i] -= 2 * M_PI;
        else if (pivot_angles[i] < 0.0) pivot_angles[i] += 2 * M_PI;
    }

    double pivot_forces[8];

    /**
     * @brief 2. get jacobian (A) -> KELORobotKinematics.c
     *
     */
    jacobian_matrix_calculator(A,
                               pivot_angles,
                               wheel_coordinates);

    /**
     * @brief 3. find force array (pivot_forces) -> KELORobotKinematics.c
     *
     */
    force_vector_finder(pivot_forces,
                        A,
                        A_tmp,
                        A_inv_T_tmp,
                        A_inv_T,
                        u,
                        u_inv,
                        V,
                        W,
                        K,
                        work,
                        b,
                        M);

    /**
     * @brief 4. find torques at individual wheels (wheel_torques) -> SmartWheelKinematics.c
     *
     */
    map_pivot_forces_to_wheel_torques(pivot_forces,
                                      wheel_torques,
                                      radius,
                                      castor_offset,
                                      half_wheel_distance);

    /**
     * @brief 5. to print results for debugging
     *
     */
    if (debug)
    {
        /**
         * @brief printing angles after offsetting the pivots
         *
         */
        printf("\n\n\n\nPivot angles:\n");
        for (int i = 0; i < 4; i++)
        {
            printf("%f\t", pivot_angles[i]);
        }

        printf("\nPivot forces:\n");
        for (int i = 0; i < 8; i++)
        {
            printf("%f\t", pivot_forces[i]);
        }

        printf("\nWheel torques:\n");

        for (int i = 0; i < 8; i++)
        {
            printf("%f\t", wheel_torques[i]);
        }
        gsl_matrix_view _Y = gsl_matrix_view_array(pivot_forces, M, 1);
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, &_Y.matrix, 0.0, b_verify);
        printf("\nReverse calculation: Platform force: \n");
        print_matrix(b_verify);
        print_matrix(b);
    }
}

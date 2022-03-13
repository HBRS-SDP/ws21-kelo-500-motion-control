/**
 * @file PlatformToWheelInverseKinematicsSolver.h
 * @author Sivva Rahul Sai (rahul.sivva@smail.inf.h-brs.de)
 * @brief this file consists of a functionality to print a matrix and to integrate all functions related to calculating torques at individual wheel units 
 * @date 2022-03-12
 * 
 */
#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifndef PLATFORM_WHEEL_INVERSE_KINEMATICS_SOLVER_H
#define PLATFORM_WHEEL_INVERSE_KINEMATICS_SOLVER_H

#include <gsl/gsl_matrix_double.h>

/**
 * @brief function which takes pivot angles and platform forces as input and calculates the required wheel torques by suitably calling individual functinalities
 * 
 * @param wheel_torques 1-D array of shape (8,) which is a placeholder for calculated wheel torques for each wheel unit
 * @param pivot_angles 1D array of shape (4,) that stores current pivot angles of individual pair of wheels
 * @param b platform force matrix of shape 3x1
 * @param b_verify platform force matrix of shape 3x1, which is calculated from pivot forces to verify results
 * @param A Jacobian matrix of shape 3x8, which is further used to store W.dot(A).dot(K)
 * @param A_inv_T initially store left singular matrix of SVD decomposition of A and later it stores the inverse of W.dot(A).dot(K)
 * @param A_tmp a temporary matrix used to store the matrix multiplication of W.dot(A)
 * @param A_inv_T_tmp used as temporary memory to calculate A_inv_T
 * @param work temporary memory to facilitate SVD of Jacobian matrix(A)
 * @param W weight matrix of shape 3x3
 * @param K weight matrix of shape 8x8
 * @param u a vector consisting of singular values of Sigma matrix in SVD decomposition of A(Jacobian matrix)
 * @param V right singular matrix of SVD decomposition
 * @param u_inv inverse of the diagonal matrix consisting of eigenvalues of Sigma matrix
 * @param M number of wheel units (8) 
 * @param N platform force dimension (3) 
 * @param debug boolean used to run the function to display necessary print values
 */
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
                    const bool debug);

/**
 * @brief function to print a matrix
 * 
 * @param m matrix to be printed
 */
void print_matrix(const gsl_matrix *m);
#endif
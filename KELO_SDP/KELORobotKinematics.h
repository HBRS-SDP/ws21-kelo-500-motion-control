/**
 * @file KELORobotKinematics.h
 * @author Kishan Sawant (kishan.sawant@smail.inf.h-brs.de)
 * @brief function to calculate the 3x8 Jacobian matrix(A) and to calculate forces at individual wheel units
 * @date 2022-02-06
 * 
 */
#ifndef KELO_ROBOT_KINEMATICS_H
#define KELO_ROBOT_KINEMATICS_H

#include <gsl/gsl_matrix_double.h>

/**
 * @brief function to calculate the 3x8 matrix(A) to map force from individual wheels to platform
 * 
 * @param A 3x8 matrix(Jacobian) which maps form individual wheels to platform
 * @param pivot_angles 1D array of shape (4,) that stores current pivot angles of individual pair of wheels
 * @param wheel_coordinates 1-D array of shape (8,) which holds the co-ordinates of individual wheels (x1,y1,x2,y2,..,y4)
 */
void jacobian_matrix_calculator(gsl_matrix *A,
                                const double *pivot_angles,
                                const double *wheel_coordinates);

/**
 * @brief to solve the inverse of the Jacobian matrix(A) and calculate the pivot forces using it
 * 
 * @param pivot_forces forces at individual wheel units
 * @param A Jacobian matrix of shape 3x8, which is further used to store W.dot(A).dot(K)
 * @param A_tmp a temporary matrix used to store the matrix multiplication of W.dot(A)
 * @param A_inv_T_tmp used as temporary memory to calculate A_inv_T
 * @param A_inv_T initially store left singular matrix of SVD decomposition of A and later it stores the inverse of W.dot(A).dot(K)
 * @param u a vector consisting of singular values of Sigma matrix in SVD decomposition of A(Jacobian matrix)
 * @param u_inv inverse of the diagonal matrix consisting of eigenvalues of Sigma matrix
 * @param V right singular matrix of SVD decomposition
 * @param W weight matrix of shape 3x3
 * @param K weight matrix of shape 8x8
 * @param work temporary memory to facilitate SVD of Jacobian matrix(A)
 * @param b platform force matrix of shape 3x1
 * @param M number of wheel units (8) 
 */
void force_vector_finder(double *pivot_forces,
                         gsl_matrix *A,
                         gsl_matrix *A_tmp,
                         gsl_matrix *A_inv_T_tmp,
                         gsl_matrix *A_inv_T,
                         gsl_vector *u,
                         gsl_matrix *u_inv,
                         gsl_matrix *V,
                         const gsl_matrix *W,
                         const gsl_matrix *K,
                         gsl_vector *work,
                         const gsl_matrix *b,
                         const unsigned int M);

#endif

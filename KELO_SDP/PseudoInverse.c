#include <stdio.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_vector_double.h>
#include "PseudoInverse.h"

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
                         const unsigned int M)
{
    /* 
    Equations:
    J# = Mq * V * pinv_dls(S) * U.T * Mx
    B  = Mx * J * Mq (Note: B = U * S * V.T)
    J# * b = pivot_forces
    */

    // B  = Mx * J * Mq
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, W, A, 0.0, A_tmp);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A_tmp, K, 0.0, A);
    gsl_matrix_transpose_memcpy(A_inv_T, A); // we need M>N to solve SVD, but as M < N, transpose of A has been considered
    // Note: singular values are invariant to matrix transpose, as S = S.T
     
    // B = U * S * V.T 
    gsl_linalg_SV_decomp(A_inv_T, V, u, work); // SVD decomposition (A gets replaced by values of U matrix)
    gsl_matrix_transpose(V); // here V is indeed U, as transpose of A is being considered

    // finding S_inv
    size_t i;
    for (i = 0; i < u->size; i++)
    {
        gsl_matrix_set(u_inv, i, i, 1/gsl_vector_get(u,i));
    }

    // J# = M_q * V * pinv_dls(S) * U.T * M_x
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, A_inv_T, 0.0, A_inv_T_tmp); 
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A_inv_T_tmp, u_inv, 0.0, A_inv_T);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A_inv_T, V, 0.0, A_inv_T_tmp);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A_inv_T_tmp, W, 0.0, A_inv_T);
    gsl_matrix_view _X = gsl_matrix_view_array(pivot_forces,M,1);

    // J# * b = pivot_forces
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A_inv_T, b, 0.0, &_X.matrix);
}
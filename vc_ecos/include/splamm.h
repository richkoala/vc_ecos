/*
 * ECOS - Embedded Conic Solver.
 * Copyright (C) 2012-2015 A. Domahidi [domahidi@embotech.com],
 * Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 * Sparse linear algebra library for setup phase, i.e. this module
 * accesses malloc and hence should not go on an embedded platform.
 */

#ifndef __SPLAMM_H__
#define __SPLAMM_H__

#include "glblopts.h"
#include "spla.h"


/**
 * Create a sparse matrix from existing arrays.
 */
spmat* ecoscreateSparseMatrix(idxint m, idxint n, idxint nnz, idxint* jc, idxint* ir, pfloat* pr);


/**
 * Create a new sparse matrix (uses MALLOC!)
 */
spmat* newSparseMatrix(idxint m, idxint n, idxint nnz);

/**
 * Create a new sparse matrix (uses FREE!)
 */
void freeSparseMatrix(spmat* M);


/**
 * Transpose a matrix; returns A = M' (uses malloc!)
 */
spmat* transposeSparseMatrix(spmat* M, idxint* MtoMt);


/**
 * Permutes a symmetric matrix with only the upper triangular part stored.
 * Writes the upper triangular part of C = A(p,p) in column compressed
 * storage format.
 *
 * The function additionally returns the mapping PK that maps the row indices
 * of the sparse matrix A on the row indices of C, such that C[P[k]] = A[k].
 *
 * NOTE: The matrix C and the vector PK are NOT created within this function 
 *       - you need to allocate them beforehand!!
 *
 * If PK is NULL then the last output argument is ignored.
 */
void permuteSparseSymmetricMatrix(spmat* A, idxint* pinv, spmat* C, idxint* PK);


/**
 * Returns the inverse of permutation p of length n.
 */
void pinv(idxint n, idxint* p, idxint* pinv);


/**
 * Returns a copy of a sparse matrix A.
 */
spmat* copySparseMatrix(spmat* A);

/* ============================= DEBUG FUNCTIONS ======================= */
#if PRINTLEVEL > 0

/**
 * Prints a dense matrix.
 */
void printDenseMatrix(pfloat *M, idxint dim1, idxint dim2, char *name);


/**
 * Prints a dense integer matrix.
 */
void printDenseMatrix_i(idxint *M, idxint dim1, idxint dim2, char *name);


/**
 * Prints a sparse matrix.
 */
void printSparseMatrix(spmat* M);

/**
 * Convert sparse matrix to dense matrix.
 */
void Spmat2Demat(spmat* SpM,demat* DeM);
void dumpDemat(demat* M, idxint len,char* fn);

void Spmat2Demat_hw_imp(spmat* SpM,demat_struct* DeM, idxint idx);
void dumpDemat_hw_imp(demat_struct* M, idxint cmd_len, idxint data_len,char* fn);
void dumpDevec_hw_imp(devec_struct* M, idxint cmd_len, idxint data_len,char* fn);
void dumpVec_hw_imp(int* M, idxint cmd_len, idxint data_len,char* fn);

/**
 * Dumps a sparse matrix in Matlab format.
 * Use SPCONVERT to read in the file.
 */
void dumpSparseMatrix(spmat* M, char* fn);


/**
 * Dumps a dense matrix of doubles to a CSV file.
 */
void dumpDenseMatrix(pfloat *M, int dim1, int dim2, char *fn);

/**
 * Dumps a dense matrix of integers to a CSV file.
 */
void dumpDenseMatrix_i(idxint *M, int dim1, int dim2, char *fn);

#endif


dump_config_int config_dat_wr_format_h(
					dump_config_int m,dump_config_int n,dump_config_int p,dump_config_int l,dump_config_int ncones,
					dump_config_float *c,// dump_config_int c_len, 
					dump_config_float *h,// dump_config_int h_len,
					dump_config_float *b,// dump_config_int b_len,
					dump_config_int   *q,// dump_config_int q_len,
					
					dump_config_int   *Gjc,
					dump_config_int   *Gir,
					dump_config_float *Gpr,
					dump_config_int	   G_nnz_len,
					
					dump_config_int   *Ajc,
					dump_config_int	  *Air,
					dump_config_float *Apr,
					dump_config_int	   A_nnz_len,
					char *fn);


dump_config_int config_dat_batch_wr(
					dump_config_int    *idx,
					dump_config_int m,dump_config_int n,dump_config_int p,dump_config_int l,dump_config_int ncones,
					dump_config_float  *c,
					dump_config_float  *h,
					dump_config_float  *b,
					dump_config_int    *q,
					
					dump_config_int    *Gjc,
					dump_config_int    *Gir,
					dump_config_float  *Gpr,
					dump_config_int		G_nnz_len,
					
					dump_config_int    *Ajc,
					dump_config_int	   *Air,
					dump_config_float  *Apr,
					dump_config_int		A_nnz_len,
					char *fn);

dump_config_int config_dat_batch_rd(
					dump_config_int    *idx,	
					dump_config_int *m,dump_config_int *n,dump_config_int *p,dump_config_int *l,dump_config_int *ncones,
					dump_config_float  *c,
					dump_config_float  *h,
					dump_config_float  *b,
					dump_config_int    *q,
					
					dump_config_int    *Gjc,
					dump_config_int    *Gir,
					dump_config_float  *Gpr,

					dump_config_int    *Ajc,
					dump_config_int	   *Air,
					dump_config_float  *Apr,
					char *fn
					);

void dumpDenseMatrix_UD(pfloat *M, int dim1, int dim2, char *fn);
void dumpDenseMatrix_i_UD(idxint *M, int dim1, int dim2, char *fn);


#endif


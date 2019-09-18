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
 * accesses MALLOC and hence should not go on an embedded platform.
 */

#include "splamm.h"
#include "errno.h"
#ifndef ZCU102_HW_IMP
#include <io.h>
#include <direct.h>
#endif

/* SYSTEM INCLUDES FOR MEMORY ALLOCATION ------------------------------- */
#include <stdio.h>
#if PRINTLEVEL > 2
	#include <stdlib.h>
#endif


/* 
 * Cumulative sum. p[i] = sum w[0..i-1] and w = p on output.
 * Defined for vectors of length m.
 */
void spla_cumsum(idxint* p, idxint* w, idxint m)
{
	idxint cumsum = 0;
	idxint i;
	for( i=0; i < m; i++ ){ 
		p[i] = cumsum; 
		cumsum += w[i]; 
		w[i] = p[i];
	}
}


/**
 * Returns the inverse of permutation p of length n.
 */
void pinv(idxint n, idxint* p, idxint* pinv)
{
	idxint i;
	for( i=0; i<n; i++ ){ pinv[p[i]] = i; }
}


/**
 * Transpose a matrix; returns A = M',
 * and an index vector MtoMt that directly maps elements of M
 * to elements of M'.
 */
spmat* transposeSparseMatrix(spmat* M, idxint* MtoMt)
{	
	idxint j, i, k, q;    
	idxint* w;
  
  spmat* A = newSparseMatrix(M->n, M->m, M->nnz);
  if (M->nnz == 0) return A;
  
	w = (idxint *)MALLOC(M->m*sizeof(idxint));

	/* row count: how often does row k occur in M? */
	for( i=0; i < M->m; i++ ) { w[i] = 0; }
	for( k=0; k < M->nnz; k++ ) { w[M->ir[k]]++; }
	
	/* row pointers: cumulative sum of w gives A->jc */
	spla_cumsum(A->jc, w, M->m);

	/* now walk through M and copy data to right places and set row counter */
	for( j=0; j < M->n; j++ ){
		for( k = M->jc[j]; k < M->jc[j+1]; k++ ){
			q = w[M->ir[k]]++;
			A->ir[q] = j;
			A->pr[q] = M->pr[k];
			MtoMt[k] = q;
		}
	}

	FREE(w);
	return A;
}


/**
 * Create a new sparse matrix (uses MALLOC!) 
 */
spmat* newSparseMatrix(idxint m, idxint n, idxint nnz)
{
	idxint* jc = (idxint *)MALLOC((n+1)*sizeof(idxint));
	idxint* ir = (idxint *)MALLOC(nnz*sizeof(idxint));
	pfloat* pr = (pfloat *)MALLOC(nnz*sizeof(pfloat));
	jc[n] = nnz;
	return ecoscreateSparseMatrix(m, n, nnz, jc, ir, pr);
}


/** 
 * Create a sparse matrix from existing arrays (no MALLOC) 
 */
spmat* ecoscreateSparseMatrix(idxint m, idxint n, idxint nnz, idxint* jc, idxint* ir, pfloat* pr)
{
	spmat* M = (spmat *)MALLOC(sizeof(spmat));
	M->m = m;
	M->n = n;    
	M->nnz = nnz;	
	M->jc = jc;
    M->ir = ir;
    M->pr = pr;
	if (M->jc) M->jc[n] = nnz;
	return M;
}


/**
 * Create a new sparse matrix (uses FREE!)
 */
void freeSparseMatrix(spmat* M)
{
	if (M->ir) FREE(M->ir);
	if (M->jc) FREE(M->jc);
	if (M->pr) FREE(M->pr);
	FREE(M);
}


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
void permuteSparseSymmetricMatrix(spmat* A, idxint* pinv, spmat* C, idxint* PK)
{
	idxint i, i2, j, j2, k, q;
	idxint* w;

	/* create matrix, permutation vector and temporary work space */	
	w = (idxint *)MALLOC(A->n*sizeof(idxint));

	/* count number of entries per column of C, store this in w */
	for( j=0; j < A->n; j++ ){ w[j] = 0; }
	for( j=0; j < A->n; j++ ){
		j2 = pinv[j];
		for( k=A->jc[j]; k < A->jc[j+1]; k++ ){
			i = A->ir[k];
			if( i > j ) continue; /* skip lower triangular part of A */
			i2 = pinv[i];
			w[ i2 > j2 ? i2 : j2]++;
		}
	}

	/* cumulative sum gives column pointers */
	spla_cumsum(C->jc, w, A->n);

	/* copy data over */
	for( j=0; j < A->n; j++ ){
		j2 = pinv[j];
		for( k=A->jc[j]; k < A->jc[j+1]; k++ ){
			i = A->ir[k];
			if( i > j ) continue; /* skip lower triangular part of A */
			i2 = pinv[i];
			q = w[i2 > j2 ? i2 : j2]++;
			C->ir[q] = i2 < j2 ? i2 : j2;
			C->pr[q] = A->pr[k];
			if( PK ) PK[k] = q;
		}
	}

	/* free work space */
	FREE(w);
}


/*
 * Returns a copy of a sparse matrix A.
 */
spmat* copySparseMatrix(spmat* A)
{	
	idxint i;
	spmat* B = newSparseMatrix(A->m, A->n, A->nnz);

	/* copy over */
	for( i=0; i<=A->n; i++ ){ B->jc[i] = A->jc[i]; }
	for( i=0; i<A->nnz; i++ ){ B->ir[i] = A->ir[i]; }
	for( i=0; i<A->nnz; i++ ){ B->pr[i] = A->pr[i]; }

	return B;
}

//FPGA transfer Dense Matrix format ROW / COL / VAULE
void Spmat2Demat(spmat* SpM, demat* DeM)
{
    idxint j, i, row_strt, row_stop;
    idxint k = 0;
	idxint idx = 0;

	//initial Dense Mat
    for(j=0; j<SpM->n; j++){
        row_strt = SpM->jc[j];
        row_stop = SpM->jc[j+1];        
        if (row_strt == row_stop)
            continue;
        else {
            for(i=row_strt; i<row_stop; i++ ){  
				DeM[idx].row = (int)SpM->ir[i]+1;
				DeM[idx].col = (int)j+1;
				DeM[idx].val = SpM->pr[k++];
				idx += 1;
            }
        }
    }
}

//初始化矩阵格式转化处理
void Spmat2Demat_hw_imp(spmat* SpM,demat_struct* DeM, idxint idx)
{
    idxint j, i, row_strt, row_stop;
    idxint k = 0;

	for(j=0; j<idx; j++){
		DeM[idx].frame_id_or_row = 0;
		DeM[idx].frame_len_or_col = 0;
		DeM[idx].double_data2 = 0;
	}

	//initial Dense Mat
    for(j=0; j<SpM->n; j++){
        row_strt = SpM->jc[j];
        row_stop = SpM->jc[j+1];
        if (row_strt == row_stop)
            continue;
        else {
            for(i=row_strt; i<row_stop; i++ ){
				DeM[idx].frame_id_or_row = (int)SpM->ir[i]+1;
				DeM[idx].frame_len_or_col = (int)j+1;
				DeM[idx].double_data2 = SpM->pr[k++];
				idx += 1;
            }
        }
    }
}

void dumpDemat(demat* M, idxint len,char* fn)
{
#if DEBUG == 1
	idxint i = 0;
	FILE *f = fopen(fn,"w");
	if( f != NULL ){
		for(i=0; i<len; i++){               
			fprintf(f,"%d\t%d\t%20.18e\n", (int)M[i].row, (int)M[i].col, M[i].val);
		}
        fprintf(f,"%d\t%d\t%20.18e\n", len, len, 0.0);
		fclose(f);
		PRINTTEXT("File %s successfully written.\n", fn);
	} else {
		PRINTTEXT("Error during writing file %s.\n", fn);
	}
#else
#endif
}

//转化后发送给pl的数据dump处理
void dumpDemat_hw_imp(demat_struct* M, idxint cmd_len, idxint data_len,char* fn)
{
#if DEBUG == 1
	idxint i = 0;
	FILE *f = fopen(fn,"w");
	if( f != NULL ){
		for (i=0; i<cmd_len; i++){
			if (i==0)
				fprintf(f,"%10d\t%10d\t%10d\t%10d\n", (int)M[i].frame_id_or_row,(int)M[i].frame_len_or_col,(int)M[i].cnt,(int)M[i].iter_num);
			else{
				//printf("%20.18lf\t20.18%lf\n", (double)M[i].double_data1,(double)M[i].double_data2);
				fprintf(f,"%20.18lf\t%20.18lf\n", (double)M[i].double_data1,(double)M[i].double_data2);
			}
		}
		for(i=cmd_len; i<cmd_len+data_len; i++){               
			fprintf(f,"%d\t%d\t%20.18lf\n", (int)M[i].frame_id_or_row, (int)M[i].frame_len_or_col, M[i].double_data2);
		}
        fprintf(f,"%d\t%d\t%20.18e\n", cmd_len+data_len, cmd_len+data_len, 0.0);
		fclose(f);
		PRINTTEXT("File %s successfully written.\n", fn);
	} else {
		PRINTTEXT("Error during writing file %s.\n", fn);
	}
#else
#endif
}

void dumpDevec_hw_imp(devec_struct* M, idxint cmd_len, idxint data_len,char* fn)
{
#if DEBUG == 1
	idxint i = 0;
	FILE *f = fopen(fn,"w");
	if( f != NULL ){

		fprintf(f,"%10d\t%10d%10d\t%10d\n", (int)M[0].frame_id_or_cnt,(int)M[0].frame_len_or_iter_num,(int)M[1].frame_id_or_cnt,(int)M[1].frame_len_or_iter_num);

		for(i=cmd_len*2; i<cmd_len*2+data_len; i++){               
			fprintf(f,"%20.18lf\n", (double)M[i].double_data1);
		}
        fprintf(f,"%d\t%d\t%20.18e\n", cmd_len+data_len, cmd_len+data_len, 0.0);
		fclose(f);
		PRINTTEXT("File %s successfully written.\n", fn);
	} else {
		PRINTTEXT("Error during writing file %s.\n", fn);
	}
#else
#endif
}

void dumpVec_hw_imp(int* M, idxint cmd_len, idxint data_len,char* fn)
{
#if DEBUG == 1
	idxint i = 0;
	FILE *f = fopen(fn,"w");
	if( f != NULL ){
		if (i==0)
			fprintf(f,"%10d\t%10d\%10d\t%10d\n", (int)M[i],  (int)M[i+1],(int)M[i+2],  (int)M[i+3]);

		for(i=cmd_len*4; i<cmd_len*4+data_len; i++){               
			fprintf(f,"%08d\n", M[i]);
		}
        fprintf(f,"%d\t%d\t%20.18e\n", cmd_len+data_len, cmd_len+data_len, 0.0);
		fclose(f);
		PRINTTEXT("File %s successfully written.\n", fn);
	} else {
		PRINTTEXT("Error during writing file %s.\n", fn);
	}
#else
#endif
}

/* dump a sparse matrix in Matlab format */
/* use LOAD and SPCONVERT to read in the file in MATLAB */
void dumpSparseMatrix(spmat* M, char* fn)
{
#if DEBUG == 1
	idxint j, i, row_strt, row_stop;
    idxint k = 0;
	FILE *f = fopen(fn,"w");
	if( f != NULL ){
		for(j=0; j<M->n; j++){
			row_strt = M->jc[j];
			row_stop = M->jc[j+1];        
			if (row_strt == row_stop)
				continue;
			else {
				for(i=row_strt; i<row_stop; i++ ){                
					fprintf(f,"%d\t%d\t%20.18e\n", (int)M->ir[i]+1, (int)j+1, M->pr[k++]);
				}
			}
		}
        fprintf(f,"%d\t%d\t%20.18e\n", (int)M->m, (int)M->n, 0.0);
		fclose(f);
		PRINTTEXT("File %s successfully written.\n", fn);
	} else {
		PRINTTEXT("Error during writing file %s.\n", fn);
	}
#else
#endif
}


/**
 * Dumps a dense matrix of doubles to a CSV file.
 */
void dumpDenseMatrix(pfloat *M, int dim1, int dim2, char *fn)
{
#if DEBUG == 1
    FILE *f = fopen(fn,"w");
    int i,j;
    
    /* swap dimensions if only vector to be printed */
    if( dim1 == 1 && dim2 > 1){
        i = dim2;
        dim2 = dim1;
        dim1 = i;
    }
    
    if( f != NULL ){
        for( i=0; i<dim1; i++ ){
            if( dim2 > 1 ){
                for( j=0; j<dim2-1; j++ ){
                    fprintf(f, "%+18.16e,",M[i*dim2+j]);
                }
                j = dim2;
                fprintf(f, "%+18.16e\n",M[i*dim2+j]);                
            } else {
                fprintf(f, "%+18.16e\n",M[i]);                
            }                
        }
		fprintf(f,"%d\t%d\t%20.18e\n", dim1, dim2, 0.0);
        fclose(f);
        printf("Written %d x %d matrix to '%s'.\n",i,dim2,fn);
    } else {
        printf("ERROR: file %s could not be opened. Exiting.",fn);
        exit(1);
    }  
#else
#endif
}


/**
 * Dumps a dense matrix of integers to a CSV file.
 */
void dumpDenseMatrix_i(idxint *M, int dim1, int dim2, char *fn)
{
#if DEBUG == 1
    FILE *f = fopen(fn,"w");
    int i,j;
    
    /* swap dimensions if only vector to be printed */
    if( dim1 == 1 && dim2 > 1){
        i = dim2;
        dim2 = dim1;
        dim1 = i;
    }
    
    if( f != NULL ){
        for( i=0; i<dim1; i++ ){
            if( dim2 > 1 ){
                for( j=0; j<dim2-1; j++ ){
                    fprintf(f, "%d,",(int)M[i*dim2+j]);
                }
                j = dim2;
                fprintf(f, "%d\n",(int)M[i*dim2+j]);
            } else {
                fprintf(f, "%d\n",(int)M[i]);
            }                
        }
		fprintf(f,"%d\t%d\t%20.18e\n", dim1, dim2, 0.0);
        fclose(f);
        printf("Written %d x %d matrix to '%s'.\n",i,dim2,fn);
    } else {
        printf("ERROR: file %s could not be opened. Exiting.",fn);
        exit(1);
    }   
#else
#endif
}



/* ================================= DEBUG FUNCTIONS ======================= */
#if PRINTLEVEL > 3
/**
 * Prints a dense matrix.
 */
void printDenseMatrix(pfloat *M, idxint dim1, idxint dim2, char *name)
{
    idxint i,j;
    PRINTTEXT("%s = \n\t",name);
    for( i=0; i<dim1; i++ ){
        for( j=0; j<dim2; j++ ){
            if( j<dim2-1 )
                PRINTTEXT("% 14.12e,  ",M[i*dim2+j]);
            else
                PRINTTEXT("% 14.12e;  ",M[i*dim2+j]);
        }
        if( i<dim1-1){
            PRINTTEXT("\n\t");
        }
    }
    PRINTTEXT("\n");
}


/**
 * Prints a dense integer matrix.
 */
void printDenseMatrix_i(idxint *M, idxint dim1, idxint dim2, char *name)
{
    idxint i,j;
    PRINTTEXT("%s = \n\t",name);
    for( i=0; i<dim1; i++ ){
        for( j=0; j<dim2; j++ ){
            if( j<dim2-1 )
                PRINTTEXT("%d,  ",(int)M[i*dim2+j]);
            else
                PRINTTEXT("%d;  ",(int)M[i*dim2+j]);
        }
        if( i<dim1-1){
            PRINTTEXT("\n\t");
        }
    }
    PRINTTEXT("\n");
}


/*
 * Prints a sparse matrix.
 */
void printSparseMatrix(spmat* M)
{
    idxint j, i, row_strt, row_stop;
    idxint k = 0;
    for(j=0; j<M->n; j++){
        row_strt = M->jc[j];
        row_stop = M->jc[j+1];
        if (row_strt == row_stop)
            continue;
        else {
            for(i=row_strt; i<row_stop; i++ ){
                PRINTTEXT("\t(%3u,%3u) = %g\n", (int)M->ir[i]+1, (int)j+1, M->pr[k++]);
            }
        }
    }
}



#endif

#ifdef ZCU102_HW_IMP
#else
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
					char *fn)

{
    FILE *fpwr_config;   //  头文件#include <stdio.h>
	dump_config_int i;
    if((fpwr_config=fopen(fn,"w"))==NULL)
    {
        printf("file cannot open \n");
        //exit(0);  头文件#include <stdlib.h>
        //exit结束程序，一般0为正常推出，其它数字为异常，其对应的错误可以自己指定。
    }
    else{	
        printf("file opened for writing \n");
        fprintf(fpwr_config, "idxint n = %d;\n", n); 
		fprintf(fpwr_config, "idxint m = %d;\n", m); 
        fprintf(fpwr_config, "idxint p = %d;\n", p); 
        fprintf(fpwr_config, "idxint l = %d;\n", l); 
        fprintf(fpwr_config, "idxint ncones = %d;\n", ncones);
		
		fprintf(fpwr_config, "pfloat c[%d] = {",n);
        for (i=0;i<n;i++)
        	fprintf(fpwr_config, "%.18f,\t", c[i]);
		fprintf(fpwr_config, "};\n");

		fprintf(fpwr_config, "pfloat h[%d] = {",m);
        for (i=0;i<m;i++)
        	fprintf(fpwr_config, "%.18f,\t", h[i]);
		fprintf(fpwr_config, "};\n");
		
		fprintf(fpwr_config, "pfloat b[%d] = {",p);
        for (i=0;i<p;i++)
        	fprintf(fpwr_config, "%.18f,\t", b[i]);
		fprintf(fpwr_config, "};\n");
		
		fprintf(fpwr_config, "idxint q[%d] = {",ncones);
        for (i=0;i<ncones;i++)
        	fprintf(fpwr_config, "%d,\t", q[i]);
		fprintf(fpwr_config, "};\n");
		
		//===G_col_len 不理解**
		fprintf(fpwr_config, "idxint Gjc[%d] = {",n+1);
        for (i=0;i<n+1;i++)
        	fprintf(fpwr_config, "%d,\t", Gjc[i]);
		fprintf(fpwr_config, "};\n");

		fprintf(fpwr_config, "idxint Gir[%d] = {",G_nnz_len);
        for (i=0;i<G_nnz_len;i++)
        	fprintf(fpwr_config, "%d,\t", Gir[i]);
		fprintf(fpwr_config, "};\n");
		
		fprintf(fpwr_config, "pfloat Gpr[%d] = {",G_nnz_len);
        for (i=0;i<G_nnz_len;i++)
        	fprintf(fpwr_config, "%.18f,\t", Gpr[i]);
		fprintf(fpwr_config, "};\n");

		//===A_col_len
		fprintf(fpwr_config, "idxint Ajc[%d] = {",n+1);
        for (i=0;i<n+1;i++)
        	fprintf(fpwr_config, "%d,\t", Ajc[i]);
		fprintf(fpwr_config, "};\n");

		fprintf(fpwr_config, "idxint Air[%d] = {",A_nnz_len);
        for (i=0;i<A_nnz_len;i++)
        	fprintf(fpwr_config, "%d,\t", Air[i]);
		fprintf(fpwr_config, "};\n");
		
		fprintf(fpwr_config, "pfloat Apr[%d] = {",A_nnz_len);
        for (i=0;i<A_nnz_len;i++)
        	fprintf(fpwr_config, "%.18f,\t", Apr[i]);
		fprintf(fpwr_config, "};\n");
    }
        
    if(fclose(fpwr_config)!=0)
        printf("config_file wr operation cannot be closed \n");
    else
        printf("config_file wr operation is now closed \n");
    return 0;
}

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
					char *fn)
{
   
	FILE *fpwr_config;   //  头文件#include <stdio.h>
	dump_config_int i;
	dump_config_int data_len;

    if((fpwr_config=fopen(fn,"a"))==NULL)
    {
        printf("file cannot open \n");
        //exit(0);  头文件#include <stdlib.h>
        //exit结束程序，一般0为正常推出，其它数字为异常，其对应的错误可以自己指定。
    }
    else{	

		*idx = *idx + 1;
		data_len = A_nnz_len*2 + G_nnz_len*2 + 2*(n+1) + m+ n + p+ ncones+7;
        printf("file opened for writing \n");
		fprintf(fpwr_config, "frame_idx : %8d\n", *idx); 
		fprintf(fpwr_config, "dv_len    : %8d\n", data_len);
		fprintf(fpwr_config, "A_nnz_len = %8d\n", A_nnz_len);
		fprintf(fpwr_config, "G_nnz_len = %8d\n", G_nnz_len);
        fprintf(fpwr_config, "m         = %8d\n", m); 
        fprintf(fpwr_config, "n         = %8d\n", n); 
        fprintf(fpwr_config, "p         = %8d\n", p); 
        fprintf(fpwr_config, "l         = %8d\n", l); 
        fprintf(fpwr_config, "ncones    = %8d\n", ncones);

        for (i=0;i<n;i++)
        	fprintf(fpwr_config, "%.18f\n", c[i]);
        for (i=0;i<m;i++)
        	fprintf(fpwr_config, "%.18f\n", h[i]);
        for (i=0;i<p;i++)
        	fprintf(fpwr_config, "%.18f\n", b[i]); 
        	
        for (i=0;i<ncones;i++)
        	fprintf(fpwr_config, "%d\n", q[i]); 
        
        for (i=0;i<n+1;i++)
        	fprintf(fpwr_config, "%d\n", Gjc[i]);
        for (i=0;i<G_nnz_len;i++)
        	fprintf(fpwr_config, "%d\n", Gir[i]);
        for (i=0;i<G_nnz_len;i++)
        	fprintf(fpwr_config, "%.18f\n", Gpr[i]); 
        	
        for (i=0;i<n+1;i++)
        	fprintf(fpwr_config, "%d\n", Ajc[i]);
        for (i=0;i<A_nnz_len;i++)
        	fprintf(fpwr_config, "%d\n", Air[i]);
        for (i=0;i<A_nnz_len;i++)
        	fprintf(fpwr_config, "%.18f\n", Apr[i]); 

		fprintf(fpwr_config, "====================\n");  
    }
        
    if(fclose(fpwr_config)!=0)
        printf("config_file wr operation cannot be closed \n");
    else
        printf("config_file wr operation is now closed \n");
    return 0;
}

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
					)
{
    FILE *fprd_config;   //  头文件#include <stdio.h>
	dump_config_int A_nnz_len;
	dump_config_int G_nnz_len;
	dump_config_int i;
	dump_config_int j=0;
    if((fprd_config=fopen(fn,"r"))==NULL)
    {
        printf("file cannot open \n");
        //exit(0);  头文件#include <stdlib.h>
        //exit结束程序，一般0为正常推出，其它数字为异常，其对应的错误可以自己指定。
    }
    else{	
        printf("file opened for reading \n");
        
		*idx = *idx + 1;

		for (i=0;i<9*3;i++){
			fscanf(fprd_config,"%s",str1); 
			if (i%3==2)
			{
				j++;
				if (j==3)
					A_nnz_len=atoi(str1);
				else if (j==4)
					G_nnz_len=atoi(str1);
				else if (j==5)
					*m     	= atoi(str1);
				else if (j==6)
					*n     	= atoi(str1);
				else if (j==7)
					*p     	= atoi(str1);
				else if (j==8)
					*l     	= atoi(str1);
				else if (j==9)
					*ncones	= atoi(str1);
					
			}
		}

		printf("A_nnz_len = %d\n",A_nnz_len);  
		printf("G_nnz_len = %d\n",G_nnz_len);  
		printf("m         = %d\n",*m        );  
		printf("n         = %d\n",*n        );  
		printf("p         = %d\n",*p        );  
		printf("l         = %d\n",*l        );  
		printf("ncones    = %d\n",*ncones   );

        for (i=0;i<*n;i++)
        	fscanf(fprd_config,"%lf",&c[i]); 
        for (i=0;i<*m;i++)
        	fscanf(fprd_config,"%lf",&h[i]);
        for (i=0;i<*p;i++)
        	fscanf(fprd_config,"%lf",&b[i]); 
        	
        for (i=0;i<*ncones;i++)
        	fscanf(fprd_config,"%d",&q[i]); 
        
        for (i=0;i<(*n+1);i++)
        	fscanf(fprd_config,"%d",&Gjc[i]);
        for (i=0;i<G_nnz_len;i++)
        	fscanf(fprd_config,"%d",&Gir[i]);
        for (i=0;i<G_nnz_len;i++)
        	fscanf(fprd_config,"%lf",&Gpr[i]); 
        	
        for (i=0;i<(*n+1);i++)
        	fscanf(fprd_config,"%d",&Ajc[i]);
        for (i=0;i<A_nnz_len;i++)
        	fscanf(fprd_config,"%d",&Air[i]);
        for (i=0;i<A_nnz_len;i++)
        	fscanf(fprd_config,"%lf",&Apr[i]); 

		fscanf(fprd_config,"%s",str1); 
		printf("=====================================\n",*idx);
		printf("frame_idx = %4d cfg_data load done\n",*idx);
		printf("=====================================\n",*idx);
    }
    
	if (*idx == FRAME_NUM){
		if(fclose(fprd_config)!=0)
			printf("config_file rd operation cannot be closed \n");
		else
			printf("config_file rd operation is now closed \n");
	}
	return 0;
}

void dumpDenseMatrix_UD(pfloat *M, int dim1, int dim2, char *fn)
{
    FILE *f = fopen(fn,"w");
    int i,j;
    
    /* swap dimensions if only vector to be printed */
    if( dim1 == 1 && dim2 > 1){
        i = dim2;
        dim2 = dim1;
        dim1 = i;
    }
    
    if( f != NULL ){
        for( i=0; i<dim1; i++ ){
            if( dim2 > 1 ){
                for( j=0; j<dim2-1; j++ ){
                    fprintf(f, "%+18.16e,",M[i*dim2+j]);
                }
                j = dim2;
                fprintf(f, "%+18.16e\n",M[i*dim2+j]);                
            } else {
                fprintf(f, "%+18.16e\n",M[i]);                
            }                
        }
        fclose(f);
        printf("Written %d x %d matrix to '%s'.\n",i,dim2,fn);
    } else {
        printf("ERROR: file %s could not be opened. Exiting.",fn);
        exit(1);
    }  
}

void dumpDenseMatrix_i_UD(idxint *M, int dim1, int dim2, char *fn)
{
    FILE *f = fopen(fn,"w");
    int i,j;
    
    /* swap dimensions if only vector to be printed */
    if( dim1 == 1 && dim2 > 1){
        i = dim2;
        dim2 = dim1;
        dim1 = i;
    }
    
    if( f != NULL ){
        for( i=0; i<dim1; i++ ){
            if( dim2 > 1 ){
                for( j=0; j<dim2-1; j++ ){
                    fprintf(f, "%d,",(int)M[i*dim2+j]);
                }
                j = dim2;
                fprintf(f, "%d\n",(int)M[i*dim2+j]);
            } else {
                fprintf(f, "%d\n",(int)M[i]);
            }                
        }
        fclose(f);
        printf("Written %d x %d matrix to '%s'.\n",i,dim2,fn);
    } else {
        printf("ERROR: file %s could not be opened. Exiting.",fn);
        exit(1);
    }
}
#endif

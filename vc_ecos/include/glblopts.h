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


/* data type definitions used with ECOS */
#include "cone.h"

#ifndef __GLBLOPTS_H__
#define __GLBLOPTS_H__

#define DATA_PATH "../../../data/"



//#define ZCU102_HW_IMP					//实现平台为zcu102
#define PEOC_REORDER_PROTOCAL_SET		//硬件并行计算处理流程与对应数据产生，将可以并行处理的代码放在一起
#define KKT_FACTOR_PL_PROCESS 0			//1 使能Pl完成fpga完成kkt_factor操作
#define KKT_SOLVE_PL_PROCESS 0			//1 使能Pl完成fpga完成kkt_solve操作
#define KKT_SOLVE_PARALLEL

//#define CONE_SPARSE

//#define COMM_LOOP_TEST				//回环通讯测试

/* SET PRINT LEVEL ----------------------------------------------------- */
#define PRINTLEVEL (3)     /* 0: no prints					             */
						   /* 1: only final info				         */
                           /* 2: progress print per iteration            */
						   /* 3: debug level, enables print & dump fcns. */

#define MATLAB_FLUSH_PRINTS
                            /* print each iteration directly to Matlab.  */
                            /* this options considerably slows down the  */
                            /* solver, but is useful if you solve big    */
                            /* problems.                                 */

/* SET PROFILING LEVEL ------------------------------------------------- */
#define PROFILING (3)      /* 0: no timing information				     */
                           /* 1: runtime (divided in setup and solve)    */
                           /* 2: detailed profiling                      */
						   /* 3: HW design reference                     */

/* SET DEBUG LEVEL ----------------------------------------------------- */
#define DEBUG (1)          /* 0: no debugging information                */
                           /* 1: debug info & dump intermediate results  */
                           /* (flag used only for development)           */


//#define DEBUG (1)		   /*0 No enable				                 */
						   /*1 enable                                    */

#define CONFIG_DATA_LOAD_MODE  (5)		//0 load config data from config.txt  <github default>
										//1 load config data from data.h  --- <github default>
										//2 load config data from config.txt  <Lunar Landing>
										//3 load config data from data.h  --- <Lunar Landing>
										//4 load config data from config.txt  <Earth Landing>
										//5 load config data from data.h  --- <Earth Landing>

#define CONFIG_DATA_DUMP_MODE (0)		//0 no dump opertion       
										//1 dump to *.h   format   ONCE   MODE
										//2 dump to *.txt format   BATCH  MODE

#if  ( CONFIG_DATA_LOAD_MODE  == (0) || CONFIG_DATA_LOAD_MODE  == (1) )
	#define G_NNZ_LEN 220				// <github demo> 
	#define A_NNZ_LEN 638	
#elif( CONFIG_DATA_LOAD_MODE  == (2) || CONFIG_DATA_LOAD_MODE  == (3) )
	#define G_NNZ_LEN 995				// <Lunar Landing>
	#define A_NNZ_LEN 1717
#elif( CONFIG_DATA_LOAD_MODE  == (4) || CONFIG_DATA_LOAD_MODE  == (5) )
	#define G_NNZ_LEN 352				// <Earth Landing>
	#define A_NNZ_LEN 840								
#endif


#if CONFIG_DATA_LOAD_MODE  == (0)
	#define FRAME_NUM 3					//FRAME_NUM according frame length in config.txt
#else
	#define FRAME_NUM 1
#endif

/* DATA TYPES ---------------------------------------------------------- */
#include <float.h>
#include <math.h>
/* NOTE: Currently, pfloat MUST be double for ecos */
typedef double pfloat;              /* for numerical values  */
#define ECOS_INFINITY   (DBL_MAX + DBL_MAX)
#define ECOS_NAN        (ECOS_INFINITY - ECOS_INFINITY)

#if defined(_MSC_VER) && (_MSC_VER < 1900)
/* this will also return true if x is nan, but we don't check that anyway */
#define isinf(x) (!_finite(x))
#endif

/* Exponential cone */
#define EXPCONE      /*When defined the exponential cone solver code is enabled*/

/* SYSTEM INCLUDES FOR PRINTING ---------------------------------------- */
#if PRINTLEVEL > 0
#ifdef MATLAB_MEX_FILE
#include "mex.h"
#define PRINTTEXT mexPrintf
#elif defined PYTHON
#include <Python.h>
#define PRINTTEXT PySys_WriteStdout
#else
#define PRINTTEXT printf
#endif
#include <stdio.h>
#else
#define PRINTTEXT(...)
#endif

#include "SuiteSparse_config.h"

/* USE SAME NUMBER REPRESENTATION FOR INDEXING AS AMD AND LDL ---------- */
#if defined(DLONG) && !defined(LDL_LONG) || defined(LDL_LONG) && !defined(DLONG)
#error "Inconsistent definition of DLONG and LDL_LONG"
#endif

#ifdef DLONG
typedef SuiteSparse_long idxint;
#else
typedef int idxint;
#endif

typedef long   dump_config_int;		//zho
typedef double dump_config_float;   //zho
char str1[20];




/* SYSTEM INCLUDE IF COMPILING FOR MATLAB ------------------------------ */
#ifdef MATLAB_MEX_FILE
#include "mex.h"
#endif

/* CHOOSE RIGHT MEMORY MANAGER ----------------------------------------- */
#ifdef MATLAB_MEX_FILE
#define MALLOC mxMalloc
#define FREE mxFree
#else
#define MALLOC malloc
#define FREE free
#endif

#endif

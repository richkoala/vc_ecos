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


/* main file with example of how to run ECOS */

#include <stdio.h>
#include <stdlib.h>
#include <glblopts.h>
#ifdef ZCU102_HW_IMP
#include "xil_printf.h"
#include "platform.h"
#include "xparameters.h"
#include "sleep.h"
#include "gpio_led.h"
#include "comm_ps_pl.h"
#include "xbram_control.h"
#endif

#include "ecos.h"
#include "data.h"
#include "splamm.h"

int main(void)
{

	dump_config_int frame_idx=0;
	FILE *fid_log;
	dump_config_int log_idx=0;	
	char *log_name[80];
	char *log_path[80];
	char *file_setup[80];
	char *file_dout[80];
	char *load_data_file[80];

	/*char ver[7];*/
    idxint exitflag = ECOS_FATAL;
	pwork* mywork;
//#if PROFILING > 1 && PRINTLEVEL > 2
//    double torder, tkktcreate, ttranspose, tfactor, tkktsolve, ttotal, tsetup, tsolve;
//#endif

#if PROFILING ==3 && PRINTLEVEL > 0
	double torder, tkktcreate, ttranspose, tfactor, tkktsolve, ttotal, tsetup, tsolve;
#endif

#if PROFILING == 3
	int kkt_factor_cnt;
	int ldl_lsolve2_cnt;
	int ldl_dsolve_cnt;
	int ldl_ltsolve_cnt;
	double ldl_lsolve2_time;
	double ldl_dsolve_time;
	double ldl_ltsolve_time;
#endif

	kkt_sign_flag  = 0;
	kkt_factor_flag = 0;

	while(frame_idx<FRAME_NUM)		//²âÊÔ´ÎÊý
	{
		log_idx = log_idx + 1;
		
		
		sprintf(log_path,"%slog/run",DATA_PATH);
		sprintf(log_name,"%s_%d%s",log_path,log_idx,".log");
		fid_log = fopen(log_name,"w");
		strcpy(load_data_file,DATA_PATH);
		#if  CONFIG_DATA_LOAD_MODE == 0
			strcat(load_data_file,"data_load/config_github.txt");
		#elif  CONFIG_DATA_LOAD_MODE == 2
			strcat(load_data_file,"data_load/config_lunar.txt");
		#elif  CONFIG_DATA_LOAD_MODE == 4
			strcat(load_data_file,"data_load/config_earth.txt");
		#endif

//*==============CONFIG_DATA_OPT==============*//
	#if  CONFIG_DATA_LOAD_MODE%2 == 0			//config from the config.txt
		config_dat_batch_rd(  
					&frame_idx,
					&m,&n,&p,&l,&ncones,				
					c,
					h,
					b,
					q,
					Gjc,
					Gir,
					Gpr,
					Ajc,
					Air,
					Apr,
					load_data_file,
					);
	#else										//config from the data.h
		#if   CONFIG_DATA_DUMP_MODE == 0		//do nothing
			frame_idx += 1;
		#elif CONFIG_DATA_DUMP_MODE == 1
			frame_idx += 1;
		#endif/**/
	#endif

	#if   CONFIG_DATA_DUMP_MODE == 0		//do nothing
	#elif CONFIG_DATA_DUMP_MODE == 1		//write to config.h
		config_dat_wr_format_h(
					m,n,p,l,ncones,
					c,// data->n, 
					h,// data->m,
					b,// data->p,
					q,// data->nsoc
					Gjc,
					Gir,
					Gpr,
					G_NNZ_LEN,//G_nnz_len,		
					Ajc,
					Air,
					Apr,
					A_NNZ_LEN,//A_nnz_len,
					load_data_file
					);
	#elif (CONFIG_DATA_DUMP_MODE == 2) || (CONFIG_DATA_LOAD_MODE%2==1)		//write to the config.txt
			//ecos setup data save
		config_dat_batch_wr(
						&frame_idx,
						m,n,p,l,ncones,
						c,
						h,
						b,
						q,
					
						Gjc,
						Gir,
						Gpr,
						G_NNZ_LEN,//G_nnz_len,

						Ajc,
						Air,
						Apr,
						A_NNZ_LEN,//A_nnz_len,
						load_data_file
						);
	#endif

	/* set up data */
	#if DEBUG==1 
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_i_UD(&n     ,1           , 1 ,strcat(file_setup,"/din/n.txt"       ));												
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_i_UD(&m     ,1           , 1 ,strcat(file_setup,"/din/m.txt"       ));											
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_i_UD(&p     ,1           , 1 ,strcat(file_setup,"/din/p.txt"       ));											
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_i_UD(&l     ,1           , 1 ,strcat(file_setup,"/din/l.txt"       ));											
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_i_UD(&ncones,1           , 1 ,strcat(file_setup,"/din/ncones.txt"  ));
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_i_UD(q      ,ncones      , 1 ,strcat(file_setup,"/din/q.txt"       ));
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_UD(Gpr      ,G_NNZ_LEN   , 1 ,strcat(file_setup,"/din/Gpr.txt"     ));
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_i_UD(Gjc    ,(n+1)       , 1 ,strcat(file_setup,"/din/Gjc.txt"     ));
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_i_UD(Gir    ,G_NNZ_LEN   , 1 ,strcat(file_setup,"/din/Gir.txt"     ));
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_UD(Apr      ,A_NNZ_LEN   , 1 ,strcat(file_setup,"/din/Apr.txt"     ));
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_i_UD(Ajc    ,(n+1)       , 1 ,strcat(file_setup,"/din/Ajc.txt"     ));
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_i_UD(Air    ,G_NNZ_LEN   , 1 ,strcat(file_setup,"/din/Air.txt"     ));
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_UD(c        ,n           , 1 ,strcat(file_setup,"/din/c.txt"       ));
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_UD(h        ,m           , 1 ,strcat(file_setup,"/din/h.txt"       ));
		strcpy(file_setup,DATA_PATH);		dumpDenseMatrix_UD(b        ,p           , 1 ,strcat(file_setup,"/din/b.txt"       ));
	#endif
		mywork = ECOS_setup(n, m, p, l, ncones, q, 0, Gpr, Gjc, Gir, Apr, Ajc, Air, c, h, b);
 
    if( mywork != NULL ){
	
		/* solve */	
		exitflag = ECOS_solve(mywork);
    
		//#ifdef DEBUG==1
		strcpy(file_dout,DATA_PATH);		dumpDenseMatrix_UD(mywork->x        , n, 1, strcat(file_dout,"/dout/result_x.txt"	));	
		strcpy(file_dout,DATA_PATH);		dumpDenseMatrix_UD(mywork->y        , p, 1, strcat(file_dout,"/dout/result_y.txt"	));	
		strcpy(file_dout,DATA_PATH);		dumpDenseMatrix_UD(mywork->z        , m, 1, strcat(file_dout,"/dout/result_z.txt"	));	
		strcpy(file_dout,DATA_PATH);		dumpDenseMatrix_UD(mywork->s        , m, 1, strcat(file_dout,"/dout/result_s.txt"	));	
		strcpy(file_dout,DATA_PATH);		dumpDenseMatrix_UD(mywork->lambda   , m, 1, strcat(file_dout,"/dout/result_lambda.txt"));
		strcpy(file_dout,DATA_PATH);		dumpDenseMatrix_UD(&(mywork->kap)   , 1, 1, strcat(file_dout,"/dout/result_kap.txt"));
		strcpy(file_dout,DATA_PATH);		dumpDenseMatrix_UD(&(mywork->tau)   , 1, 1, strcat(file_dout,"/dout/result_tau.txt"));
		//#endif
    	/* test second solve
    	exitflag = ECOS_solve(mywork); */

		#if PROFILING > 1 && PRINTLEVEL > 2
			
			/* some statistics in milliseconds */
			tsolve = mywork->info->tsolve         * 1000;
			tsetup = mywork->info->tsetup         * 1000;
			ttotal = tsetup + tsolve;
	
			torder = mywork->info->torder         * 1000;
			tkktcreate = mywork->info->tkktcreate * 1000;
			ttranspose = mywork->info->ttranspose * 1000;
			tfactor = mywork->info->tfactor       * 1000;
			tkktsolve = mywork->info->tkktsolve   * 1000;
		
			printf("ECOS timings (all times in milliseconds):\n\n");
			printf("1. Setup: %7.3f (%4.1f%%)\n", tsetup,  tsetup / ttotal*100);
			printf("2. Solve: %7.3f (%4.1f%%)\n", tsolve,  tsolve / ttotal*100);
			printf("----------------------------------\n");
			printf(" Total solve time: %7.3f ms\n\n", ttotal);
	
			printf("Detailed timings in SETUP:\n");
			printf("Create transposes: %7.3f (%4.1f%%)\n", ttranspose, ttranspose / tsetup*100);
			printf("Create KKT Matrix: %7.3f (%4.1f%%)\n", tkktcreate, tkktcreate / tsetup*100);
			printf(" Compute ordering: %7.3f (%4.1f%%)\n", torder,         torder / tsetup*100);
			printf("            Other: %7.3f (%4.1f%%)\n", tsetup-torder-tkktcreate-ttranspose,         (tsetup-torder-tkktcreate-ttranspose) / tsetup*100);
			printf("\n");

			printf("Detailed timings in SOLVE:\n");
			printf("   Factorizations: %7.3f (%4.1f%% of tsolve / %4.1f%% of ttotal)\n", tfactor,     tfactor / tsolve*100, tfactor / ttotal*100);
			printf("       KKT solves: %7.3f (%4.1f%% of tsolve / %4.1f%% of ttotal)\n", tkktsolve, tkktsolve / tsolve*100, tfactor / ttotal*100);
			printf("            Other: %7.3f (%4.1f%% of tsolve / %4.1f%% of ttotal)\n", tsolve-tkktsolve-tfactor, (tsolve-tkktsolve-tfactor) / tsolve*100, (tsolve-tkktsolve-tfactor) / ttotal*100);
		#endif

		#if PROFILING == 3
	
			tsolve = mywork->info->tsolve         * 1000;
			tsetup = mywork->info->tsetup         * 1000;
			ttotal = tsetup + tsolve;
	
			torder = mywork->info->torder         * 1000;
			tkktcreate = mywork->info->tkktcreate * 1000;
			ttranspose = mywork->info->ttranspose * 1000;
			tfactor = mywork->info->tfactor       * 1000;
			tkktsolve = mywork->info->tkktsolve   * 1000;
	
			printf("ECOS timings (all times in milliseconds):\n\n");
			printf("1. Setup: %7.3f (%4.1f%%)\n", tsetup,  tsetup / ttotal*100);
			printf("2. Solve: %7.3f (%4.1f%%)\n", tsolve,  tsolve / ttotal*100);
			printf("----------------------------------\n");
			printf(" Total solve time: %7.3f ms\n\n", ttotal);

			kkt_factor_cnt	 = mywork->info->kkt_factor_cnt;
			ldl_lsolve2_cnt  = mywork->info->ldl_lsolve2_cnt;
			ldl_dsolve_cnt   = mywork->info->ldl_dsolve_cnt;
			ldl_ltsolve_cnt  = mywork->info->ldl_ltsolve_cnt;

			ldl_lsolve2_time = mywork->info->ldl_lsolve2_time* 1000;
			ldl_dsolve_time  = mywork->info->ldl_dsolve_time * 1000;
			ldl_ltsolve_time = mywork->info->ldl_ltsolve_time* 1000;
		
			printf("\n");
			printf("   KKT_factor_num : %5d Factorizations Time : %12.3f \n", kkt_factor_cnt,tfactor);
			printf("   ldl_ltsolve_num: %5d ldl_ltsolve_time    : %12.8f \n", ldl_dsolve_cnt,ldl_ltsolve_time);
			printf("   ldl_dsolve_num : %5d ldl_dsolve_time     : %12.8f \n", ldl_dsolve_cnt,ldl_dsolve_time);
			printf("   ldl_lsolve2_num: %5d ldl_lsolve2_time    : %12.8f \n", ldl_lsolve2_cnt,ldl_lsolve2_time);
			printf("\n");
			printf("\n");

			printf("==========================================================\n");
			printf("===========================SUMMARY========================\n");
			printf("==========================================================\n");
			printf("   ALL Process Time         : %12.8f \n", tfactor+ldl_ltsolve_time+ldl_dsolve_time+ldl_lsolve2_time );
			printf("   Factorizations Time      : %12.8f \n", tfactor);
			printf("   Func LDLx=b Time         : %12.8f \n", ldl_ltsolve_time+ldl_dsolve_time+ldl_lsolve2_time );
			printf("   One iter  Process Time   : %12.8f \n", (tfactor+ldl_ltsolve_time+ldl_dsolve_time+ldl_lsolve2_time)/kkt_factor_cnt );
			printf("==========================================================\n");
			printf("==========================================================\n");
			printf("\n");

			//write run information to run.log 
			fprintf(fid_log,"ECOS timings (all times in milliseconds):\n\n");
			fprintf(fid_log,"1. Setup: %7.3f (%4.1f%%)\n", tsetup,  tsetup / ttotal*100);
			fprintf(fid_log,"2. Solve: %7.3f (%4.1f%%)\n", tsolve,  tsolve / ttotal*100);
			fprintf(fid_log,"----------------------------------\n");
			fprintf(fid_log," Total solve time: %7.3f ms\n\n", ttotal);

			fprintf(fid_log,"\n");
			fprintf(fid_log,"   KKT_factor_num : %5d Factorizations Time  : %12.3f \n", kkt_factor_cnt,tfactor);
			fprintf(fid_log,"   ldl_ltsolve_num: %5d  ldl_ltsolve_time    : %12.8f \n", ldl_dsolve_cnt,ldl_ltsolve_time);
			fprintf(fid_log,"   ldl_dsolve_num : %5d  ldl_dsolve_time     : %12.8f \n", ldl_dsolve_cnt,ldl_dsolve_time);
			fprintf(fid_log,"   ldl_lsolve2_num: %5d  ldl_lsolve2_time    : %12.8f \n", ldl_lsolve2_cnt,ldl_lsolve2_time);
			fprintf(fid_log,"\n");
			fprintf(fid_log,"\n");

			fprintf(fid_log,"==========================================================\n");
			fprintf(fid_log,"===========================SUMMARY========================\n");
			fprintf(fid_log,"==========================================================\n");
			fprintf(fid_log,"   ALL Process      Time   : %12.8f \n", tfactor+ldl_ltsolve_time+ldl_dsolve_time+ldl_lsolve2_time );
			fprintf(fid_log,"   Factorizations   Time   : %12.8f \n", tfactor);
			fprintf(fid_log,"   Func LDLx=b      Time   : %12.8f \n", ldl_ltsolve_time+ldl_dsolve_time+ldl_lsolve2_time );
			fprintf(fid_log,"   One iter Process Time   : %12.8f \n", (tfactor+ldl_ltsolve_time+ldl_dsolve_time+ldl_lsolve2_time)/kkt_factor_cnt );
			fprintf(fid_log,"==========================================================\n");
			fprintf(fid_log,"==========================================================\n");
			fprintf(fid_log,"\n");
			printf(" mywork->x[0]=%28.18f,mywork->x[1]=%28.18f\n",mywork->x[0],mywork->x[1]);

			fclose(fid_log);
		#endif
    	/* clean up memory */
			ECOS_cleanup(mywork, 0);

		}
        
  }
    
    /* test version number
    ECOS_ver(ver);
    printf("This test has been run on ECOS version %s\n", ver);
     */
	
    /* explicitly truncate exit code */
	return (int)exitflag;
}

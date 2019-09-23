`timescale 1ns/1ps
module ps_master#( 
	parameter		string 	file_name  	= ("../data/db/fpga/config.dat"),
					integer Dem 		= 727,
					integer PS2PL_SEND_NUM = 82,
					integer PERIOD      = 10
				)
	(
	input				pl_clk,
	input				pl_resetn,
	
	output reg  [127:0]	m_axis_mm2s_tdata,
	output reg [15:0]	m_axis_mm2s_tkeep,
	output reg 			m_axis_mm2s_tlast,
	input	 			m_axis_mm2s_tready,
	output reg 			m_axis_mm2s_tvalid,
	
	input [127:0]		S_AXIS_tdata,
	input [15:0]		S_AXIS_tkeep,
	input 				S_AXIS_tlast,
	output 				S_AXIS_tready,
	input 				S_AXIS_tvalid
	);

	//protocol
	parameter CMDT_LDL_MatA_INIT    	= 16'h00A0;
	parameter CMDT_LDL_MatA_T_INIT  	= 16'h00A1;
	parameter CMDT_LDL_MatA_ITER    	= 16'h00A2;
	parameter CMDT_LDL_MatA_T_ITER  	= 16'h00A3;
	parameter CMDTR_LDL_MatA_INIT    	= 16'h01A0;
	parameter CMDTR_LDL_MatA_T_INIT  	= 16'h01A1;
	parameter CMDTR_LDL_MatA_ITER    	= 16'h01A2;
	parameter CMDTR_LDL_MatA_T_ITER  	= 16'h01A3;
	parameter CMDT_LDL_SIGN		  		= 16'h00A8;
	parameter CMDT_CAL_Vecb_INIT1   	= 16'h00B0;
	parameter CMDT_CAL_Vecb_INIT2   	= 16'h00B1;
	parameter CMDT_CAL_Vecb_INIT12   	= 16'h00B2;
	parameter CMDTR_CAL_Vecb_INIT1   	= 16'h01B0;
	parameter CMDTR_CAL_Vecb_INIT2   	= 16'h01B1;
	parameter CMDT_CAL_Vecb_ITER1   	= 16'h00B8;
	parameter CMDT_CAL_Vecb_ITER2   	= 16'h00B9;
	parameter CMDTR_CAL_Vecb_ITER1   	= 16'h01B8;
	parameter CMDTR_CAL_Vecb_ITER2   	= 16'h01B9;
	parameter CMDT_CAL_Vecb_ITER12  	= 16'h00BA;
	parameter CMDT_CAL_Vecb_ITER3   	= 16'h00BB;
	parameter CMDTR_CAL_Vecb_ITER3   	= 16'h01BB;
	parameter CMDT_INFO_MatL_COLNUM 	= 16'h00C0;
	parameter CMDT_INFO_MatL_T_COLNUM 	= 16'h00C1;
	parameter CMDT_MatLD_INIT 			= 16'h00C2;
	parameter CMDT_MatLD_T_INIT 		= 16'h00C3;
	parameter CMDT_MatLD_ITER 			= 16'h00C4;
	parameter CMDT_MatLD_T_ITER 		= 16'h00C5;
	parameter CMDR_DB_MatLD		    	= 16'h00D0;
	parameter CMDR_DB_FBX  				= 16'h00D1;
	parameter CMDR_DOUTX_INIT1    		= 16'h00F0;
	parameter CMDR_DOUTX_INIT2	  		= 16'h00F1;
	parameter CMDR_DOUTX_INIT12	  		= 16'h00F2;
	parameter CMDR_DOUTX_ITER1    		= 16'h00F8;
	parameter CMDR_DOUTX_ITER2	  		= 16'h00F9;
	parameter CMDR_DOUTX_ITER12	  		= 16'h00FA;
	parameter CMDR_DOUTX_ITER3	  		= 16'h00FB;
                   
	parameter Sign_Col_Row_Len 			= Dem*4; 
	parameter Vecbx_len	  	   			= Dem*8; 
	parameter VecPb_len	  	   			= Dem*8*2; 
 	parameter MAX_LEN					= 65536;
 	parameter PS2PL_SOP_LEN				= 16;
 	parameter num						= 16;
 	parameter DW						= 8;
 	parameter Tco						= 0.5;
	
   
	integer	  ps2pl_cnt;
	integer	  fid_load;
	integer   fid_r;
	integer   ps2pl_trans_cnt;
	
	reg 	  [DW*4-1 :0] 	buf_w32[0:1024-1];
	reg 	  [DW*8-1 :0] 	buf_w64[0:1024-1];
	reg 	  [DW*16-1 :0] 	buf_w128[0:MAX_LEN-1];
	reg		  [DW*16-1:0]   ps2pl_sop;
	reg		  [DW*16-1:0]   ps2pl_sop_tmp;
  	reg						ps2pl_start;
  	
  	reg		  [DW*4-1:0 ]	frame_id;		
  	reg		  [DW*4-1:0 ]	frame_len; 		
  	reg		  [DW*4-1:0 ]	frame_cnt; 		
  	reg		  [DW*4-1:0 ]	frame_iter_cnt;
  	integer					data_last_num 	;
  	integer					ps2pl_send_cnt; 
  	  	
  	reg 					trans_dat32_flag ;
  	reg 					trans_dat64_flag ;
  	reg						trans_datP64_flag;
  	reg 					trans_dat128_flag;
  	
  	reg 	  [DW*4-1 :0] 	last_dat_w32;
  	reg 	  [DW*8-1 :0] 	last_dat_w64;
  	reg 	  [DW*16-1 :0] 	last_dat_w128;
  	
  	wire	[31:0]			sign_0;
  	wire	[31:0]			sign_1;
  	wire	[31:0]			sign_2;
  	wire	[31:0]			sign_3;
  	wire	[31:0]			col_0;
  	wire	[31:0]			col_1;
  	wire	[31:0]			col_2;
  	wire	[31:0]			col_3;
  	wire	[31:0]			row_0;
  	wire	[31:0]			row_1;
  	wire	[31:0]			row_2;
  	wire	[31:0]			row_3;

  	wire	[31:0]			row_idx;
  	wire	[31:0]			col_idx;
  	wire	[63:0]			dat_val;
  
  	wire	[63:0] 			b_val0;
  	wire	[63:0] 			b_val1; 	
 	
  	initial
	begin	
		fid_load = $fopen(file_name,"rb");	
		
		if (!fid_load)
		begin
			$display("//----------------------------//");
			$display("%m  can't not open the data loadfile,%s",file_name);
			$display("//----------------------------//");
			$stop(1);
		end			
	end	
	
//trans_start	            
	initial
	begin
		ps2pl_start			<= 1'b0;  
		repeat(300) @(posedge pl_clk);
		ps2pl_start			<= 1'b1; 
	end
    
    initial
    begin
    	m_axis_mm2s_tdata = 128'h0;
		m_axis_mm2s_tkeep = 16'h0;
		m_axis_mm2s_tlast = 1'b0;
		m_axis_mm2s_tvalid= 1'b0;
		
		ps2pl_send_cnt	  = 0;
		trans_dat32_flag  = 1'b0;                                                 
		trans_dat64_flag  = 1'b0;                                                 
		trans_datP64_flag = 1'b0;
		trans_dat128_flag = 1'b0; 	
    end 
     
   	initial
   	begin
   		
   		@(posedge ps2pl_start)
   		for (ps2pl_send_cnt=1;ps2pl_send_cnt<(PS2PL_SEND_NUM+1);ps2pl_send_cnt=ps2pl_send_cnt+1)
   		begin
   			fid_r 			= $fread(ps2pl_sop_tmp,fid_load, ,1);
			cmd_sop_reverse(ps2pl_sop_tmp,ps2pl_sop);
			protocol_decode;
			trans_info_disp;
  			trans_send;	
  			trans_interval(1);			//µ¥Î»us
   		end
   		
   		#10000;
   		$stop();
   	end  
   	
   	
   	
   	
   	assign 			sign_0	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_LDL_SIGN )			 ? m_axis_mm2s_tdata[32*1-1:32*0]:32'h0;
  	assign 			sign_1	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_LDL_SIGN )			 ? m_axis_mm2s_tdata[32*2-1:32*1]:32'h0;
  	assign 			sign_2	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_LDL_SIGN )			 ? m_axis_mm2s_tdata[32*3-1:32*2]:32'h0;
  	assign 			sign_3	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_LDL_SIGN )			 ? m_axis_mm2s_tdata[32*4-1:32*3]:32'h0;
  	assign 			col_0	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_INFO_MatL_COLNUM )	 ? m_axis_mm2s_tdata[32*1-1:32*0]:32'h0;
  	assign 			col_1	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_INFO_MatL_COLNUM )	 ? m_axis_mm2s_tdata[32*2-1:32*1]:32'h0;
  	assign 			col_2	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_INFO_MatL_COLNUM )	 ? m_axis_mm2s_tdata[32*3-1:32*2]:32'h0;
  	assign 			col_3	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_INFO_MatL_COLNUM )	 ? m_axis_mm2s_tdata[32*4-1:32*3]:32'h0;
  	assign 			row_0	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_INFO_MatL_T_COLNUM )	 ? m_axis_mm2s_tdata[32*1-1:32*0]:32'h0;
  	assign 			row_1	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_INFO_MatL_T_COLNUM )	 ? m_axis_mm2s_tdata[32*2-1:32*1]:32'h0;
  	assign 			row_2	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_INFO_MatL_T_COLNUM )	 ? m_axis_mm2s_tdata[32*3-1:32*2]:32'h0;
  	assign 			row_3	= (trans_dat32_flag & frame_id[15:0] ==  CMDT_INFO_MatL_T_COLNUM )	 ? m_axis_mm2s_tdata[32*4-1:32*3]:32'h0;
  	assign 			row_idx	= (trans_dat128_flag ) ? m_axis_mm2s_tdata[32*1-1:32*0]:32'h0;
  	assign 			col_idx	= (trans_dat128_flag ) ? m_axis_mm2s_tdata[32*2-1:32*1]:32'h0;
  	assign 			dat_val	= (trans_dat128_flag ) ? m_axis_mm2s_tdata[32*4-1:32*2]:32'h0;
  	assign 			b_val0	= (trans_datP64_flag ) ? m_axis_mm2s_tdata[32*2-1:32*0]:32'h0;   
   	assign 			b_val1	= (trans_datP64_flag ) ? m_axis_mm2s_tdata[32*4-1:32*2]:32'h0;
	                                                                              
	task protocol_decode;                                                         
	                                                                              
		trans_dat32_flag  = 1'b0;                                                 
		trans_dat64_flag  = 1'b0;
		trans_datP64_flag = 1'b0;                                                
		trans_dat128_flag = 1'b0;                                                 
	                                                                              
		frame_id 		= ps2pl_sop[DW*4-1:DW*0];
		frame_len 		= ps2pl_sop[DW*8-1:DW*4];
		frame_cnt 		= ps2pl_sop[DW*12-1:DW*8];
		frame_iter_cnt 	= ps2pl_sop[DW*16-1:DW*12];
	
		data_last_num  = frame_len%16;
		if (data_last_num!=0)
			ps2pl_trans_cnt = frame_len/16+1;
		else
			ps2pl_trans_cnt = frame_len/16;
		
		if (frame_id[15:0] ==  CMDT_LDL_SIGN ||
			frame_id[15:0] ==  CMDT_INFO_MatL_COLNUM || 
			frame_id[15:0] ==  CMDT_INFO_MatL_T_COLNUM) begin
			trans_dat32_flag = 1'b1;
			fid_r = $fread(buf_w32,fid_load, ,(frame_len-16)/4);
			//$display("32bit sign_col_row data transfer");
		end
		else if( 
			frame_id[15:0] ==CMDT_CAL_Vecb_INIT1   	     ||
			frame_id[15:0] ==CMDT_CAL_Vecb_INIT2   	     ||
			frame_id[15:0] ==CMDTR_CAL_Vecb_INIT1   	 ||
			frame_id[15:0] ==CMDTR_CAL_Vecb_INIT2   	 ||
			frame_id[15:0] ==CMDT_CAL_Vecb_ITER1   	     ||
			frame_id[15:0] ==CMDT_CAL_Vecb_ITER2   	     ||
			frame_id[15:0] ==CMDTR_CAL_Vecb_ITER1   	 ||
			frame_id[15:0] ==CMDTR_CAL_Vecb_ITER2   	 ||
			frame_id[15:0] ==CMDT_CAL_Vecb_ITER3   	     ||
			frame_id[15:0] ==CMDTR_CAL_Vecb_ITER3 ) begin
			trans_dat64_flag = 1'b1;
			fid_r = $fread(buf_w64,fid_load, ,(frame_len-16)/8);
			//$display(" 64bit B    data transfer");
		end
		else if( 
			frame_id[15:0] ==CMDT_CAL_Vecb_INIT12   	 ||
			frame_id[15:0] ==CMDT_CAL_Vecb_ITER12) begin
			trans_datP64_flag = 1'b1;
			fid_r = $fread(buf_w128,fid_load, ,(frame_len-16)/16);
			//$display(" 128bit PB    data transfer");
		end
		else if(
			frame_id[15:0] ==  CMDT_LDL_MatA_INIT    	||
			frame_id[15:0] ==  CMDT_LDL_MatA_T_INIT  	|| 
			frame_id[15:0] ==  CMDT_LDL_MatA_ITER    	||
			frame_id[15:0] ==  CMDT_LDL_MatA_T_ITER  	||
			frame_id[15:0] ==  CMDTR_LDL_MatA_INIT    	||
			frame_id[15:0] ==  CMDTR_LDL_MatA_T_INIT  	||
			frame_id[15:0] ==  CMDTR_LDL_MatA_ITER    	||
			frame_id[15:0] ==  CMDTR_LDL_MatA_T_ITER  	||
			frame_id[15:0] ==  CMDT_MatLD_INIT 			||
			frame_id[15:0] ==  CMDT_MatLD_T_INIT 		||
			frame_id[15:0] ==  CMDT_MatLD_ITER 			||
			frame_id[15:0] ==  CMDT_MatLD_T_ITER 		)begin
			trans_dat128_flag = 1'b1;
			fid_r = $fread(buf_w128,fid_load, ,(frame_len-16)/16);  
			//$display("128bit MatA data transfer");
		end
		else begin
			$display("protocol frame id decode error");
			$display("protocol frame id is %04h",frame_id[15:0]);
			#100;
			$stop();
		end
   	endtask
  	
   	task trans_info_disp;                                                         
        //input disp_en;
        data_w32_reverse( buf_w32[(frame_len-16)/4-1]  ,last_dat_w32);   
        data_w64_reverse( buf_w64[(frame_len-16)/8-1]  ,last_dat_w64);   
        data_w128_reverse(buf_w128[(frame_len-16)/16-1],last_dat_w128);
        
        //if (disp_en == 1'b1)begin                                       
		if (frame_id[15:0] ==  CMDT_LDL_SIGN)					$display(" %04d bit32  		 sign 	     *** last data    is  %08d  " 					, ps2pl_send_cnt,last_dat_w32);
		else if	(frame_id[15:0] ==  CMDT_INFO_MatL_COLNUM )		$display(" %04d bit32  		 col 	     *** last data    is  %08d  " 					, ps2pl_send_cnt,last_dat_w32);
		else if	(frame_id[15:0] == CMDT_INFO_MatL_T_COLNUM	)	$display(" %04d bit32  		 row 	     *** last data    is  %08d  " 					, ps2pl_send_cnt,last_dat_w32);
		                                                                                                                              					
		else if	(frame_id[15:0] == CMDT_CAL_Vecb_INIT1  	)	$display(" %04d bit64  		 Init B1     *** last data    is  %016h " 					, ps2pl_send_cnt,last_dat_w64); 
		else if	(frame_id[15:0] == CMDT_CAL_Vecb_INIT2  	)   $display(" %04d bit64  		 Init B2     *** last data    is  %016h " 					, ps2pl_send_cnt,last_dat_w64); 
		else if	(frame_id[15:0] == CMDTR_CAL_Vecb_INIT1 	)	$display(" %04d bit64  Debug Init B1     *** last data    is  %016h " 					, ps2pl_send_cnt,last_dat_w64); 
		else if	(frame_id[15:0] == CMDTR_CAL_Vecb_INIT2 	)	$display(" %04d bit64  Debug Init B2     *** last data    is  %016h " 					, ps2pl_send_cnt,last_dat_w64); 
		else if	(frame_id[15:0] == CMDT_CAL_Vecb_ITER1  	)  	$display(" %04d bit64  		 Iter B1     *** last data    is  %016h " 					, ps2pl_send_cnt,last_dat_w64); 
		else if	(frame_id[15:0] == CMDT_CAL_Vecb_ITER2  	)	$display(" %04d bit64  		 Iter B2     *** last data    is  %016h " 					, ps2pl_send_cnt,last_dat_w64); 
		else if	(frame_id[15:0] == CMDTR_CAL_Vecb_ITER1 	)	$display(" %04d bit64  Debug Iter B1     *** last data    is  %016h " 					, ps2pl_send_cnt,last_dat_w64); 
		else if	(frame_id[15:0] == CMDTR_CAL_Vecb_ITER2 	)	$display(" %04d bit64  Debug Iter B2     *** last data    is  %016h " 					, ps2pl_send_cnt,last_dat_w64); 
		else if	(frame_id[15:0] == CMDT_CAL_Vecb_INIT12 	)	$display(" %04d bit64  		 Init PB     *** last data_x1 is  %016h data_x2 is  %016h " , ps2pl_send_cnt,last_dat_w128[63:0],last_dat_w128[127:64]); 
		else if	(frame_id[15:0] == CMDT_CAL_Vecb_ITER12 	)	$display(" %04d bit64  		 Iter PB     *** last data_x1 is  %016h data_x2 is  %016h " , ps2pl_send_cnt,last_dat_w128[63:0],last_dat_w128[127:64]); 
		else if	(frame_id[15:0] == CMDT_CAL_Vecb_ITER3  	)	$display(" %04d bit64  		 Iter B3     *** last data    is  %016h " 					, ps2pl_send_cnt,last_dat_w64); 
		else if	(frame_id[15:0] == CMDTR_CAL_Vecb_ITER3 	)	$display(" %04d bit64  Debug Iter B3     *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]);	 
		else if	(frame_id[15:0] == CMDT_LDL_MatA_INIT    	)   $display(" %04d bit128 		 Init MAT_A  *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]);
		else if	(frame_id[15:0] == CMDT_LDL_MatA_T_INIT  	)   $display(" %04d bit128 		 Init MAT_AT *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]); 
		else if	(frame_id[15:0] == CMDT_LDL_MatA_ITER    	)   $display(" %04d bit128 		 Iter MAT_T  *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]); 
		else if	(frame_id[15:0] == CMDT_LDL_MatA_T_ITER  	)   $display(" %04d bit128		 Iter MAT_AT *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]); 
		else if	(frame_id[15:0] == CMDTR_LDL_MatA_INIT    	)   $display(" %04d bit128 Debug Init MAT_A  *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]); 
		else if	(frame_id[15:0] == CMDTR_LDL_MatA_T_INIT  	)   $display(" %04d bit128 Debug Init MAT_AT *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]); 
		else if	(frame_id[15:0] == CMDTR_LDL_MatA_ITER    	)   $display(" %04d bit128 Debug Iter MAT_T  *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]); 
		else if	(frame_id[15:0] == CMDTR_LDL_MatA_T_ITER  	)   $display(" %04d bit128 Debug Iter MAT_AT *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]); 
		else if	(frame_id[15:0] == CMDT_MatLD_INIT 			)   $display(" %04d bit128 DUMP  Init MAT_L  *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]);
		else if	(frame_id[15:0] == CMDT_MatLD_T_INIT 		)   $display(" %04d bit128 DUMP	 Init MAT_LT *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]); 
		else if	(frame_id[15:0] == CMDT_MatLD_ITER 			)   $display(" %04d bit128 DUMP	 Iter MAT_L  *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]);
		else if	(frame_id[15:0] == CMDT_MatLD_T_ITER 		)   $display(" %04d bit128 DUMP	 Iter MAT_LT *** last data row    %08d col %08d dat %016h " , ps2pl_send_cnt,last_dat_w128[31:0],last_dat_w128[63:32],last_dat_w128[128:64]); 
		else begin                                                                                                                                                                
			$display("protocol frame id decode error");
		end
   	endtask
   	
   	
   	
   	
   	task trans_send;
   		integer i;
   		for (i=0;i<ps2pl_trans_cnt;i=i+1)
   		begin
   			@(posedge pl_clk)
   			if (m_axis_mm2s_tready == 1'b1) begin
	   			if (i==0) begin
	   					m_axis_mm2s_tdata = ps2pl_sop;
						m_axis_mm2s_tkeep = 16'hFFFF;
						m_axis_mm2s_tlast = 1'b0;
						m_axis_mm2s_tvalid= 1'b1;
					end
				else begin
					if (trans_dat32_flag == 1'b1) 
						if (i == ps2pl_trans_cnt-1 && data_last_num !=0)begin
							m_axis_mm2s_tdata[127:0] = 128'h0;
							case(data_last_num/4)
				            	1:  begin 
				              		data_w32_reverse(buf_w32[(i-1)*4],m_axis_mm2s_tdata[31:0]);
									m_axis_mm2s_tkeep = 16'h000F;
								end
								2:  begin 
				              		data_w64_reverse({buf_w32[(i-1)*4],buf_w32[(i-1)*4+1]},m_axis_mm2s_tdata[63:0]);
									m_axis_mm2s_tkeep = 16'h00FF;
								end
								3: begin  
				              		data_w96_reverse({buf_w32[(i-1)*4],buf_w32[(i-1)*4+1],buf_w32[(i-1)*4+2]},m_axis_mm2s_tdata[95:0]);
				              		//$display("the dat is %h",{buf_w32[(i-1)*4],buf_w32[(i-1)*4+1],buf_w32[(i-1)*4+2]});
									m_axis_mm2s_tkeep = 16'h0FFF;
								end
				              default: ;
				
				              endcase
				           	m_axis_mm2s_tlast = 1'b1;
							m_axis_mm2s_tvalid= 1'b1; 
						end 
						else if (i == ps2pl_trans_cnt-1 && data_last_num ==0) begin
							data_w128_reverse({buf_w32[(i-1)*4],buf_w32[(i-1)*4+1],buf_w32[(i-1)*4+2],buf_w32[(i-1)*4+3]},m_axis_mm2s_tdata);
							m_axis_mm2s_tkeep = 16'hFFFF;
							m_axis_mm2s_tlast = 1'b1;
							m_axis_mm2s_tvalid= 1'b1; 
						end
						else begin 
							data_w128_reverse({buf_w32[(i-1)*4],buf_w32[(i-1)*4+1],buf_w32[(i-1)*4+2],buf_w32[(i-1)*4+3]},m_axis_mm2s_tdata);
							m_axis_mm2s_tkeep = 16'hFFFF;
							m_axis_mm2s_tlast = 1'b0;
							m_axis_mm2s_tvalid= 1'b1; 
						end
					else if (trans_dat64_flag == 1'b1)
						if (i == ps2pl_trans_cnt-1 && data_last_num !=0)begin
							m_axis_mm2s_tdata[127:0] = 128'h0;
							case(data_last_num/8)
				            	1:  begin 
				              		data_w64_reverse(buf_w64[(i-1)*2],m_axis_mm2s_tdata[63:0]);
									m_axis_mm2s_tkeep = 16'h00FF;
								end
				              default: ;
				              endcase
				           	m_axis_mm2s_tlast = 1'b1;
							m_axis_mm2s_tvalid= 1'b1; 
						end 
						else if (i == ps2pl_trans_cnt-1 && data_last_num ==0) begin
							data_w128_reverse({buf_w64[(i-1)*2],buf_w64[(i-1)*2+1]},m_axis_mm2s_tdata);
							m_axis_mm2s_tkeep = 16'hFFFF;
							m_axis_mm2s_tlast = 1'b1;
							m_axis_mm2s_tvalid= 1'b1; 
						end
						else begin 
							data_w128_reverse({buf_w64[(i-1)*2],buf_w64[(i-1)*2+1]},m_axis_mm2s_tdata);
							m_axis_mm2s_tkeep = 16'hFFFF;
							m_axis_mm2s_tlast = 1'b0;
							m_axis_mm2s_tvalid= 1'b1; 
						end 
					else if (trans_dat128_flag == 1'b1 || trans_datP64_flag == 1'b1) 
						if (i == ps2pl_trans_cnt-1 ) begin
							data_w128_reverse({buf_w128[(i-1)]},m_axis_mm2s_tdata);
							m_axis_mm2s_tkeep = 16'hFFFF;
							m_axis_mm2s_tlast = 1'b1;
							m_axis_mm2s_tvalid= 1'b1; 
						end
						else begin
							data_w128_reverse({buf_w128[(i-1)]},m_axis_mm2s_tdata);
							m_axis_mm2s_tkeep = 16'hFFFF;
							m_axis_mm2s_tlast = 1'b0;
							m_axis_mm2s_tvalid= 1'b1;
						end
					else begin
							m_axis_mm2s_tdata = 128'h0;
							m_axis_mm2s_tkeep = 16'h0;
							m_axis_mm2s_tlast = 1'b0;
							m_axis_mm2s_tvalid= 1'b0;
						end
					end
				end		
				
			else begin
				i = i-1;
				m_axis_mm2s_tdata = m_axis_mm2s_tdata; 
				m_axis_mm2s_tkeep = m_axis_mm2s_tkeep; 
				m_axis_mm2s_tlast = m_axis_mm2s_tlast; 
				m_axis_mm2s_tvalid= m_axis_mm2s_tvalid;
			end
		end
		
		
		@(posedge pl_clk)
			m_axis_mm2s_tdata = 128'h0;
			m_axis_mm2s_tkeep = 16'h0;
			m_axis_mm2s_tlast = 1'b0;
			m_axis_mm2s_tvalid= 1'b0;
			
		trans_dat32_flag  = 1'b0;                                                 
		trans_dat64_flag  = 1'b0;                                                 
		trans_datP64_flag = 1'b0;
		trans_dat128_flag = 1'b0; 	
	endtask
   	
	task trans_interval;
   		input integer t_us;
   		
   		repeat(t_us*1000/PERIOD)
   		begin
   			@(posedge pl_clk);
   			//$display("=====aa====");
;   	end
   			
	endtask

   	      
	
	task cmd_sop_reverse;
   		input  [127:0] 	din;		
   		output [127:0]	dout;
		dout[7+8*(PS2PL_SOP_LEN-1-0 ):8*(PS2PL_SOP_LEN-0 -1)] = din[7+8*0 :8*0 ];
		dout[7+8*(PS2PL_SOP_LEN-1-1 ):8*(PS2PL_SOP_LEN-1 -1)] = din[7+8*1 :8*1 ];
		dout[7+8*(PS2PL_SOP_LEN-1-2 ):8*(PS2PL_SOP_LEN-2 -1)] = din[7+8*2 :8*2 ];
		dout[7+8*(PS2PL_SOP_LEN-1-3 ):8*(PS2PL_SOP_LEN-3 -1)] = din[7+8*3 :8*3 ];
		dout[7+8*(PS2PL_SOP_LEN-1-4 ):8*(PS2PL_SOP_LEN-4 -1)] = din[7+8*4 :8*4 ];
		dout[7+8*(PS2PL_SOP_LEN-1-5 ):8*(PS2PL_SOP_LEN-5 -1)] = din[7+8*5 :8*5 ];
		dout[7+8*(PS2PL_SOP_LEN-1-6 ):8*(PS2PL_SOP_LEN-6 -1)] = din[7+8*6 :8*6 ];
		dout[7+8*(PS2PL_SOP_LEN-1-7 ):8*(PS2PL_SOP_LEN-7 -1)] = din[7+8*7 :8*7 ];
 		dout[7+8*(PS2PL_SOP_LEN-1-8 ):8*(PS2PL_SOP_LEN-8 -1)] = din[7+8*8 :8*8 ];
		dout[7+8*(PS2PL_SOP_LEN-1-9 ):8*(PS2PL_SOP_LEN-9 -1)] = din[7+8*9 :8*9 ];
		dout[7+8*(PS2PL_SOP_LEN-1-10):8*(PS2PL_SOP_LEN-10-1)] = din[7+8*10:8*10];
		dout[7+8*(PS2PL_SOP_LEN-1-11):8*(PS2PL_SOP_LEN-11-1)] = din[7+8*11:8*11];
		dout[7+8*(PS2PL_SOP_LEN-1-12):8*(PS2PL_SOP_LEN-12-1)] = din[7+8*12:8*12];
		dout[7+8*(PS2PL_SOP_LEN-1-13):8*(PS2PL_SOP_LEN-13-1)] = din[7+8*13:8*13];
		dout[7+8*(PS2PL_SOP_LEN-1-14):8*(PS2PL_SOP_LEN-14-1)] = din[7+8*14:8*14];
		dout[7+8*(PS2PL_SOP_LEN-1-15):8*(PS2PL_SOP_LEN-15-1)] = din[7+8*15:8*15];  		
	endtask
	
	task data_w32_reverse;
   		input  [32-1:0] din;		
   		output [32-1:0]	dout;
		dout[7+8*(4-1-0 ):8*(4-0 -1)] = din[7+8*0 :8*0 ];
		dout[7+8*(4-1-1 ):8*(4-1 -1)] = din[7+8*1 :8*1 ];
		dout[7+8*(4-1-2 ):8*(4-2 -1)] = din[7+8*2 :8*2 ];
		dout[7+8*(4-1-3 ):8*(4-3 -1)] = din[7+8*3 :8*3 ]; 		
	endtask
	
	task data_w64_reverse;
   		input  [64-1:0] din;		
   		output [64-1:0]	dout;
		dout[7+8*(8-1-0 ):8*(8-0 -1)] = din[7+8*0 :8*0 ];
		dout[7+8*(8-1-1 ):8*(8-1 -1)] = din[7+8*1 :8*1 ];
		dout[7+8*(8-1-2 ):8*(8-2 -1)] = din[7+8*2 :8*2 ];
		dout[7+8*(8-1-3 ):8*(8-3 -1)] = din[7+8*3 :8*3 ];
		dout[7+8*(8-1-4 ):8*(8-4 -1)] = din[7+8*4 :8*4 ];
		dout[7+8*(8-1-5 ):8*(8-5 -1)] = din[7+8*5 :8*5 ];
		dout[7+8*(8-1-6 ):8*(8-6 -1)] = din[7+8*6 :8*6 ];
		dout[7+8*(8-1-7 ):8*(8-7 -1)] = din[7+8*7 :8*7 ];  		
	endtask
	
	task data_w96_reverse;
   		input  [96-1:0] din;		
   		output [96-1:0]	dout;
		dout[7+8*(12-1-0 ):8*(12-0 -1)] = din[7+8*0 :8*0 ];
		dout[7+8*(12-1-1 ):8*(12-1 -1)] = din[7+8*1 :8*1 ];
		dout[7+8*(12-1-2 ):8*(12-2 -1)] = din[7+8*2 :8*2 ];
		dout[7+8*(12-1-3 ):8*(12-3 -1)] = din[7+8*3 :8*3 ];
		dout[7+8*(12-1-4 ):8*(12-4 -1)] = din[7+8*4 :8*4 ];
		dout[7+8*(12-1-5 ):8*(12-5 -1)] = din[7+8*5 :8*5 ];
		dout[7+8*(12-1-6 ):8*(12-6 -1)] = din[7+8*6 :8*6 ];
		dout[7+8*(12-1-7 ):8*(12-7 -1)] = din[7+8*7 :8*7 ];
 		dout[7+8*(12-1-8 ):8*(12-8 -1)] = din[7+8*8 :8*8 ];
		dout[7+8*(12-1-9 ):8*(12-9 -1)] = din[7+8*9 :8*9 ];
		dout[7+8*(12-1-10):8*(12-10-1)] = din[7+8*10:8*10];
		dout[7+8*(12-1-11):8*(12-11-1)] = din[7+8*11:8*11];  		
	endtask
	
	task data_w128_reverse;
   		input  [127:0] 	din;		
   		output [127:0]	dout;
		dout[7+8*(PS2PL_SOP_LEN-1-0 ):8*(PS2PL_SOP_LEN-0 -1)] = din[7+8*0 :8*0 ];
		dout[7+8*(PS2PL_SOP_LEN-1-1 ):8*(PS2PL_SOP_LEN-1 -1)] = din[7+8*1 :8*1 ];
		dout[7+8*(PS2PL_SOP_LEN-1-2 ):8*(PS2PL_SOP_LEN-2 -1)] = din[7+8*2 :8*2 ];
		dout[7+8*(PS2PL_SOP_LEN-1-3 ):8*(PS2PL_SOP_LEN-3 -1)] = din[7+8*3 :8*3 ];
		dout[7+8*(PS2PL_SOP_LEN-1-4 ):8*(PS2PL_SOP_LEN-4 -1)] = din[7+8*4 :8*4 ];
		dout[7+8*(PS2PL_SOP_LEN-1-5 ):8*(PS2PL_SOP_LEN-5 -1)] = din[7+8*5 :8*5 ];
		dout[7+8*(PS2PL_SOP_LEN-1-6 ):8*(PS2PL_SOP_LEN-6 -1)] = din[7+8*6 :8*6 ];
		dout[7+8*(PS2PL_SOP_LEN-1-7 ):8*(PS2PL_SOP_LEN-7 -1)] = din[7+8*7 :8*7 ];
 		dout[7+8*(PS2PL_SOP_LEN-1-8 ):8*(PS2PL_SOP_LEN-8 -1)] = din[7+8*8 :8*8 ];
		dout[7+8*(PS2PL_SOP_LEN-1-9 ):8*(PS2PL_SOP_LEN-9 -1)] = din[7+8*9 :8*9 ];
		dout[7+8*(PS2PL_SOP_LEN-1-10):8*(PS2PL_SOP_LEN-10-1)] = din[7+8*10:8*10];
		dout[7+8*(PS2PL_SOP_LEN-1-11):8*(PS2PL_SOP_LEN-11-1)] = din[7+8*11:8*11];
		dout[7+8*(PS2PL_SOP_LEN-1-12):8*(PS2PL_SOP_LEN-12-1)] = din[7+8*12:8*12];
		dout[7+8*(PS2PL_SOP_LEN-1-13):8*(PS2PL_SOP_LEN-13-1)] = din[7+8*13:8*13];
		dout[7+8*(PS2PL_SOP_LEN-1-14):8*(PS2PL_SOP_LEN-14-1)] = din[7+8*14:8*14];
		dout[7+8*(PS2PL_SOP_LEN-1-15):8*(PS2PL_SOP_LEN-15-1)] = din[7+8*15:8*15];  		
	endtask
	
	
endmodule
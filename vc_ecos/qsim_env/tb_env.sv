`timescale 1ns/1ps
module tb_env();

	reg				pl_clk;
	reg				pl_resetn;
  
	wire [127:0]	S_AXIS_tdata;
	wire [15:0]		S_AXIS_tkeep;
	wire 			S_AXIS_tlast;
	reg 			S_AXIS_tready;
	wire 			S_AXIS_tvalid;
	
	reg [127:0]		m_axis_mm2s_tdata;
	reg [15:0]		m_axis_mm2s_tkeep;
	reg 			m_axis_mm2s_tlast;
	wire 			m_axis_mm2s_tready;
	reg 			m_axis_mm2s_tvalid;
	


	//parameter	kf_num = 1;
	parameter PS2PL_NUM 				= 82;
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
                   
	parameter Dem = 727;
	parameter Sign_Col_Row_Len 			= Dem*4; 
	parameter Vecbx_len	  	   			= Dem*8; 
	parameter VecPb_len	  	   			= Dem*8*2; 
 	parameter MAX_LEN					= 65536;
 	parameter PS2PL_SOP_LEN				= 16;
 	parameter num						= 16;
 	parameter DW						= 8;
 	parameter Tco						= 1;
	localparam file_name = {"../data/db/fpga/config.dat"};
   
	integer	  ps2pl_cnt;
	integer	  fid_load;
	integer   fid_r;
	
	reg 	  [DW - 1 :0] 	buffer[0 : MAX_LEN - 1];
	reg		  [DW*16-1:0]   ps2pl_sop_tmp;
  	reg		  [DW*16-1:0]   ps2pl_sop;
  	reg		  [DW*4-1 :0]	frame_len;
  	reg						ps2pl_start;
 	
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

//clk  
	initial
	begin
		pl_clk <= 1'b0;
		forever
		begin
			#5 pl_clk <= ~pl_clk;
		end
	end   
//rst	            
	initial
	begin
		pl_resetn			<= 1'b0;  
		repeat(300) @(posedge pl_clk);
		pl_resetn			<= 1'b1; 
	end
	
//trans_start	            
	initial
	begin
		ps2pl_start			<= 1'b0;  
		repeat(300) @(posedge pl_clk);
		ps2pl_start			<= 1'b1; 
	end
    
    genvar i;
	generate 
		for (i=0; i<num; i=i+1)
		begin
			 assign  ps2pl_sop[7+8*(num-1-i):8*(num-i-1)] = 0 ? ps2pl_sop_tmp[7+8*(num-1-i):8*(num-i-1)] : ps2pl_sop_tmp[7+8*i:8*i];
			 
		end	
	endgenerate
    
     
   	initial
   	begin
   		
   		@(posedge ps2pl_start)
   		
   		forever
   		begin
   			fid_r 		= $fread(ps2pl_sop_tmp,fid_load, ,PS2PL_SOP_LEN);
   			#Tco;
   			frame_len 	= ps2pl_sop[DW*8-1:DW*4];
   			@(posedge ps2pl_start)
   				;
   		end
   	end       

endmodule
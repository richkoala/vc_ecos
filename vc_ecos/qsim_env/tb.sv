`timescale 1ns/1ps
module tb_env();

	reg				pl_clk;
	reg				pl_resetn;
  
	wire  [127:0]	m_axis_mm2s_tdata;
	wire [15:0]		m_axis_mm2s_tkeep;
	wire 			m_axis_mm2s_tlast;
	reg	 			m_axis_mm2s_tready;
	wire 			m_axis_mm2s_tvalid;
	
	wire [127:0]	S_AXIS_tdata;
	wire [15:0]		S_AXIS_tkeep;
	wire 			S_AXIS_tlast;
	reg 			S_AXIS_tready;
	wire 			S_AXIS_tvalid;

	parameter PERIOD					= 10;
	parameter Dem 						= 727;
	parameter PS2PL_SEND_NUM			= 82;
	//localparam file_name 				= {"../data/db/fpga/config_s727.dat"};
	localparam file_name 				= {"./ecos_ps2pl_dat/config_s727.dat"};

	//parameter PS2PL_SEND_NUM			= 61;
	//localparam file_name 				= {"../data/db/fpga/config_p727.dat"};	




	
ps_master #( 
				.file_name  	(file_name),
				.Dem 			(Dem						 ),
				.PS2PL_SEND_NUM (PS2PL_SEND_NUM		 ),
				.PERIOD			(10							 )
				)
	ps_master_inst(
	.pl_clk				(pl_clk		),
	.pl_resetn			(pl_resetn	),
	
	.m_axis_mm2s_tdata	(m_axis_mm2s_tdata	),
	.m_axis_mm2s_tkeep	(m_axis_mm2s_tkeep	),
	.m_axis_mm2s_tlast	(m_axis_mm2s_tlast	),
	.m_axis_mm2s_tready	(m_axis_mm2s_tready	),
	.m_axis_mm2s_tvalid	(m_axis_mm2s_tvalid	),
	
	.S_AXIS_tdata		(S_AXIS_tdata		),
	.S_AXIS_tkeep		(S_AXIS_tkeep		),
	.S_AXIS_tlast		(S_AXIS_tlast		),
	.S_AXIS_tready		(S_AXIS_tready		),
	.S_AXIS_tvalid	    (S_AXIS_tvalid		)
	);	

//clk  
	initial
	begin
		pl_clk <= 1'b0;
		forever
		begin
			#(PERIOD/2) pl_clk <= ~pl_clk;
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
/*	            
	initial
	begin
		ps2pl_start			<= 1'b0;  
		repeat(300) @(posedge pl_clk);
		ps2pl_start			<= 1'b1; 
	end
     */
   	
   	
   	initial
   	begin
   		@(posedge pl_clk);m_axis_mm2s_tready <= 1'b1;
   		#10e3;@(posedge pl_clk);#2;m_axis_mm2s_tready <= 1'b0;
   		#1e3 ;@(posedge pl_clk);#2;m_axis_mm2s_tready <= 1'b1;
   		#10e3;@(posedge pl_clk);#2;m_axis_mm2s_tready <= 1'b0;
   		#1e3 ;@(posedge pl_clk);#2;m_axis_mm2s_tready <= 1'b1;
   	end

	
	
endmodule
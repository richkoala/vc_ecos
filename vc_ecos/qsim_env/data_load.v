`timescale 1 ns/ 1 ps
module data_load #( parameter DW 		= 32,
					parameter file_name = ("data_load.dat"),
					parameter INV		= 0,
					parameter SOP		= 4						//帧头无效数据个数
					
				)
				(
				input				clk,
				input				rst,
				input 				den,
				output [DW - 1: 0]	dout
				);

localparam 	num			= DW/8;
localparam 	buffer_size	= 1;
localparam  Tco 		= 0;

integer 					fid_load,fid_r;
integer 					j;

reg 	[DW - 1:0] 	buffer[0 : buffer_size - 1];
reg		[DW - 1:0] 	cnt = 1'b0;
reg							int_flag = 1'b0;

wire	[DW - 1: 0] dout_temp;

reg 	[DW - 1:0] 	rg_tmp;


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
	
	$fseek(fid_load,SOP*num,1);
	fid_r = $fread(buffer,fid_load, ,buffer_size);				
end	

always @(posedge clk)
begin
	if (den)
		begin
			if (cnt < buffer_size -1)
				begin
					cnt <= cnt + 1;	
				end
			else
				begin
					cnt <= 0;
					fid_r = $fread(buffer,fid_load, ,buffer_size);	
				end
			int_flag <= 1'b1;	
		end
end	

		
genvar i;
generate 
	for (i=0; i<num; i=i+1)
	begin
		 assign #Tco dout_temp[7+8*(num-1-i):8*(num-i-1)] = !INV ? buffer[cnt][7+8*(num-1-i):8*(num-i-1)] : buffer[cnt][7+8*i:8*i];
	end	
endgenerate

//assign dout = (fid_r != 0) ? dout_temp : {{DW}{1'bZ}};
assign dout = (fid_r != 0) ? dout_temp : 32'h80808080;

endmodule

`timescale 1ns / 1ps

// DATA MEMORY INTERFACE SIGNALS
module d_mem(
	input clk,    
	input [31:0] dmem_data_in,
    output [31:0] dmem_data_out,
    input [31:0] dmem_addr,
    input dmem_wen,
    input dmem_ren
);

// DATA MEMORY
parameter DMEM_DEPTH = 256;
reg [31:0] dmem [0:DMEM_DEPTH-1];
    
wire [6:0] dmem_addr_small;
assign dmem_addr_small = dmem_addr[8:2];
    
integer i;
initial $readmemh("../sim/d_mem_init.txt", dmem);
    
always @(posedge clk)
	if(dmem_wen) 
		dmem[dmem_addr_small] <= dmem_data_in;
                 
assign dmem_data_out = dmem[dmem_addr_small]; 


endmodule

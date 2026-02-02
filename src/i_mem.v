`timescale 1ns / 1ps


// INSTRUCTION MEMORY INTERFACE SIGNALS
module i_mem (
	input clk,
    output [31:0] imem_data, 
    input [31:0] imem_addr,
    input imem_en
   );


// INSTRUCTION MEMORY
    reg [31:0] imem [0:255];
    initial $readmemh("../sim/main.txt", imem); 
    wire [5:0] imem_addr_small;
    assign imem_addr_small = imem_addr[7:2];
    assign imem_data = imem[imem_addr_small]; 

endmodule

`timescale 1ns / 1ps


`define REG_INBUFFER        4'b0000 
`define REG_WBUFFER_0       4'b0001 
`define REG_OUT             4'b0010 // read only

module apb_psi_hwacc
#(
    parameter APB_ADDR_WIDTH = 12  //APB slaves are 4KB by default
    )
    (
    input  logic                      HCLK,
    input  logic                      HRESET,
    input  logic [APB_ADDR_WIDTH-1:0] PADDR,
    input  logic               [31:0] PWDATA,
    input  logic                      PWRITE,
    input  logic                      PSEL,
    input  logic                      PENABLE,
    output logic               [31:0] PRDATA,
    output logic                      PREADY,
    output logic                      PSLVERR
    );

    logic [3:0]         s_apb_addr;
    logic [0:2] [31:0]  regs_q, regs_n;
    logic [7:0]         w0, w1, w2;

    assign s_apb_addr = PADDR[5:2];

    // register write from apb
    always_comb
        begin
          regs_n[`REG_INBUFFER] = regs_q[`REG_INBUFFER];
          regs_n[`REG_WBUFFER_0] = regs_q[`REG_WBUFFER_0];
 
         if (PSEL && PENABLE && PWRITE)
            begin
                // register update logic
                case (s_apb_addr)
                `REG_INBUFFER: begin
                  regs_n[`REG_INBUFFER][7:0]   = PWDATA[7:0];
                  regs_n[`REG_INBUFFER][15:8]  = regs_q[`REG_INBUFFER][7:0];
                  regs_n[`REG_INBUFFER][23:16] = regs_q[`REG_INBUFFER][15:8];
                end
                `REG_WBUFFER_0:
                  regs_n[`REG_WBUFFER_0]  = PWDATA;
                endcase
            end
    end

    // register read from apb
    always_comb
    begin
      if (PSEL && PENABLE)
         begin
              case(s_apb_addr)
              `REG_OUT:
                PRDATA    = regs_q[`REG_OUT];
              endcase
       end
    end

    // perform multiply and accumulate and assign new output value
    assign w0 = regs_q[`REG_WBUFFER_0][7:0];
    assign w1 = regs_q[`REG_WBUFFER_0][15:8];
    assign w2 = regs_q[`REG_WBUFFER_0][23:16];

    assign regs_n[`REG_OUT]  = w0 * regs_q[`REG_INBUFFER][23:16] + w1 * regs_q[`REG_INBUFFER][15:8] + w2 * regs_q[`REG_INBUFFER][7:0];

    always_ff @(posedge HCLK, negedge HRESET) 
    begin
        if(HRESET) 
        begin
          //reset weight buffer and input buffer
          regs_q[`REG_INBUFFER]   <= 0;
          regs_q[`REG_WBUFFER_0]  <= 0;
        end
        else
          //assign updated values to registers
          regs_q <= regs_n; 
    end
    
    assign PREADY = 1'b1;
    assign PSLVERR = 1'b0;

endmodule

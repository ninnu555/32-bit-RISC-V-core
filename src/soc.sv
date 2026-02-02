`timescale 1ns / 1ps

module soc (
input clk, rst,
inout wire i2c_sda,
output wire i2c_scl,
input  logic               [31:0] gpio_in,
output logic               [31:0] gpio_in_sync,
output logic               [31:0] gpio_out,
output logic               [31:0] gpio_dir
);


wire [31:0] imem_data;
wire [31:0] imem_addr;
wire imem_en;
    
wire [31:0] dmem_data_in; 
wire  [31:0] dmem_data_out;
wire [31:0] dmem_addr;
wire dmem_wen;
wire dmem_ren;
wire dmem_en;

parameter NB_MASTER = 5;
parameter APB_DATA_WIDTH = 32;
parameter APB_ADDR_WIDTH = 32;

// SLAVE PORT
logic                                     penable_i;
logic                                     pwrite_i;
logic [31:0]                              paddr_i;
logic [31:0]                              pwdata_i;
logic [31:0]                              prdata_o;
logic                                     pready_o;
logic                                     pslverr_o;

// MASTER PORTS
logic [NB_MASTER-1:0]                     penable_o;
logic [NB_MASTER-1:0]                     pwrite_o;
logic [NB_MASTER-1:0][31:0]               paddr_o;
logic [NB_MASTER-1:0]                     psel_o;
logic [NB_MASTER-1:0][31:0]               pwdata_o;
logic [NB_MASTER-1:0][31:0]               prdata_i;
logic [NB_MASTER-1:0]                     pready_i;
logic [NB_MASTER-1:0]                     pslverr_i;

// CONFIGURATION PORT
logic [NB_MASTER-1:0][APB_ADDR_WIDTH-1:0] START_ADDR_i;
logic [NB_MASTER-1:0][APB_ADDR_WIDTH-1:0] END_ADDR_i;

////////////////////////////////////////////////////////
// Localparam definitions for memory-mapped addresses //
////////////////////////////////////////////////////////

// dmem
localparam START_ADDR_DMEM  = 32'h00000600; 
localparam END_ADDR_DMEM    = 32'h000006FF; 
// gpio
localparam START_ADDR_GPIO  = 32'h00000400; 
localparam END_ADDR_GPIO    = 32'h0000043F;
// i2c
localparam START_ADDR_I2C   = 32'h00000440; 
localparam END_ADDR_I2C     = 32'h0000047F; 
// timer
localparam START_ADDR_TIMER = 32'h00000480; 
localparam END_ADDR_TIMER   = 32'h000004BF;
// acc
localparam START_ADDR_ACC   = 32'h000004C0; 
localparam END_ADDR_ACC     = 32'h000004FF;


assign START_ADDR_i[0] = START_ADDR_DMEM;
assign END_ADDR_i[0]   = END_ADDR_DMEM;

assign START_ADDR_i[1] = START_ADDR_I2C;
assign END_ADDR_i[1]   = END_ADDR_I2C;

assign START_ADDR_i[2] = START_ADDR_ACC;
assign END_ADDR_i[2]   = END_ADDR_ACC;

assign START_ADDR_i[3] = START_ADDR_GPIO;
assign END_ADDR_i[3]   = END_ADDR_GPIO;

assign START_ADDR_i[4] = START_ADDR_TIMER;
assign END_ADDR_i[4]   = END_ADDR_TIMER;



///////////////
// RISC-V uP //
///////////////

simple_rv rv (
    .clk            (   clk             ),
    .rst            (   rst             ),
    .imem_data      (   imem_data       ),
    .imem_addr      (   imem_addr       ),
    .imem_en        (   imem_en         ),
   
    .dmem_data_in   (   dmem_data_in    ),
    .dmem_data_out  (   dmem_data_out   ),
    .dmem_addr      (   dmem_addr       ),
    .dmem_wen       (   dmem_wen        ),
    .dmem_ren       (   dmem_ren        )
    );

////////////////////////
// INSTRUCTION MEMORY //
////////////////////////

i_mem imem(
	.clk             ( clk           ),
    .imem_data       ( imem_data     ), 
    .imem_addr       ( imem_addr     ),
    .imem_en         ( imem_en       ) 
   );

////////////////////////////////////
// RISC-V and APB NODE CONNECTION //
//////// to be completed... ////////
////////////////////////////////////

assign dmem_en          = dmem_ren || dmem_wen;
assign penable_i        = dmem_en;
assign pwrite_i         = dmem_wen;
assign paddr_i          = dmem_addr;
assign pwdata_i         = dmem_data_in;
assign dmem_data_out    = prdata_o;
assign pready_i[0]      = 1'b1;
assign pslverr_i[0]     = 1'b0;

/////////////////////////////////////////
// APB NODE and DATA MEMORY CONNECTION //
/////////////////////////////////////////

wire apb_dmem_wen, apb_dmem_ren; 
assign apb_dmem_wen = psel_o[0] & pwrite_o[0] & penable_o[0];
assign apb_dmem_ren = psel_o[0] & ~pwrite_o[0] & penable_o[0];

d_mem dmem( 
    .clk             (  clk               ),
	.dmem_data_in    (  pwdata_o[0]       ),
    .dmem_data_out   (  prdata_i[0]       ),
    .dmem_addr       (  paddr_o[0]        ),
    .dmem_wen        (  apb_dmem_wen      ),
    .dmem_ren        (  apb_dmem_ren      )
);  

// APB NODE    
// module APB NODE goes here...
apb_node
apb_node_i
(
    .penable_i      (   penable_i      ),
    .pwrite_i       (   pwrite_i       ),
    .paddr_i        (   paddr_i        ),
    .pwdata_i       (   pwdata_i       ),
    .prdata_o       (   prdata_o       ),
    .pready_o       (   pready_o       ),
    .pslverr_o      (   pslverr_o      ),

    .penable_o      (   penable_o      ),
    .pwrite_o       (   pwrite_o       ),
    .paddr_o        (   paddr_o        ),
    .psel_o         (   psel_o         ),
    .pwdata_o       (   pwdata_o       ),
    .prdata_i       (   prdata_i       ),
    .pready_i       (   pready_i       ),
    .pslverr_i      (   pslverr_i      ),

    .START_ADDR_i   (   START_ADDR_i   ),
    .END_ADDR_i     (   END_ADDR_i     )
);


// I2C     
// module I2C goes here...
apb_psi_i2c
apb_psi_i2c_i
(
    .HCLK       ( clk            ),    
    .HRESET     ( rst            ),        
    .PADDR      ( paddr_o[1]     ),    
    .PWDATA     ( pwdata_o[1]    ),    
    .PWRITE     ( pwrite_o[1]    ),    
    .PSEL       ( psel_o[1]      ),    
    .PENABLE    ( penable_o[1]   ),        
    .PRDATA     ( prdata_i[1]    ),    
    .PREADY     ( pready_i[1]    ),    
    .PSLVERR    ( pslverr_i[1]   ),
 
    .sda        ( i2c_sda        ),    
    .scl        ( i2c_scl        )
);

// HW ACCELERATOR
// module HW ACCELERATOR goes here...
apb_psi_hwacc
apb_psi_hwacc_i
(
   .HCLK            ( clk           ),
   .HRESET          ( rst           ),
   .PADDR           ( paddr_o[2]    ),
   .PWDATA          ( pwdata_o[2]   ),
   .PWRITE          ( pwrite_o[2]   ),
   .PSEL            ( psel_o[2]     ),
   .PENABLE         ( penable_o[2]  ),
   .PRDATA          ( prdata_i[2]   ),
   .PREADY          ( pready_i[2]   ),
   .PSLVERR         ( pslverr_i[2]  )
);

// GPIO
// module GPIO goes here...
apb_psi_gpio
apb_psi_gpio_i
(
   .HCLK           ( clk           ),
   .HRESET         ( rst           ),
   .PADDR          ( paddr_o[3]    ),
   .PWDATA         ( pwdata_o[3]   ),
   .PWRITE         ( pwrite_o[3]   ),
   .PSEL           ( psel_o[3]     ),
   .PENABLE        ( penable_o[3]  ),
   .PRDATA         ( prdata_i[3]   ),
   .PREADY         ( pready_i[3]   ),
   .PSLVERR        ( pslverr_i[3]  ),

   .gpio_in        ( gpio_in       ),
   .gpio_in_sync   ( gpio_in_sync  ),
   .gpio_out       ( gpio_out      ),
   .gpio_dir       ( gpio_dir      )
   // .gpio_padcfg    ( gpio_padcfg   ),
   // .power_event    ( power_event   ),
   // .interrupt      ( interrupt     )
);

// TIMER
// module TIMER goes here...
apb_psi_timer
apb_psi_timer_i
(
    .HCLK           ( clk           ),
    .HRESET         ( rst           ),
    .PADDR          ( paddr_o[4]    ),
    .PWDATA         ( pwdata_o[4]   ),
    .PWRITE         ( pwrite_o[4]   ),
    .PSEL           ( psel_o[4]     ),
    .PENABLE        ( penable_o[4]  ),
    .PRDATA         ( prdata_i[4]   ),
    .PREADY         ( pready_i[4]   ),
    .PSLVERR        ( pslverr_i[4]  )
);

endmodule

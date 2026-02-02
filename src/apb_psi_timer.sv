`define REG_TIMER                 2'b00
`define REG_TIMER_CTRL            2'b01
`define REG_CMP                   2'b10

`define P_1                   3'b000    // PS2,Ps1,PS0
`define P_2                   3'b001
`define P_4                   3'b010
`define P_8                   3'b100

`define MAX_COUNT             32'hFFFFFFFF


module apb_psi_timer
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
    output logic                      PSLVERR,

    // output logic                      cmp_interrupt,
    // output logic                      of_interrupt
    output logic                [1:0] irq_o_timer
);

    // APB register interface
    logic [1:0]       register_adr;
    assign register_adr = PADDR[3:2];
    // APB logic: we are always ready to capture the data into our regs
    // not supporting transfare failure
    assign PREADY  = 1'b1;
    assign PSLVERR = 1'b0;


    // registers
    logic [0:2] [31:0]  regs_q, regs_n;
    logic [31:0] cycle_counter_n, cycle_counter_q;
    logic [2:0] prescaler_int;                      // int represents code in control register
    // logic [2:0] prescaler_target;                   // target contains counter target value (2,4,8 or 1 default)
    logic timer_enable;

    logic en;
    logic [3:0] count;
    // logic [1:0] irq_o_timer;

    //irq logic: check if interrupt condition is met (overflow or compare match)
    always_comb
    begin
        irq_o_timer = 2'b0;

        if ((regs_q[`REG_TIMER] == regs_q[`REG_CMP] - 1) && timer_enable)
            irq_o_timer[0] = 1'b1;

        if (regs_q[`REG_TIMER] == `MAX_COUNT && cycle_counter_n == 0 && timer_enable)  // con questa condizione siamo certi che siamo in overflow
            irq_o_timer[1] = 1'b1;

    end

    assign timer_enable     = regs_q[`REG_TIMER_CTRL][0];       // based on current value of control register 
    assign prescaler_int    = regs_q[`REG_TIMER_CTRL][4:2];    // based on current value of control register 

    // define clk_counter_target based on prescale value
    // ho preferito definire di quanto shiftare il valore a fino a cui contare messo nel COMPARE_REG
    // always_comb
    // begin
    //   case(prescaler_int)
    //     `P_1:
    //         prescaler_target = 0;   // no scaling
    //     `P_2:
    //         prescaler_target = 1;   // scaling by 2
    //     `P_4:
    //         prescaler_target = 2;   // scaling by 4
    //     `P_8:
    //         prescaler_target = 3;   // scaling by 8

    //     default: 
    //         prescaler_target = 1;
    //   endcase
    // end

    always_ff @(posedge HCLK, negedge HRESET) begin
        if (HRESET) begin
            count <= 0;
            en    <= 0;
        end
        else 
        begin
        if (timer_enable) 
        begin
            // $display("prescaler_int = %b", prescaler_int);
            // $display("reg_ctrl = %b", regs_q[`REG_TIMER_CTRL]);
            case(prescaler_int)
        `P_1: begin
            en <= 1;
            count <= 0;
        end
        `P_2: begin
            if (count == 1) begin
                count <= 0;    
                en <= 1;
            end else begin
                count <= count + 1;
                en <= 0;
            end
        end
        `P_4: begin
            if (count == 3) begin
                count <= 0;    
                en <= 1;
            end else begin
                count <= count + 1;
                en <= 0;
            end
        end
        `P_8: begin
            if (count == 7) begin
                count <= 0;    
                en <= 1;
            end else begin
                count <= count + 1;
                en <= 0;
            end
        end

        default: begin
            count <= 0;
            en <= 0;
        end
        endcase
    end
    end
end
    

    // register write logic
    always_comb
    begin
        regs_n = regs_q;
        cycle_counter_n = cycle_counter_q;
        regs_n[`REG_TIMER] = cycle_counter_q;

        //increment counter of clock cycles
        if (en && timer_enable)
        begin
            if (cycle_counter_n == regs_n[`REG_CMP] - 1) begin
                cycle_counter_n = 0;
            end
            else 
            begin
                cycle_counter_n = cycle_counter_n + 1;
            end
        end

        // reset timer after cmp or overflow

        if (irq_o_timer != 0) begin
            regs_n          = 0;
            cycle_counter_n = 0;
        end
            
        // check condition to increment counter register based on enable bit, prescale and clock counter

        // if ()

        // reset prescaler cycle counter when target is met

        // if (cycle_counter_q == regs_q[`REG_CMP])


        // written from APB bus - gets priority

        if (PSEL && PENABLE && PWRITE)
        begin
            case (register_adr)
               // when compare register is written, set to 0 counter register
                `REG_CMP: begin
                    regs_n[`REG_CMP] = PWDATA;
                    regs_n[`REG_TIMER] = 32'b0;
                end
                `REG_TIMER:
                    regs_n[`REG_TIMER] = PWDATA;    // non penso sia utile questo, a meno che la CPU non voglia cambiare il valore del timer
                `REG_TIMER_CTRL:
                    regs_n[`REG_TIMER_CTRL] = PWDATA;

            endcase
        end
    end

    // APB register read logic
    always_comb
    begin
        PRDATA = 'b0;
        if (PSEL && PENABLE && !PWRITE)
        begin
            case (register_adr)
                `REG_CMP:
                    PRDATA = regs_q[`REG_CMP];
                `REG_TIMER:
                    PRDATA = regs_q[`REG_TIMER];
                `REG_TIMER_CTRL: 
                    PRDATA = regs_q[`REG_TIMER_CTRL];
            endcase

        end
    end
    // synchronouse part
    always_ff @(posedge HCLK, negedge HRESET)
    begin
        if(HRESET)
        begin
            regs_q          <= '{default: 32'b0};
            cycle_counter_q <= 32'b0;
        end
        else
        begin
            regs_q          <= regs_n;
            cycle_counter_q <= cycle_counter_n;
        end
    end

endmodule

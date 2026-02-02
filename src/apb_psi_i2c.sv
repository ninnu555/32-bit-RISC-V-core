// Copyright 2017 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

module apb_psi_i2c
  #(
     parameter APB_ADDR_WIDTH = 7  //APB slaves are 4KB by default
   )
   (
     // interfaccia con l'APB
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
     // interfaccia con la periferica esterna I2C
     inout  logic                      sda,
     output logic                      scl
   );
  //    localparam S_ADDRESS = 7'b0000100;

  localparam IDLE             = 0;
  localparam START            = 1;
  localparam SEND_ADDR        = 2;
  localparam WAIT_ACK         = 3;
  localparam READ_DATA        = 4;
  localparam READ_ACK         = 5;
  localparam WRITE_DATA       = 6;
  localparam WRITE_ACK        = 7;
  localparam STOP             = 8;

  logic       [3:0]   state;
  logic               i2c_clk;
  logic       [1:0]   count_i2c_clk;
  logic               write_enable;
  logic       [3:0]   counter;
  logic       [7:0]   addr;
  logic       [7:0]   w_data;
  logic       [7:0]   r_data;
  logic               en_scl;
  logic               valid;
  logic               sda_out;
  logic               aux_start;
  logic               stop;
  logic       [3:0]   delay_PS_PE;

  logic       [15:0]  r_i2c_cpr;
  logic       [15:0]  r_i2c_ctrl;
  logic       [15:0]  r_i2c_tx;
  logic       [15:0]  r_i2c_rx;
  logic       [15:0]  r_i2c_cmd;
  logic       [15:0]  r_i2c_status;

  logic [2:0] s_apb_addr;

  logic      [APB_ADDR_WIDTH-1:0] paddr;
  logic      [7:0] pwdata;
  logic       pwrite;
  logic       psel;
  logic       penable;

  logic       start_valid;

  localparam REG_RX        = 3'b000;
  localparam SLAVE_ADDR   = 3'b001;

    // controller logic
  assign PREADY  = 1'b1;
  assign PSLVERR = 1'b0;


///////////////////////////////////////////////
// SDA logic

  assign sda = (write_enable == 1) ? sda_out : 'bz;

//////////////////////////////////////////////////////////////////////////////////
// Costrutto per propagare lo start e hittare il fronte di clock positivo di i2clk

  always @(posedge HCLK, negedge HRESET)
  begin
    if (HRESET)
    begin
      delay_PS_PE <= 0;
      aux_start   <= 0;
      // PREADY      <= 1;
    end
    else if (((PSEL && PENABLE) || (PSEL && PENABLE && PWRITE)) && (PADDR[4:2] == SLAVE_ADDR))
    begin
      delay_PS_PE     <= 8;
      aux_start       <= 1;
      // PREADY      <= 0;
    end
    else if (delay_PS_PE > 0)
    begin
      delay_PS_PE <= delay_PS_PE - 1;
      aux_start       <= 1;
      // PREADY      <= 0;
    end
    else
    begin
      aux_start       <= 0;
      // PREADY      <= 1;
    end
  end

  assign start = aux_start;

///////////////////////////////////////////////////////7
// Aggiornamento registro ricezione

  always_ff @(posedge HCLK, negedge HRESET)
  begin
    if(HRESET)
    begin
      r_i2c_rx          <= {1'b1, 15'b0};
    end
    else
      if ((s_apb_addr == SLAVE_ADDR) && psel && penable && ~pwrite) // Se c'è una nuova lettura, va messo pwrite negato, sennò aggiornerebbe il registro RX anche durante una scrittura
        begin
          r_i2c_rx      <= {valid, 7'b0, r_data};
          end
          else
      if (psel & penable & pwrite)
      begin
        r_i2c_rx        <= {start_valid, r_i2c_rx[14:0]};
      end
      else
            r_i2c_rx        <= {start_valid, r_i2c_rx[14:0]};
      end

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Aggiornamento di PRDATA in caso di: lettura del registro RX, stato IDLE o stato durante una transazione 

  always_comb
  begin
    if (~psel && ~penable) begin
        PRDATA  = {16'b0, 1'b1, 15'b0};   // IDLING
      if (PADDR[4:2] == REG_RX)
        PRDATA  = {16'b0, r_i2c_rx};      // Read of RX by controller while idling
    end
    else 
      PRDATA  = 0;                        // Set to zero during transaction
  end

/////////////////////////////////////////////////
// i2c_clock and scl generation

  always @(posedge HCLK, negedge HRESET)
  begin
    if (HRESET)
    begin
      i2c_clk         <= 0;
      count_i2c_clk   <= 0;
    end
    else if (count_i2c_clk == 3)
    begin
      i2c_clk         <= ~i2c_clk;
      count_i2c_clk   <= 0;
    end
    else
    begin
      count_i2c_clk <= count_i2c_clk + 1;
    end
  end

  assign scl = (en_scl == 0)? 1 : i2c_clk;
 
  ////////////////////////////////////////////////////////////////////
  // propagazione di questi segnali utili durante tutta una transazione

  always_ff @(posedge HCLK, negedge HRESET)
  begin
    if(HRESET)
    begin
      s_apb_addr  <= 0;
      paddr       <= 0;
      pwdata      <= 0;
      psel        <= 0;
      penable     <= 0;
      pwrite      <= 0;
    end
  begin
    if (PSEL && PENABLE && state == IDLE && PADDR [4:2] == SLAVE_ADDR)
    begin
      s_apb_addr  = PADDR[4:2];
      paddr       = PADDR;
      pwdata      = PWDATA[7:0];
      psel        = 1;
      penable     = 1;
      if (PWRITE) begin
        pwrite    = 1;
      end
    end
    else if (psel && penable && state == STOP && sda && scl) begin
      s_apb_addr  = 0;
      // paddr       = 0;
      // pwdata      = 0;
      psel        = 0;
      penable     = 0;
      if (pwrite) begin
        pwrite    = 0;
      end 
    end
  end
end


//////////////////////////////////////////////
// metà logica di valid signal

assign start_valid = (psel && penable) ? 0 : 1;

  ////////////////////////////////////////////
  // Macchina a stati

  always @(negedge HRESET)
  begin
    state           <= IDLE;
    write_enable    <= 1;  // nella condizione di reset vogliamo che il controller abbia il controllo di sda
    valid           <= 1;
  end

  always @(posedge i2c_clk)
  begin   //aggiornamento stato sul fronte positivo
    //always @(start, count_addr, sda, data_out[7])
    case (state)
      IDLE:
      begin
        addr        <= {paddr, ~pwrite};
        w_data      <= pwdata;
        // r_data          <= 0;
        // r_data      <= 0;
        if (start)
        begin
          state <= START;
        end
        else
        begin
          state <= IDLE;
        end
      end

      START:
      begin
        counter     <= 7;
        state       <= SEND_ADDR;
      end

      SEND_ADDR:
        if (counter == 0)
        begin
          state <= WAIT_ACK;
        end
        else
        begin
          state <= SEND_ADDR; //si può anche sottointendere poichè si instaurerebbe un latch
          counter <= counter - 1;
        end

      WAIT_ACK:
      begin
        counter <= 7;    // inizializziamo il counter per il suo utilizzo su WRITE_DATA o READ_DATA
        if (sda == 0)
        begin
          if (addr[0])
          begin
            state <= READ_DATA;     // PWRITE = 1 => scrive lo slave
          end
          else
          begin
            state <= WRITE_DATA;    // PWRITE = 0 => scrive il controller
          end
        end
        else
          state <= STOP; // oppure IDLE
      end

      READ_DATA:
      begin
        r_data[counter] <= sda;
        if (counter == 0)
        begin
          state <= READ_ACK;
        end
        else
          counter <= counter - 1;
      end

      READ_ACK:
      begin
        state <= STOP;
      end

      WRITE_DATA:
      begin
        if(counter == 0)
        begin
          state <= WRITE_ACK;
        end
        else
          counter <= counter - 1;
      end

      WRITE_ACK:
      begin
        state <= STOP;
      end

      STOP:
      begin
        state <= IDLE;
      end

    endcase
  end

  always @(negedge i2c_clk)
  begin   //aggiornamento uscite sul fronte negativo
    write_enable    <= 0;
    en_scl          <= 0;
    // PREADY          <= 0;
    valid           <= 0;
    // scl_down        <= 0;
    stop            <= 0;

    case (state)
      IDLE:
      begin
        write_enable    <= 1;
        //    PREADY          <= 0;
        sda_out         <= 1;
      end                           

      START:
      begin
        write_enable    <= 1;
        sda_out         <= 0;   // sda_out varia sui fronti negativi
      end

      SEND_ADDR:
      begin
        write_enable    <= 1;
        en_scl          <= 1;
        sda_out         <= addr[counter];
      end

      WAIT_ACK:
      begin
        en_scl          <= 1;
        // scl_down        <= 1;
      end

      READ_DATA:
      begin
        en_scl          <= 1;
      end

      READ_ACK:
      begin
        // valid           <= 1;
        write_enable    <= 1;
        sda_out         <= 0;
        en_scl          <= 1;
      end

      WRITE_DATA:
      begin
        write_enable    <= 1;
        sda_out         <= w_data[counter];
        en_scl          <= 1;
      end

      WRITE_ACK:
      begin
        en_scl          <= 1;
      end

      STOP:
      begin
        valid           <= 1;   
        write_enable    <= 1;
        stop            <= 1;
        sda_out         <= 1;
      end
    endcase
  end

endmodule

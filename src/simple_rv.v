`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: UNICA, DIEE
// Engineers: Matteo Matta & Andrea Carrus
// 
// Create Date: 19.09.2024 10:25:45
// Design Name: simple-rv
// Module Name: simple-rv-core
// Project Name: PSI-RV
// Target Devices: 180 nm
//
// Description: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module simple_rv(
    input clk, rst,
    // instruction memory interface
    input [31:0] imem_data, 
    output [31:0] imem_addr,
    output imem_en,
    // data memory interface
    output [31:0] dmem_data_in, 
    input  [31:0] dmem_data_out,
    output [31:0] dmem_addr,
    output dmem_wen,
    output dmem_ren
    );


// FETCH SIGNALS

reg     [31:0] pc;
reg     [31:0] ir;

// DECODE SIGNALS

wire I; 
wire R;
wire S;
wire B;
wire L;
wire JL;
wire JLR;

wire    [6:0]  opcode; 
wire    [6:0]  funct7;
wire    [2:0]  funct3;
wire    [4:0]  rd, rs1, rs2;
wire    [31:0] imm_i, imm_br, imm_ld, imm_st, imm_jal, imm_jalr, imm_shift;
reg     [31:0] imm;

reg     [4:0] operation; 

wire    reg_file_wen; 

wire    DM_wen, DM_ren;

reg     [31:0] reg_file [0:31];

reg    [31:0] rs1_value;
reg    [31:0] aux_rs2_value;
wire    [31:0] rs2_value;

reg     [31:0] offset;

wire    sel_mux1;
wire    sel_mux2;
wire    sel_mux3;

// EXECUTION SIGNALS

wire    [31:0] alu_in_1;
reg     [31:0] alu_in_2;
reg     [31:0] alu_out;

reg     taken;

reg [31:0] target_address;

// DATA MEMORY SIGNALS


// WRITE BACK SIGNALS

reg    [31:0] wb;

// PIPELINE STAGES 
// IF/ID stage
reg [31:0] id_pc;
reg [31:0] id_ir;

reg PC_en;
reg IR_en;
reg IDEX_en;
reg IFID_en;
reg IFID_flush;

// ID/EX stage
reg [4:0]  ex_operation; 
reg [31:0] ex_rs1_value; 
reg [31:0] ex_aux_rs2_value;
reg [4:0]  ex_rd;        
reg [31:0] ex_offset;    
reg [31:0] ex_imm;       
reg        ex_DM_wen;    
reg        ex_DM_ren;    
reg        ex_sel_mux1;  
reg        ex_sel_mux2;  
reg        ex_sel_mux3;  
reg [31:0] ex_pc;
reg        ex_I;
reg        ex_R;
reg        ex_L;
reg        ex_S;
reg        ex_B;
reg        ex_JL;
reg        ex_JLR;
reg [4:0]  ex_rs1;
reg [4:0]  ex_rs2;
reg        ex_reg_file_wen;
reg [31:0] ex_ir;

// EX/MEM stage
reg [31:0] mem_alu_out;  
reg [31:0] mem_target_address;
reg        mem_taken; 
reg        mem_DM_wen;
reg        mem_DM_ren;
reg [4:0]  mem_rd;
reg        mem_sel_mux3;
reg        mem_sel_mux2;
reg [31:0] mem_aux_rs2_value;
reg        mem_L;
reg        mem_JL;
reg        mem_JLR;
reg        mem_reg_file_wen;
reg [31:0] mem_fw2_rs2;
reg        mem_B;
reg [31:0] mem_ir;

// MEM/WB
reg [31:0] wb_alu_out;    
reg [4:0]  wb_rd;         
reg [31:0] wb_dmem_data_out; 
reg        wb_sel_mux2;
reg        wb_reg_file_wen;
reg        wb_B;
reg        wb_L;   
reg        wb_JL;
reg        wb_JLR;   
reg        wb_taken;  
reg [31:0] wb_ir; 

// FORWARDING UNIT
reg [31:0] fw1_rs1;
reg [31:0] fw2_rs2;

wire sel_fw1;
wire sel_fw2;

// DECODE SIGNALS
// opcode decode
parameter I_TYPE    = 7'b0010011;
parameter R_TYPE    = 7'b0110011;
parameter STORE_IR  = 7'b0100011;
parameter BRANCH_IR = 7'b1100011;
parameter LOAD_IR   = 7'b0000011;  
parameter JAL_IR    = 7'b1101111; 
parameter JALR_IR   = 7'b1100111;

// funct3 parameter
parameter F3_ADD    = 3'b000;
parameter F3_AND    = 3'b111;
parameter F3_OR     = 3'b110;
parameter F3_XOR    = 3'b100;
parameter F3_SRL    = 3'b101;
parameter F3_SLL    = 3'b001;
parameter F3_SRA    = 3'b101;
parameter F3_ADDI   = 3'b000;
parameter F3_SUB    = 3'b000;
parameter F3_ANDI   = 3'b111;
parameter F3_ORI    = 3'b110;
parameter F3_XORI   = 3'b100;
parameter F3_SLLI   = 3'b001;
parameter F3_SRLI   = 3'b101;
parameter F3_SRAI   = 3'b101;
parameter F3_LW     = 3'b010;
parameter F3_SW     = 3'b010;
parameter F3_BEQ    = 3'b000;  
parameter F3_BNE    = 3'b001;  
parameter F3_BLT    = 3'b100;  
parameter F3_BGE    = 3'b101;  
parameter F3_BLTU   = 3'b110;  
parameter F3_BGEU   = 3'b111;

// operation parameter
parameter ADD       = 5'b00000;  // 0x0
parameter SLLI      = 5'b00001;  // 0x1
parameter SLL       = 5'b00010;  // 0x2
parameter SRAI      = 5'b00011;  // 0x3
parameter SRA       = 5'b00100;  // 0x4
parameter SRLI      = 5'b00101;  // 0x5
parameter SRL       = 5'b00110;  // 0x6
parameter XOR       = 5'b00111;  // 0x7
parameter OR        = 5'b01000;  // 0x8
parameter AND       = 5'b01001;  // 0x9
parameter ADDI      = 5'b01010;  // 0xA
parameter XORI      = 5'b01011;  // 0xB
parameter ORI       = 5'b01100;  // 0xC
parameter ANDI      = 5'b01101;  // 0xD
parameter SUB       = 5'b01110;  // 0xE
parameter LW        = 5'b01111;  // 0xF
parameter SW        = 5'b10000;  // 0x10
parameter JAL       = 5'b10001;  // 0x11
parameter JALR      = 5'b10010;  // 0x12
parameter BEQ       = 5'b10011;  // 0x13 
parameter BNE       = 5'b10100;  // 0x14 
parameter BLT       = 5'b10101;  // 0x15 
parameter BGE       = 5'b10110;  // 0x16 
parameter BLTU      = 5'b10111;  // 0x17 
parameter BGEU      = 5'b11000;  // 0x18
    
/////////////////////////////////
//     __      _       _       //
//    / _| ___| |_ ___| |__    //
//   | |_ / _ \ __/ __| '_ \   //
//   |  _|  __/ || (__| | | |  //
//   |_|  \___|\__\___|_| |_|  //
//                             //
/////////////////////////////////       
//        PC and IR logic      //
/////////////////////////////////

always @(posedge clk)
    if(rst)
        pc <= 0;
    else if(PC_en) begin
        if (sel_mux3)
            pc <= mem_target_address;
        else
            pc <= pc + 4;
    end

assign imem_en = 1'b1;
assign imem_addr = pc;

always @(posedge clk)
    if(rst)
        ir <= 0;
    else if (IR_en)
        ir <= imem_data;

///////////////////////////////////////////////        
//    _____ ______       __  _____ _____     //
//   |_   _|  ____|     / / |_   _|  __ \    //
//     | | | |__       / /    | | | |  | |   //
//     | | |  __|     / /     | | | |  | |   //
//    _| |_| |       / /     _| |_| |__| |   //
//   |_____|_|      /_/     |_____|_____/    //
//                                           //
///////////////////////////////////////////////
//              IF / ID register             //
///////////////////////////////////////////////
        
always @(posedge clk) begin
    if (rst) begin
        id_pc <= 0;
        id_ir <= 0;
    end
    else if (IFID_flush)
            id_ir <= 32'h00000013;  // NOP  
         else if (IFID_en) begin
                id_pc <= pc;
                id_ir <= ir;
            end
end
    
///////////////////////////////////////////
//        _                    _         //
//     __| | ___  ___ ___   __| | ___    //
//    / _` |/ _ \/ __/ _ \ / _` |/ _ \   //
//   | (_| |  __/ (_| (_) | (_| |  __/   //
//    \__,_|\___|\___\___/ \__,_|\___|   //
//                                       //
///////////////////////////////////////////
//    Control Unit and Register File     //
///////////////////////////////////////////

assign opcode       = id_ir[6:0];
assign rd           = id_ir[11:7];
assign rs1          = id_ir[19:15];
assign rs2          = id_ir[24:20];
assign imm_i        = {{20{id_ir[31]}}, id_ir[31:20]};
assign funct7       = id_ir[31:25];
assign funct3       = id_ir[14:12];
assign imm_br       = {{20{id_ir[31]}},id_ir[7], id_ir[30:25], id_ir[11:8],1'b0};
assign imm_ld       = {{20{id_ir[31]}}, id_ir[31:20]};
assign imm_st       = {{20{id_ir[31]}}, id_ir[31:25], id_ir[11:7]};
assign imm_jal      = {{12{id_ir[31]}}, id_ir[19:12], id_ir[20], id_ir[30:21], 1'b0};
assign imm_jalr     = {{20{id_ir[31]}}, id_ir[31:20]};
assign imm_shift    = {{27{id_ir[24]}}, id_ir[24:20]};

assign I  = (opcode == I_TYPE);
assign R  = (opcode == R_TYPE);
assign S  = (opcode == STORE_IR);
assign B  = (opcode == BRANCH_IR);
assign L  = (opcode == LOAD_IR);
assign JL  = (opcode == JAL_IR);
assign JLR = (opcode == JALR_IR);

assign reg_file_wen = I | R | L | JL | JLR ;

assign sel_mux1 = I | L | S;
assign sel_mux2 = L;
assign sel_mux3 = mem_taken | mem_JL | mem_JLR;

assign DM_wen = S;
assign DM_ren = L;

always @(funct7, funct3, opcode)
case (opcode) 
    R_TYPE:
        if (funct7 == 7'b0100000)
            case (funct3)
                F3_SRA : operation = SRA;
                F3_SUB : operation = SUB;
                default: operation = 5'bxxxxx;
            endcase
        else
            case (funct3) 
            F3_ADD : operation = ADD; 
            F3_AND : operation = AND; 
            F3_OR  : operation = OR; 
            F3_XOR : operation = XOR; 
            F3_SRL : operation = SRL; 
            F3_SLL : operation = SLL;
            default: operation = 5'bxxxxx;
        endcase
    I_TYPE:
        if (funct7 == 7'b0100000 && funct3 == F3_SRAI)
            operation = SRAI;
        else 
            case (funct3)
            F3_ADDI : operation = ADDI;
            F3_ANDI : operation = ANDI;
            F3_ORI  : operation = ORI;
            F3_XORI : operation = XORI;
            F3_SLLI : operation = SLLI;
            F3_SRLI : operation = SRLI;
            default : operation = 5'bxxxxx;
            endcase
    LOAD_IR:
        if (funct3 == F3_LW)
            operation = LW;
        else
            operation = 5'bxxxxx;
    STORE_IR:
        if (funct3 == F3_SW)
            operation = SW;
        else
            operation = 5'bxxxxx;
    BRANCH_IR:
        case (funct3)
        F3_BEQ:  operation = BEQ;
        F3_BNE:  operation = BNE;
        F3_BLT:  operation = BLT;
        F3_BGE:  operation = BGE;
        F3_BLTU: operation = BLTU;
        F3_BGEU: operation = BGEU;
        default: operation = 5'bxxxxx; // Default to NOP or error handling
    endcase
    JAL_IR:
        operation = JAL;
    JALR_IR:
        operation = JALR;
    
default: operation = 5'bxxxxx;
endcase

integer i;
always @(posedge clk)
    if(rst)
        for(i=0;i<32;i=i+1)
            reg_file[i] <= 0;
    else
        if(wb_reg_file_wen && wb_rd != 0) 
            reg_file[wb_rd] <= wb; // RF write port

always @(*) begin
    if (B)
        offset = imm_br;
    else if (JL)
        offset = imm_jal;
        else
            offset = imm_jalr + rs1_value;  
end

// assign rs1_value = reg_file[rs1];
// assign aux_rs2_value = reg_file[rs2];

always @(*) begin
    if(I)
        imm = imm_i;
    else if(L)
            imm = imm_ld;
         else if (S)
                imm = imm_st;
end

always @(*) begin
    if (I || R || B || S || L || JLR) begin
        // if (ex_reg_file_wen && (rs1 == ex_rd))
        //     rs1_value = alu_out;  
        // else
        if (mem_reg_file_wen && (rs1 == mem_rd))
           rs1_value = mem_alu_out;
        else
        if (wb_reg_file_wen && (rs1 == wb_rd))
            rs1_value = wb;
        else
            rs1_value = reg_file[rs1];
    end 
    else
        rs1_value = reg_file[rs1];
end

always @(*) begin
        if (R || B || S) begin
            // if (ex_reg_file_wen && (rs2 == ex_rd))
            //     aux_rs2_value = alu_out;
            // else
            if (mem_reg_file_wen && (rs2 == mem_rd))
                aux_rs2_value = mem_alu_out;
            else
                if (wb_reg_file_wen && (rs2 == wb_rd))
                    aux_rs2_value = wb;
                else
                    aux_rs2_value = reg_file[rs2];
        end else
            aux_rs2_value = reg_file[rs2];
end

///////////////////////////////////////////////////////////////    
//    _    _                        _   _    _       _ _     //
//   | |  | |                      | | | |  | |     (_) |    //
//   | |__| | __ _ ______ _ _ __ __| | | |  | |_ __  _| |_   // 
//   |  __  |/ _` |_  / _` | '__/ _` | | |  | | '_ \| | __|  //
//   | |  | | (_| |/ / (_| | | | (_| | | |__| | | | | | |_   //
//   |_|  |_|\__,_/___\__,_|_|  \__,_|  \____/|_| |_|_|\__|  //
//                                                           //
///////////////////////////////////////////////////////////////
//                      Hazard Unit                          //
///////////////////////////////////////////////////////////////

always @(*) begin
PC_en      = 1; // DEFAULT
IR_en      = 1;
IDEX_en    = 1; 
IFID_en    = 1;
IFID_flush = 0;  

    if (ex_L && (((rs1 == ex_rd) || (rs1 == mem_rd)))) begin    // LW stall in caso di RAW hazard
        PC_en       = 0;    // probabilmente la condizione "rs1 == mem_rd" non è necessaria in quanto le due istruzioni sono già distanziate da un ciclo di clock
        IR_en       = 0;        
        IDEX_en     = 0;
        IFID_en     = 0;
        IFID_flush  = 0;
        if (ex_ir == mem_ir) begin    // libera la pipeline dallo stall di LW
            PC_en       = 1;
            IR_en       = 1;
            IDEX_en     = 1;
            IFID_en     = 1;
            IFID_flush  = 0;
        end
    end
         else
    if (B || JL || JLR) begin   // DECODE STAGE
        PC_en      = 0;
        IR_en      = 0;
        IDEX_en    = 1;
        IFID_en    = 1; // don't care
        IFID_flush = 1;
    end else if (ex_B || ex_JL || ex_JLR) begin    // EXECUTE STAGE
        PC_en      = 0;
        IR_en      = 0;
        IDEX_en    = 1; // 0
        IFID_en    = 1; // don't care
        IFID_flush = 1; 
    end else 
		if (mem_taken || wb_taken || mem_JL || mem_JLR || wb_JL || wb_JLR) begin   // JL e JLR sono considerate come dei branch SEMPRE TAKEN
                    PC_en      = 1;
                    IR_en      = 1;                                                // Il SALTO è in MEM. Al ciclo di clock successivo PC, viene aggiornato con l'istruzione a cui saltare (PC_en),
                    IDEX_en    = 0;                                                // e ir si aggiorna dell'istruzione relativa al vecchio valore di PC (inutile, IR_en)
                    IFID_en    = 0;                                                // Successivamente, il salto va in WB. Al ciclo di clock successivo, PC viene aggiornato col PC_new + 4 (PC_en)e ir 
                    IFID_flush = 0;                                                // si aggiorna al valore del PC aggiornato dal salto (IR_en)
            end	                                                                   // Nel mentre i registri IDEX e IFID sono congelati, in modo che non avanzino istruzioni errate corrispondenti    
end


///////////////////////////////////////////////        
//    _____ _____        __  ________   __   //
//   |_   _|  __ \      / / |  ____\ \ / /   //
//     | | | |  | |    / /  | |__   \ V /    //
//     | | | |  | |   / /   |  __|   > <     //
//    _| |_| |__| |  / /    | |____ / . \    //
//   |_____|_____/  /_/     |______/_/ \_\   //
//                                           //
///////////////////////////////////////////////
//              ID / EX register             //
///////////////////////////////////////////////

always @(posedge clk) begin
    if(rst) begin
        // CU signals
        ex_operation        <= 0;
        ex_rs1_value        <= 0;
        ex_aux_rs2_value    <= 0;
        ex_rd               <= 0;
        ex_offset           <= 0;
        ex_imm              <= 0;
        ex_DM_wen           <= 0;
        ex_DM_ren           <= 0;
        ex_sel_mux1         <= 0;
        ex_sel_mux2         <= 0;
        ex_sel_mux3         <= 0;
        ex_I                <= 0;
        ex_R                <= 0;
        ex_L                <= 0;
        ex_B                <= 0;
        ex_S                <= 0;
        ex_JL               <= 0;
        ex_JLR              <= 0;
        ex_reg_file_wen     <= 0;
        //no-CU signals
        ex_rs1              <= 0;
        ex_rs2              <= 0;        
        ex_pc               <= 0;
        ex_ir               <= 0;
    end
    else if (IDEX_en) begin
        // CU signals
        ex_operation        <= operation;
        ex_rs1_value        <= rs1_value;
        ex_aux_rs2_value    <= aux_rs2_value;
        ex_rd               <= rd;
        ex_offset           <= offset;
        ex_imm              <= imm;
        ex_DM_wen           <= DM_wen;
        ex_DM_ren           <= DM_ren;
        ex_sel_mux1         <= sel_mux1;
        ex_sel_mux2         <= sel_mux2;
        ex_sel_mux3         <= sel_mux3;
        ex_I                <= I;
        ex_R                <= R;
        ex_L                <= L;
        ex_B                <= B;
        ex_S                <= S;
        ex_JL               <= JL;
        ex_JLR              <= JLR;
        ex_reg_file_wen     <= reg_file_wen;
        //no-CU signals
        ex_rs1              <= rs1;
        ex_rs2              <= rs2;
        ex_pc               <= id_pc;
        ex_ir               <= id_ir;
    end
end
   
/////////////////////////////////////////////////////
//                             _   _               //
//     _____  _____  ___ _   _| |_(_) ___  _ __    //
//    / _ \ \/ / _ \/ __| | | | __| |/ _ \| '_ \   //
//   |  __/>  <  __/ (__| |_| | |_| | (_) | | | |  //
//    \___/_/\_\___|\___|\__,_|\__|_|\___/|_| |_|  //
//                                                 //       
/////////////////////////////////////////////////////
//                The ALU is here                  //
/////////////////////////////////////////////////////

assign sel_fw1 = ex_I || ex_R || ex_B || ex_S || ex_L || ex_JLR;
assign sel_fw2 = ex_R || ex_B || ex_S;

// Forwarding Unit 1
always @(*) begin
    if (sel_fw1) begin
        if (mem_reg_file_wen && (ex_rs1 == mem_rd) && ~mem_L)   // ~mem_L evita il forwarding per istruzioni LOAD stallate che scrivono un mem_rd == ex_rs1 (il cui dato risulterebbe non ancora aggiornato)
        begin   
                fw1_rs1 = mem_alu_out;
            end
        else if (wb_reg_file_wen && (ex_rs1 == wb_rd))
            begin
                // if (wb_L)                       // AMMETTENDO CHE LA SELEZIONE DEL MUX2 SIA GIUSTA, QUESTA PARTE E' RIDONDANTE.
                //     fw1_rs1 = wb_dmem_data_out; // forward dalla fase di wb->ex per RAW Hazard da parte di istruzioni su una load
                // else
                fw1_rs1 = wb;
            end
            else
                fw1_rs1 = ex_rs1_value;
    end else
        fw1_rs1 = ex_rs1_value;
end

// Forwarding Unit 2
always @(*) begin
    if (sel_fw2) begin
        if (mem_reg_file_wen && (ex_rs2 == mem_rd) && ~mem_L)
        begin
            fw2_rs2 = mem_alu_out; 
        end
        else if (wb_reg_file_wen && (ex_rs2 == wb_rd))
            begin
                // if(wb_L)
                //     fw2_rs2 = wb_dmem_data_out;
                // else 
                    fw2_rs2 = wb;
            end
            else
                fw2_rs2 = ex_aux_rs2_value;
    end else
        fw2_rs2 = ex_aux_rs2_value;
end

assign rs2_value = (ex_operation == SUB) ? (~fw2_rs2) + 1 : fw2_rs2;

assign alu_in_1 = fw1_rs1;

always @(*) begin
    if (ex_sel_mux1)
        alu_in_2 = ex_imm;
    else
        alu_in_2 = rs2_value;   //per esempio: if (B) alu_in_2 = rs2_value;
    end

always @(*) begin
    case (ex_operation) 

        ADD     : alu_out = alu_in_1 + alu_in_2;
        SUB     : alu_out = alu_in_1 + alu_in_2;
        ADDI    : alu_out = alu_in_1 + alu_in_2;
//        SUBI     : alu_out = alu_in_1 + alu_in_2; //non esiste nell'ISA
        AND     : alu_out = alu_in_1 & alu_in_2;
        ANDI    : alu_out = alu_in_1 & alu_in_2;
        OR      : alu_out = alu_in_1 | alu_in_2;
        ORI     : alu_out = alu_in_1 | alu_in_2;
        XOR     : alu_out = alu_in_1 ^ alu_in_2;
        XORI    : alu_out = alu_in_1 ^ alu_in_2;
        SRL     : alu_out = alu_in_1 >> alu_in_2[4:0];
        SRLI    : alu_out = alu_in_1 >> alu_in_2[4:0];
        SLL     : alu_out = alu_in_1 << alu_in_2[4:0];
        SLLI    : alu_out = alu_in_1 << alu_in_2[4:0];
        SRA     : alu_out = $signed(alu_in_1) >>> alu_in_2[4:0];    // In verilog i segnali sono trattati di default come unsigned
                                                                    // Uno shift come numero unsigned farebbe shiftare in modo errato
                                                                    // es. SRA(1010) di 1 -> 0101 in quanto non vedrebbe il MSB come
                                                                    // bit di segno ma vedrebbe uno zero come bit di segno
        SRAI    : alu_out = $signed(alu_in_1) >>> alu_in_2[4:0];
        LW      : alu_out = alu_in_1 + alu_in_2;
        SW      : alu_out = alu_in_1 + alu_in_2;
        JAL     : alu_out = 0;                          // prevediamo dunque che la ALU abbia il segnale "pc" come ingresso
        JALR    : alu_out = alu_in_1 + alu_in_2;        // lasciamo pc e non "pc +4" perchè quando il
                                                        // segnale operation viene riconosciuto come JL o JLR
                                                        //  sono già passati due cicli di clock e il PC si
                                                        // è incrementato di 8 invece che di 4. Quindi      
        default : alu_out = 32'b0;
    endcase
end

always @(*) begin
    case (ex_operation) 
        BEQ     : taken = (alu_in_1 == alu_in_2) ? ex_B : 1'b0;
        BNE     : taken = (alu_in_1 != alu_in_2) ? ex_B : 1'b0;
        BLT     : taken = (alu_in_1 < alu_in_2) ? ex_B : 1'b0;
        BGE     : taken = (alu_in_1 >= alu_in_2) ? ex_B : 1'b0;
        BLTU    : taken = ($unsigned(alu_in_1) < $unsigned(alu_in_2)) ? ex_B : 1'b0;
        BGEU    : taken = ($unsigned(alu_in_1) >= $unsigned(alu_in_2)) ? ex_B : 1'b0;   //JAL e JLAR non prevedono un segnale di taken

        default : taken = 1'b0;
    endcase
end

always @(*) begin
    if (taken)  // OCCHIO !!!
        target_address = ex_pc + ex_offset - 4; // altrimenti al posto di mettere - 4 prendiamo il segnale "pc" invece che "ex_pc"
    else if(ex_JL)
        target_address = ex_pc + ex_offset - 4;
    else if (ex_JLR)
            target_address = ex_offset;
        else
            target_address = target_address;
end

//////////////////////////////////////////////////////   
//   ________   __      __  __  __ ______ __  __    //
//  |  ____\ \ / /     / / |  \/  |  ____|  \/  |   //
//  | |__   \ V /     / /  | \  / | |__  | \  / |   //
//  |  __|   > <     / /   | |\/| |  __| | |\/| |   //
//  | |____ / . \   / /    | |  | | |____| |  | |   //
//  |______/_/ \_\ /_/     |_|  |_|______|_|  |_|   //
//                                                  //
//////////////////////////////////////////////////////
//                  EX / MEM register               //
//////////////////////////////////////////////////////

always @(posedge clk) begin
    if(rst) begin
        // ALU signals
        mem_alu_out             <= 0;
        mem_target_address      <= 0;
        mem_taken               <= 0;
        // no-ALU signals
        mem_aux_rs2_value       <= 0;
        mem_DM_wen              <= 0;
        mem_DM_ren              <= 0;
        mem_rd                  <= 0;
        mem_sel_mux2            <= 0;
        mem_sel_mux3            <= 0;
        mem_JL                  <= 0;
        mem_JLR                 <= 0;
        mem_reg_file_wen        <= 0;
        mem_fw2_rs2             <= 0;
        mem_B                   <= 0;
        mem_L                   <= 0;  
        mem_ir                  <= 0;      
    end
    else begin
        // From-EX signals
        mem_alu_out        <= alu_out;
        mem_target_address <= target_address;
        mem_taken          <= taken;
        mem_fw2_rs2        <= fw2_rs2;
        // not From-EX signals
         mem_aux_rs2_value  <= ex_aux_rs2_value;
        mem_DM_wen         <= ex_DM_wen;
        mem_DM_ren         <= ex_DM_ren;
        mem_rd             <= ex_rd;
        mem_sel_mux2       <= ex_sel_mux2;
        mem_sel_mux3       <= ex_sel_mux3;
        mem_JL             <= ex_JL;
        mem_JLR            <= ex_JLR;
        mem_reg_file_wen   <= ex_reg_file_wen;
        mem_B              <= ex_B;
        mem_L              <= ex_L;
        mem_ir             <= ex_ir;
        
    end
end

///////////////////////////////////////////////////////                      
//      _       _                                    //
//   __| | __ _| |_ __ _   _ __ ___   ___ _ __ ___   //
//  / _` |/ _` | __/ _` | | '_ ` _ \ / _ \ '_ ` _ \  //
// | (_| | (_| | || (_| | | | | | | |  __/ | | | | | //
//  \__,_|\__,_|\__\__,_| |_| |_| |_|\___|_| |_| |_| //
//                                                   //
///////////////////////////////////////////////////////    

assign dmem_addr    = mem_alu_out;     
assign dmem_data_in = mem_fw2_rs2;
assign dmem_wen     = mem_DM_wen;
assign dmem_ren     = mem_DM_ren;


//////////////////////////////////////////////////////////////
//    __  __ ______ __  __       __ __          ______      //
//   |  \/  |  ____|  \/  |     / / \ \        / /  _ \     //
//   | \  / | |__  | \  / |    / /   \ \  /\  / /| |_) |    //
//   | |\/| |  __| | |\/| |   / /     \ \/  \/ / |  _ <     //
//   | |  | | |____| |  | |  / /       \  /\  /  | |_) |    //
//   |_|  |_|______|_|  |_| /_/         \/  \/   |____/     //
//                                                          //
//////////////////////////////////////////////////////////////
//                    MEM / WB register                     //
//////////////////////////////////////////////////////////////

always @(posedge clk) begin
    if(rst) begin
        wb_alu_out          <= 0;
        wb_rd               <= 0;
        wb_dmem_data_out    <= 0;
        wb_sel_mux2         <= 0;
        wb_reg_file_wen     <= 0;
        wb_B                <= 0;
        wb_L                <= 0;
        wb_JL               <= 0;
        wb_JLR              <= 0;
        wb_taken            <= 0;
        wb_ir               <= 0;
    end
    else begin
        wb_alu_out          <= mem_alu_out;
        wb_rd               <= mem_rd;
        wb_dmem_data_out    <= dmem_data_out;
        wb_sel_mux2         <= mem_sel_mux2;
        wb_reg_file_wen     <= mem_reg_file_wen;
        wb_B                <= mem_B;
        wb_L                <= mem_L;
        wb_taken            <= mem_taken;
        wb_JL               <= mem_JL;
        wb_JLR              <= mem_JLR;
        wb_ir               <= mem_ir;
    end    
end

////////////////////////////////////////////////////////
//                 _ _         _                _     //
//  __      ___ __(_) |_ ___  | |__   __ _  ___| | __ //
//  \ \ /\ / / '__| | __/ _ \ | '_ \ / _` |/ __| |/ / //
//   \ V  V /| |  | | ||  __/ | |_) | (_| | (__|   <  //
//    \_/\_/ |_|  |_|\__\___| |_.__/ \__,_|\___|_|\_\ //
//                                                    //
////////////////////////////////////////////////////////
// Here you can model the 32-bit signal wb to either  //
// write the ALU output in the register file or       //
// something else                                     //
////////////////////////////////////////////////////////
                      
always @(*) begin
    if (wb_sel_mux2)
        wb = wb_dmem_data_out;
    else
        wb = wb_alu_out;    
end
    
endmodule
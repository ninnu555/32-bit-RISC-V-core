# RISCV core
A 32-bit RISC-V architecture processor.  
This project was devoloped as the final deliverable for the "Design of integrated systems" course during my Master's degree.  
The core's functionality has been validated using custom and C-generated assembly programs designed to control various peripherals, including GPIO, I2C, a timer, and a digital filter module with hardware acceleration.

## Features
RV32I implemented instructions:  
* I-type: JALR, LW, ADDI, SLTI, XORI, ORI, ANDI, SLLI, SRLI, SRAI;
* R-type: ADD, SUB, SLL, XOR, SRL, SRA, OR, AND
* B-type: BEQ, BNE, BLT, BGE, BLTU, BGEU
* S-type: SW
* J-type: JAL

5-stages pipeline with:  
* Data forwarding  
* Branch handling  
* Pipeline stall

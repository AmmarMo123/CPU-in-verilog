// Control Unit Module (yC2)
// This module generates control signals for register writing, ALU source selection,
// memory reading, memory writing, and memory-to-register selection based on instruction
// types set from yC1.
module yC2 (
    output RegWrite,   // Signal to enable writing to a register
    output ALUSrc,     // Signal to select ALU source (immediate vs. register)
    output MemRead,    // Signal to enable memory read operation
    output MemWrite,   // Signal to enable memory write operation
    output Mem2Reg,    // Signal to select data from memory to write to register
    input isStype,     // Input: S-Type instruction indicator
    input isRtype,     // Input: R-Type instruction indicator
    input isItype,     // Input: I-Type instruction indicator
    input isLw,        // Input: Load Word (LW) instruction indicator
    input isjump,      // Input: Jump (UJ-Type) instruction indicator
    input isbranch     // Input: Branch (SB-Type) instruction indicator
);

    // ALUSrc is 1 for I-Type and sw and UJ
    // Mem2Reg is 1 for lw
    // RegWrite is 1 for R-format and lw and UJ
    // MemRead is 1 for lw
    // MemWrite is 1 for sw

    nor (ALUSrc, isRtype, isbranch);			//0 - do calculation; 1 - add immediate
    nor (RegWrite, isStype, isbranch);			//need to write to a register

    assign Mem2Reg = isLw;
    assign MemRead = isLw;
    assign MemWrite = isStype;
endmodule
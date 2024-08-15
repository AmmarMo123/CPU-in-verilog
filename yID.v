// Instruction Decode Module
// This module decodes the instruction and generates necessary control signals and outputs.
module yID (
    output [31:0] rd1,      // Data read from the first register
    output [31:0] rd2,      // Data read from the second register
    output [31:0] immOut,   // Immediate value output (based on instruction type)
    output [31:0] jTarget,  // Jump target address (for UJ-Type instructions)
    output [31:0] branch,   // Branch address (for SB-Type instructions)
    input [31:0] ins,       // Instruction input
    input [31:0] wd,        // Write data to register file
    input RegWrite,         // Register write enable
    input clk               // Clock signal
);

    // Internal signals
    wire [19:0] zeros, ones;
    wire [11:0] zerosj, onesj;
    wire [31:0] imm, saveImm;

    // Register file instantiation
    // Reads data from registers based on instruction's rs1, rs2, and rd fields
    rf myRF (
        .rd1(rd1),
        .rd2(rd2),
        .rs1(ins[19:15]),
        .rs2(ins[24:20]),
        .wn(ins[11:7]),
        .wd(wd),
        .clk(clk),
        .w(RegWrite)
    );

    // Immediate value extraction and sign extension
    assign imm[11:0] = ins[31:20];  // Extract immediate from instruction
    assign zeros = 20'h00000;        // Zero constant for sign extension
    assign ones = 20'hFFFFF;         // One constant for sign extension
    yMux #(20) se (
        .z(imm[31:12]),
        .a(zeros),
        .b(ones),
        .c(ins[31])
    );

    // Immediate value for S-Type instructions
    assign saveImm[11:5] = ins[31:25]; // Upper part of immediate
    assign saveImm[4:0] = ins[11:7];   // Lower part of immediate
    yMux #(20) saveImmSe (
        .z(saveImm[31:12]),
        .a(zeros),
        .b(ones),
        .c(ins[31])
    );

    // Select the appropriate immediate value based on instruction type
    yMux #(32) immSelection (
        .z(immOut),
        .a(imm),
        .b(saveImm),
        .c(ins[5])
    );

    // Branch address calculation for SB-Type instructions
    assign branch[11] = ins[31];          // Sign bit for upper part of branch address
    assign branch[10] = ins[7];           // Middle part of branch address
    assign branch[9:4] = ins[30:25];      // Upper part of branch address
    assign branch[3:0] = ins[11:8];       // Lower part of branch address
    yMux #(20) bra (
        .z(branch[31:12]),
        .a(zeros),
        .b(ones),
        .c(ins[31])
    );

    // Jump target address calculation for UJ-Type instructions
    assign zerosj = 12'h000;             // Zero constant for UJ-Type
    assign onesj = 12'hFFF;              // One constant for UJ-Type
    assign jTarget[19] = ins[31];        // Sign bit for upper part of jump target
    assign jTarget[18:11] = ins[19:12];  // Middle part of jump target
    assign jTarget[10] = ins[20];        // Additional part of jump target
    assign jTarget[9:0] = ins[30:21];    // Lower part of jump target
    yMux #(12) jum (
        .z(jTarget[31:20]),
        .a(zerosj),
        .b(onesj),
        .c(jTarget[19])
    );

endmodule
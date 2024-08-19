// Top-level Processor Module (yChip)
// This module integrates all the stages of the processor pipeline: Instruction Fetch (IF), 
// Instruction Decode (ID), Execution (EX), Data Memory (DM), and Write-Back (WB). 
// It controls the flow of instructions through the processor.
module yChip (
    output [31:0] ins,       // Current instruction (Added only for testing purposes)
    output [31:0] rd2,       // Data from the second register operand (Added only for testing purposes)
    output [31:0] wb,        // Data to be written back to the register file (Added only for testing purposes)
    input [31:0] entryPoint, // Starting address for the program
    input INT,               // Interrupt signal
    input clk                // Clock signal
);

    // Internal signals
    wire [31:0] PCin, PC, wd, rd1, imm, PCp4, z, branch, jTarget, memOut;
    wire [2:0] op, funct3;
    wire [6:0] opCode;
    wire zero;
    wire isStype, isRtype, isItype, isLw, isjump, isbranch;
    wire RegWrite, ALUSrc, MemRead, MemWrite, Mem2Reg;
    wire [1:0] ALUop;

    // Instruction Fetch (IF) Stage
    yIF myIF(
        .ins(ins), 
        .PC(PC), 
        .PCp4(PCp4), 
        .PCin(PCin), 
        .clk(clk)
    );

    // Instruction Decode (ID) Stage
    yID myID(
        .rd1(rd1), 
        .rd2(rd2), 
        .immOut(imm), 
        .jTarget(jTarget), 
        .branch(branch), 
        .ins(ins), 
        .wd(wd), 
        .RegWrite(RegWrite), 
        .clk(clk)
    );

    // Execute (EX) Stage
    yEX myEx(
        .z(z), 
        .zero(zero), 
        .rd1(rd1), 
        .rd2(rd2), 
        .imm(imm), 
        .op(op), 
        .ALUSrc(ALUSrc)
    );

    // Data Memory (DM) Stage
    yDM myDM(
        .memOut(memOut), 
        .exeOut(z), 
        .rd2(rd2), 
        .clk(clk), 
        .MemRead(MemRead), 
        .MemWrite(MemWrite)
    );

    // Write-Back (WB) Stage
    yWB myWB(
        .wb(wb), 
        .exeOut(z), 
        .memOut(memOut), 
        .Mem2Reg(Mem2Reg)
    );

    // Program Counter (PC) Module
    yPC myPC(
        .PCin(PCin), 
        .PC(PC), 
        .PCp4(PCp4), 
        .INT(INT), 
        .entryPoint(entryPoint), 
        .branchImm(branch), 
        .jImm(jTarget), 
        .zero(zero), 
        .isbranch(isbranch), 
        .isjump(isjump)
    );

    // Control Unit - Part 1: Decode opCode, generate 
    // ALUSrc, RegWrite, Mem2Reg, MemRead, MemWrite, isjump
    // and isbranch signals
    assign opCode = ins[6:0];
    yC1 myC1(
        .isStype(isStype), 
        .isRtype(isRtype), 
        .isItype(isItype), 
        .isLw(isLw), 
        .isjump(isjump), 
        .isbranch(isbranch), 
        .opCode(opCode)
    );

    // Control Unit - Part 2: Set ALUSrc, RegWrite, Mem2Reg,
    // MemRead and MemWrite based on the output of yC1
    yC2 myC2(
        .RegWrite(RegWrite), 
        .ALUSrc(ALUSrc), 
        .MemRead(MemRead), 
        .MemWrite(MemWrite), 
        .Mem2Reg(Mem2Reg), 
        .isStype(isStype), 
        .isRtype(isRtype), 
        .isItype(isItype), 
        .isLw(isLw), 
        .isjump(isjump), 
        .isbranch(isbranch)
    );

    // ALU Control Unit, yC3 and yC4 generate the opcode collabaratively.
    // yC3, is responsible for non-R-type, and yC4 takes care of R-types.
    yC3 myC3(
        .ALUop(ALUop), 
        .isRtype(isRtype), 
        .isbranch(isbranch)
    );

    assign funct3 = ins[14:12];
    yC4 myC4(
        .op(op), 
        .ALUop(ALUop), 
        .funct3(funct3)
    );

    // Write-back Data Assignment
    assign wd = wb;
endmodule
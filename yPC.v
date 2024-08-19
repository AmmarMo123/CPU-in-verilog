// Program Counter (PC) Module
// This module computes the next value of the Program Counter (PC) based on different control signals 
// such as interrupts, branching, and jumping.
module yPC (
    output [31:0] PCin,        // Next PC value to be used in the next clock cycle
    input [31:0] PC,           // Current PC value
    input [31:0] PCp4,         // PC + 4 (next sequential instruction address)
    input INT,                 // Interrupt signal
    input [31:0] entryPoint,   // Entry point address (used for interrupt handling)
    input [31:0] branchImm,    // Immediate value for branching
    input [31:0] jImm,         // Immediate value for jumping
    input zero,                // Zero flag (used to determine branch condition)
    input isbranch,            // Branch control signal
    input isjump               // Jump control signal
);

    // Internal signals
    wire [31:0] branchImmX4;
    wire [31:0] jImmX4;
    wire [31:0] jImmX4PPCp4;
    wire [31:0] bTarget;
    wire [31:0] choiceA, choiceB;
    wire doBranch;
    wire zf;
    wire [2:0] add;

    // Assign constants
    assign enable = 1'b1;    // Enable the PC register to update
    assign a = 32'h0004;     // Constant value 4 for PC increment
    assign add = 3'b010;     // ALU operation: 010 for addition

    // Shifting branch immediate left by 2 bits (multiply by 4)
    assign branchImmX4[31:2] = branchImm[29:0];
    assign branchImmX4[1:0] = 2'b00;

    // Shifting jump immediate left by 2 bits (multiply by 4)
    assign jImmX4[31:2] = jImm[29:0];
    assign jImmX4[1:0] = 2'b00;

    // Calculate branch target address by adding PC and branchImmX4
    yAlu bALU (
        .z(bTarget), 
        .ex(zf), 
        .a(PC), 
        .b(branchImmX4), 
        .op(add)
    );

    // Calculate jump target address by adding PC and jImmX4
    yAlu jALU (
        .z(jImmX4PPCp4), 
        .ex(zf), 
        .a(PC), 
        .b(jImmX4), 
        .op(add)
    );

    // Determine whether to branch (isbranch AND zero)
    and decide_doBranch(doBranch, isbranch, zero);

    // First Mux: Choose between sequential PC (PCp4) and branch target (bTarget)
    yMux #(32) mux1 (
        .z(choiceA), 
        .a(PCp4), 
        .b(bTarget), 
        .c(doBranch)
    );

    // Second Mux: Choose between previous choice (choiceA) and jump target (jImmX4PPCp4)
    yMux #(32) mux2 (
        .z(choiceB), 
        .a(choiceA), 
        .b(jImmX4PPCp4), 
        .c(isjump)
    );

    // Third Mux: Choose between previous choice (choiceB) and entry point (entryPoint) based on interrupt
    yMux #(32) mux3 (
        .z(PCin), 
        .a(choiceB), 
        .b(entryPoint), 
        .c(INT)
    );
endmodule
// Instruction Fetch (IF) Module
// This module handles fetching instructions from memory, updating the program counter (PC), and calculating the next PC.
module yIF (
    output [31:0] ins,   // Fetched instruction from memory
    output [31:0] PC,    // Current program counter value
    output [31:0] PCp4,  // Program counter + 4 (next instruction address)
    input [31:0] PCin,   // Input for updating the program counter
    input clk            // Clock signal
);

    // Internal signals
    wire zero;
    wire read, write;
    wire enable;
    wire [31:0] a;
    wire [31:0] memIn;
    wire [2:0] add;

    // Assign constants
    assign enable = 1'b1;    // Enable the PC register to update
    assign a = 32'h0004;     // Constant value 4 for PC increment
    assign add = 3'b010;     // ALU operation: 010 for addition
    assign read = 1'b1;      // Enable memory read operation
    assign write = 1'b0;     // Disable memory write operation

    // Program Counter Register
    register #(32) pcReg (
        .q(PC),
        .d(PCin),
        .clk(clk),
        .enable(enable)
    );

    // Instruction Memory
    mem insMem (
        .memOut(ins),
        .address(PC),
        .memIn(memIn),
        .clk(clk),
        .memRead(read),
        .memWrite(write)
    );

    // ALU for calculating PC + 4
    yAlu myAlu (
        .z(PCp4),
        .ex(zero),
        .a(a),
        .b(PC),
        .op(add)
    );
endmodule
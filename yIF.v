// Instruction Fetch Module
// This module fetches instructions from memory and computes the next PC value.
module yIF (
    output [31:0] ins,    // Instruction fetched from memory
    output [31:0] PCp4,   // Program Counter + 4 (next address)
    input [31:0] PCin,    // Program Counter input
    input clk             // Clock signal
);

    // Internal signals
    wire zerflag;
    wire [31:0] regOut;

    // Register to hold the current Program Counter value
    register #(32) my_reg (
        .q(regOut),
        .d(PCin),
        .clk(clk),
        .enable(1'b1)
    );

    // ALU to compute the next Program Counter value (PC + 4)
    // The ALU adds 4 to the current PC value to compute PCp4
    yAlu pc_inc (
        .z(PCp4),
        .ex(zerflag),
        .a(regOut),
        .b(32'd4),
        .op(3'b010)
    );

    // Memory module to fetch the instruction from memory
    // The memIn port is not connected as this module only reads data
    mem data (
        .memOut(ins),
        .address(regOut),
        .memIn(32'b0),
        .clk(clk),
        .memRead(1'b1),
        .memWrite(1'b0)
    );
endmodule
// Execute Stage Module
// This module performs ALU operations based on the inputs and control signals.
module yEX (
    output [31:0] z,     // Result of the ALU operation
    output zero,         // Zero flag indicating if the result is zero
    input [31:0] rd1,    // First operand for ALU operation
    input [31:0] rd2,    // Second operand for ALU operation (or immediate value)
    input [31:0] imm,    // Immediate value used for certain operations
    input [2:0] op,      // ALU operation code
    input ALUSrc         // Control signal to select between rd2 and imm
);

    // Internal signal to hold the result of the mux operation
    wire [31:0] muxOut;

    // Mux to select between rd2 and immediate value based on ALUSrc
    // When ALUSrc is 1, imm is selected; otherwise, rd2 is selected
    yMux #(32) reg_or_imm (
        .z(muxOut),
        .a(rd2),
        .b(imm),
        .c(ALUSrc)
    );

    // ALU to perform the operation based on the opcode and selected operand
    yAlu execAlu (
        .z(z),
        .ex(zero),
        .a(rd1),
        .b(muxOut),
        .op(op)
    );

endmodule
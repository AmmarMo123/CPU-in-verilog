// ALU Control Unit (yC4)
// This module generates the final ALU operation code (op) based on ALUop and funct3 inputs.
module yC4 (
    output [2:0] op,       // 3-bit ALU operation code
    input [1:0] ALUop,     // 2-bit ALU operation code generated by yC2
    input [2:0] funct3     // 3-bit function code from the instruction
);

    wire f21out, f10out, upperAndOut, notALU, notf3;

    // Bit 2 of the ALU operation code
    xor f21 (f21out, funct3[2], funct3[1]);
    and upperAnd(upperAndOut, ALUop[1], f21out);
    or upperOr(op[2], ALUop[0], upperAndOut);

    // Bit 1 of the ALU operation code
    not ALUop1no(notALU, ALUop[1]);
    not f3no(notf3, funct3[1]);
    or lowerOr(op[1], notALU, notf3);

    // Bit 0 of the ALU operation code
    xor f10 (f10out, funct3[1], funct3[0]);
    and lowerAnd(op[0], ALUop[1], f10out);
endmodule
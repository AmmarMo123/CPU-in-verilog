// Control Unit Module (yC3)
// This module generates the ALUop, which determines non-R-type instructions 
module yC3 (
    output [1:0] ALUop, // 2-bit ALU operation code
    input isRtype,      // Input: R-Type instruction indicator
    input isbranch      // Input: Branch instruction indicator
);

    // Generate the ALU operation code based on the instruction type
    assign ALUop[1] = isRtype;
    assign ALUop[0] = isbranch;
endmodule
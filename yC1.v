// Control Unit Module (yC1)
// This module decodes the operation code (opCode) to generate control signals
// for different instruction types: S-Type, R-Type, I-Type, LW, JUMP, and BRANCH.
module yC1 (
    output isStype,    // Signal indicating S-Type instruction
    output isRtype,    // Signal indicating R-Type instruction
    output isItype,    // Signal indicating I-Type instruction
    output isLw,       // Signal indicating LW instruction
    output isjump,     // Signal indicating JUMP instruction (UJ-Type)
    output isbranch,   // Signal indicating BRANCH instruction (SB-Type)
    input [6:0] opCode // 7-bit operation code input
);

    // opCode
    // lw      0000011
    // I-Type  0010011
    // R-Type  0110011
    // SB-Type 1100011 beq
    // UJ-Type 1101111 jal
    // S-Type  0100011

    // Detect UJ-type
    assign isjump= opCode[3];

    // Detect lw
    or lw_detect(lwor, opCode[6], opCode[5], opCode[4], opCode[3], opCode[2]); not (isLw, lwor);

    // Select between S-Type and I-Type
    xor s_i_xor(ISselect, opCode[6], opCode[3], opCode[2], opCode[1], opCode[0]);
    and s_and(isStype, ISselect, opCode[5]);
    and i_and(isItype, ISselect, opCode[4]);

    // Detect R-Type
    and r_and(isRtype, opCode[5], opCode[4]);

    // Select between JAL and Branch
    and JB_and(JBselect, opCode[6], opCode[5]); // SB and UJ are the only ones with bits 5 and 6
    not branch_detect(sbz, opCode[3]); // SB has 0 in bits 2 and 3
    and branch_and(isbranch, JBselect, sbz);
endmodule
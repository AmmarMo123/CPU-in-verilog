// Write-Back Module
// This module selects the appropriate value to be written back to the register file
// based on the Mem2Reg control signal.
module yWB (
    output [31:0] wb,      // Data to be written back to the register file
    input [31:0] exeOut,   // ALU result (potential write-back data)
    input [31:0] memOut,   // Data read from memory (potential write-back data)
    input Mem2Reg          // Control signal to select between exeOut and memOut
);

    // Instantiate a 2-to-1 multiplexer to select the write-back data
    // based on the Mem2Reg signal.
    yMux #(32) writeback (
        .z(wb),
        .a(exeOut),
        .b(memOut),
        .c(Mem2Reg)
    );
endmodule
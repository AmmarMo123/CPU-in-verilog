// Data Memory Module
// This module interfaces with the memory module to handle read and write operations.
module yDM (
    output [31:0] memOut,   // Data read from memory
    input [31:0] exeOut,    // Address for read/write operations
    input [31:0] rd2,       // Data to be written to memory
    input clk,              // Clock signal
    input MemRead,          // Memory read enable
    input MemWrite          // Memory write enable
);

    // Instantiate the memory module
    mem data_mem (
        .memOut(memOut),
        .address(exeOut),
        .memIn(rd2),
        .clk(clk),
        .memRead(MemRead),
        .memWrite(MemWrite)
    );
endmodule
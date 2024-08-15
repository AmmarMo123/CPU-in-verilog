// 4-to-1 Multiplexer Module
// Selects one of four inputs (a0, a1, a2, a3) based on the 2-bit select signal c
module yMux4to1 (
    output [SIZE-1:0] z,   // Select value for one of the inputs
    input [SIZE-1:0] a0,   // First data input
    input [SIZE-1:0] a1,   // Second data input
    input [SIZE-1:0] a2,   // Third data input
    input [SIZE-1:0] a3,   // Fourth data input
    input [1:0] c          // Select input: determines which data input to choose
);

    parameter SIZE = 2;    // Parameter defining the width of the data inputs and output

    // Internal signals
    wire [SIZE-1:0] zLo;
    wire [SIZE-1:0] zHi;

    // 2-to-1 multiplexer for the lower half of the inputs (a0 and a1)
    yMux #(SIZE) lo (
        .z(zLo),
        .a(a0),
        .b(a1),
        .c(c[0])
    );

    // 2-to-1 multiplexer for the upper half of the inputs (a2 and a3)
    yMux #(SIZE) hi (
        .z(zHi),
        .a(a2),
        .b(a3),
        .c(c[0])
    );

    // 2-to-1 multiplexer to select between the results of the lower and upper multiplexers
    yMux #(SIZE) final (
        .z(z),
        .a(zLo),
        .b(zHi),
        .c(c[1])
    );

endmodule
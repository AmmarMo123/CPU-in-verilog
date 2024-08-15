// N-to-1 Multiplexer Module
module yMux #(
    parameter SIZE = 2  // Parameter to define the bit-width of the multiplexer
)(
    output [SIZE-1:0] z,  // Output vector of the multiplexer
    input [SIZE-1:0] a,   // Input vector a
    input [SIZE-1:0] b,   // Input vector b
    input c               // Select signal
);

    // Instantiate a vector of 2-to-1 multiplexers
    // Each bit of the output z is selected based on the corresponding bits of a, b, and the select signal c
    yMux1 mine[SIZE-1:0](
        .z(z),
        .a(a),
        .b(b),
        .c(c)
    );

endmodule
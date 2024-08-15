// Arithmetic Unit Module
// Performs addition if ctrl = 0, and subtraction if ctrl = 1
module yArith (
    output [31:0] z,     // 32-bit output result
    output cout,         // Carry-out bit for addition
    input [31:0] a,      // 32-bit input operand a
    input [31:0] b,      // 32-bit input operand b
    input ctrl           // Control signal: 0 for addition, 1 for subtraction
);

    // Internal signals
    wire [31:0] notB;
    wire [31:0] tmp;
    wire cin;

    // Generate the bitwise negation of b
    not b_not[31:0] (
        notB,
        b
    );

    // Multiplexer to select between b and ~b based on ctrl signal
    // If ctrl = 0, tmp = b (for addition)
    // If ctrl = 1, tmp = ~b (for subtraction)
    yMux #(.SIZE(32)) my_mux[31:0] (
        .z(tmp),
        .a(b),
        .b(notB),
        .c(ctrl)
    );

    // Assign carry-in for the adder from ctrl
    assign cin = ctrl;

    // 32-bit adder to compute z = a + tmp
    // cout is the carry-out bit
    yAdder my_add[31:0] (
        .z(z),
        .cout(cout),
        .a(a),
        .b(tmp),
        .cin(cin)
    );

endmodule
// 32-bit Ripple Carry Adder Module
module yAdder(
    output [31:0] z,    // 32-bit Sum output
    output cout,        // Carry-out output
    input [31:0] a,     // 32-bit Input operand a
    input [31:0] b,     // 32-bit Input operand b
    input cin           // Carry-in input
);

    // Internal signals
    wire [31:0] in;
    wire [31:0] out;

    // Instantiate 32 1-bit full adders
    yAdder1 mine[31:0](
        .z(z),
        .cout(out),
        .a(a),
        .b(b),
        .cin(in)
    );

    // Connect carry-in to the first adder and propagate carry-out to the next adder
    assign in[0] = cin;           // Initialize carry-in for the least significant bit
    assign in[31:1] = out[30:0]; // Propagate carry-out from each adder to the next adder's carry-in

    // The final carry-out is the carry-out of the most significant bit
    assign cout = out[31];

endmodule
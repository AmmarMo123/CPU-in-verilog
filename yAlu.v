// ALU Module
// Performs various arithmetic and logic operations based on the op code
module yAlu (
    output [31:0] z,   // Result of the operation
    output ex,         // Zero flag: set if result z is zero
    input [31:0] a,    // Operand a
    input [31:0] b,    // Operand b
    input [2:0] op     // Operation code - op=000: z=a AND b, op=001: z=a|b, op=010: z=a+b, op=110: z=a-b
);

    // Internal signals
    wire [15:0] z16;
    wire [7:0] z8;
    wire [3:0] z4;
    wire [1:0] z2;
    wire z1;
    wire cout;
    wire [31:0] zAnd, zOr, zArith, slt;
    wire condition;
    wire [31:0] aSubB;

    // Perform bitwise AND operation
    and ab_and[31:0] (
        zAnd,
        a,
        b
    );

    // Perform bitwise OR operation
    or ab_or[31:0] (
        zOr,
        a,
        b
    );

    // Compute the zero flag
    or or16[15:0] (z16, z[15:0], z[31:16]);
    or or8[7:0] (z8, z16[7:0], z16[15:8]);
    or or4[3:0] (z4, z8[3:0], z8[7:4]);
    or or2[1:0] (z2, z4[1:0], z4[3:2]);
    or or1 (z1, z2[1], z2[0]);
    not zero_not(ex, z1);

    // Perform set-less-than operation
    xor slt_xor (
        condition,
        a[31],
        b[31]
    );

    yArith slt_arith (
        .z(aSubB),
        .cout(cout),
        .a(a),
        .b(b),
        .ctrl(1'b1)
    );

    yMux1 my_mux_slt (
        .z(slt[0]),
        .a(aSubB[31]),
        .b(a[31]),
        .c(condition)
    );

    // Perform arithmetic operations (add or subtract)
    yArith ab_arith[31:0] (
        .z(zArith),
        .cout(cout),
        .a(a),
        .b(b),
        .ctrl(op[2])
    );

    // Select between the results based on the op code
    yMux4to1 #(.SIZE(32)) my_mux (
        .z(z),
        .a0(zAnd),
        .a1(zOr),
        .a2(zArith),
        .a3(slt),
        .c(op[1:0])
    );

endmodule
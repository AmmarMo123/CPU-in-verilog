// 1-bit Full Adder Module
module yAdder1(
    output z,      // Sum output
    output cout,   // Carry-out output
    input a,       // Input bit a
    input b,       // Input bit b
    input cin      // Carry-in input
);

    // Internal signals
    wire tmp;
    wire outL;
    wire outR;

    // Compute the sum (z) using XOR gates
    xor (tmp, a, b);
    xor (z, cin, tmp);

    // Compute the carry-out (cout) using AND and OR gates
    and (outL, a, b);
    and (outR, tmp, cin);
    or (cout, outR, outL);

endmodule
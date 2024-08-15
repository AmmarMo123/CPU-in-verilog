// 2-to-1 1 bit Multiplexer Module
// Selects between inputs a and b based on control signal c
module yMux1 (
    output z,    // Output of the multiplexer
    input a,     // Input a
    input b,     // Input b
    input c      // Control signal: 0 selects a, 1 selects b
);

    // Internal signals
    wire notC;
    wire upper;
    wire lower;

    // Generate the negation of control signal c
    not my_not (
        notC,
        c
    );

    // Compute upper as a AND NOT c
    and upperAnd (
        upper,
        a,
        notC
    );

    // Compute lower as b AND c
    and lowerAnd (
        lower,
        b,
        c
    );

    // Compute final output z as (upper OR lower)
    or my_or (
        z,
        upper,
        lower
    );

endmodule